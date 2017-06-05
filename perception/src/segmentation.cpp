#include "perception/segmentation.h"

#include "pcl_conversions/pcl_conversions.h"
#include <iostream>
#include <string>

#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/common/common.h"
#include "pcl/segmentation/extract_clusters.h"

#include "geometry_msgs/Pose.h"
#include "simple_grasping/shape_extraction.h"
#include "perception/box_fitter.h"
#include "shape_msgs/SolidPrimitive.h"
#include "perception/object_recognizer.h"

#include "pcl_ros/transforms.h"
#include "tf/transform_listener.h"

#include "perception/crop.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"
#include "pcl/common/common.h"

#include "perception/typedefs.h"
#include <tf/transform_datatypes.h>

using namespace std;
namespace perception {
  void SegmentSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices, pcl::ModelCoefficients::Ptr coeff) {
    pcl::PointIndices indices_internal;
    pcl::SACSegmentation<PointC> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud);
    Eigen::Vector3f axis;
    axis << 0, 0, 1;
    seg.setAxis(axis);
    seg.setEpsAngle(pcl::deg2rad(10.0));

    seg.segment(indices_internal, *coeff);

    double distance_above_plane;
    ros::param::param("distance_above_plane", distance_above_plane, 0.005);

    // Build custom indices that ignores points above the plane.
    for (size_t i = 0; i < cloud->size(); ++i) {
      const PointC& pt = cloud->points[i];
      float val = coeff->values[0] * pt.x + coeff->values[1] * pt.y +
                  coeff->values[2] * pt.z + coeff->values[3];
      if (val <= distance_above_plane) {
        indices->indices.push_back(i);
      }
    }

    if (indices->indices.size() == 0) {
      //ROS_ERROR("Unable to find surface.");
      return;
    }
  }

  void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, geometry_msgs::Pose* pose, geometry_msgs::Vector3* dimensions) {
    PointC min_pcl;
    PointC max_pcl;
    pcl::getMinMax3D<PointC>(*cloud, min_pcl, max_pcl);
    ROS_INFO("min: %f, max: %f", min_pcl.x, max_pcl.x);
    pose->position.x = (max_pcl.x + min_pcl.x)/2;
    pose->position.y = (max_pcl.y + min_pcl.y)/2;
    pose->position.z = (max_pcl.z + min_pcl.z)/2;
    dimensions->x = (max_pcl.x - min_pcl.x);
    dimensions->y = (max_pcl.y - min_pcl.y);
    dimensions->z = (max_pcl.z - min_pcl.z);
  }

  void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr surface_indices, std::vector<pcl::PointIndices>* object_indices) {
    pcl::ExtractIndices<PointC> extract;

    pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices());
    extract.setInputCloud(cloud);
    extract.setIndices(surface_indices);
    extract.setNegative(true);
    extract.filter(above_surface_indices->indices);

    double cluster_tolerance;
    int min_cluster_size, max_cluster_size;
    // ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.02);
    // ros::param::param("ec_min_cluster_size", min_cluster_size, 30);
    // ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);
    ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
    ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
    ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);

    pcl::EuclideanClusterExtraction<PointC> euclid;
    euclid.setInputCloud(cloud);
    euclid.setIndices(above_surface_indices);
    euclid.setClusterTolerance(cluster_tolerance);
    euclid.setMinClusterSize(min_cluster_size);
    euclid.setMaxClusterSize(max_cluster_size);
    euclid.extract(*object_indices);

    size_t min_size = std::numeric_limits<size_t>::max();
    size_t max_size = std::numeric_limits<size_t>::min();
    for (size_t i = 0; i < object_indices->size(); ++i) {
      size_t cluster_size = (*object_indices)[i].indices.size();
      if (cluster_size < min_size) {
        min_size = cluster_size;
      }
      if (cluster_size > max_size) {
        max_size = cluster_size;
      }
    }

    ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
             object_indices->size(), min_size, max_size);
  }



  void SegmentTabletopScene(PointCloudC::Ptr cloud, std::vector<Object>* objects) {
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
    SegmentSurface(cloud, table_inliers, coeff);

    // extract objects above the tray cropped cloud and publish to above_surface
    PointCloudC::Ptr cloud_out(new PointCloudC);
    pcl::ExtractIndices<PointC> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(table_inliers);
    extract.setNegative(false);
    extract.filter(*cloud_out);

    std::vector<pcl::PointIndices> object_indices;
    SegmentSurfaceObjects(cloud, table_inliers, &object_indices);

    extract.setInputCloud(cloud);
    extract.setNegative(false);

    for (size_t i = 0; i < object_indices.size(); ++i) {
      // Reify indices into a point cloud of the object.
      pcl::PointIndices::Ptr indices(new pcl::PointIndices);
      *indices = object_indices[i];
      extract.setIndices(indices);
      PointCloudC::Ptr object_cloud(new PointCloudC());
      extract.filter(*object_cloud);

      PointCloudC::Ptr extract_out_object(new PointCloudC());
      shape_msgs::SolidPrimitive object_shape;
      geometry_msgs::Pose object_pose;
      FitBox(*object_cloud, coeff, *extract_out_object, object_shape, object_pose);

      Object object;
      object.cloud = object_cloud;
      object.pose = object_pose;
      object.dimensions.x = object_shape.dimensions[0];
      object.dimensions.y = object_shape.dimensions[1];
      object.dimensions.z = object_shape.dimensions[2];
      objects->push_back(object);
    }
  }

  Segmenter::Segmenter(const ros::Publisher& surface_points_pub, const ros::Publisher& marker_pub, const ObjectRecognizer& recognizer)
    : surface_points_pub_(surface_points_pub), marker_pub_(marker_pub), recognizer_(recognizer) {}

  void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
    
    // Here we want to check the frame_id of the PointCloud
    // and do a transformation to the base_link frame.
    tf::TransformListener tf_listener;
    tf_listener.waitForTransform("base_link", msg.header.frame_id,
                                  ros::Time(0), ros::Duration(5.0));
    tf::StampedTransform transform;
    try {
      tf_listener.lookupTransform("base_link", msg.header.frame_id,
                                  ros::Time(0), transform);
    } catch (tf::LookupException& e) {
        std::cerr << e.what() << std::endl;
        return;
    } catch (tf::ExtrapolationException& e) {
        std::cerr << e.what() << std::endl;
        return;
    }    
    sensor_msgs::PointCloud2 cloud_transform_msg;
    pcl_ros::transformPointCloud("base_link", transform, msg, cloud_transform_msg);
    
    PointCloudC::Ptr cloud_transformed(new PointCloudC());
    pcl::fromROSMsg(cloud_transform_msg, *cloud_transformed);

    // Clean the transformed cloud by removing NANs
    PointCloudC::Ptr cloud(new PointCloudC());
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud_transformed, *cloud, index);

    // crop the cloud to the estimated tray area
    // PointCloudC::Ptr tray_cropped_cloud = CropTrayAndPublishMarker(cloud_transformed_cleaned);


    // Define the dimensions of the handle
    const double handle_x = 0.030;
    const double handle_y = 0.075;
    const double handle_z = 0.037;

    const double x_thres = 0.212;
    const double y_thres = 0.092;
    const double z_thres = 0.185;

    const double x_lo = handle_x * (1.0 - x_thres);
    const double x_hi = handle_x * (1.0 + x_thres);
    const double y_lo = handle_y * (1.0 - y_thres);
    const double y_hi = handle_y * (1.0 + y_thres);
    const double z_lo = handle_z * (1.0 - z_thres);
    const double z_hi = handle_z * (1.0 + z_thres);

    // Define if correct yaw is negative (true means yaw should be negative)
    const bool yawIsNegative = true;

    visualization_msgs::Marker table_marker;
    SegmentTableAndPublishMarker(cloud, table_marker);

    std::vector<Object> objects;
    SegmentTabletopScene(cloud, &objects);

    for (size_t i = 0; i < objects.size(); ++i) {
      const Object& object = objects[i];

      // Create a bounding box
      visualization_msgs::Marker object_marker;
      object_marker.ns = "objects";
      object_marker.id = i;
      object_marker.header.frame_id = "base_link";
      object_marker.type = visualization_msgs::Marker::CUBE;
      object_marker.pose = object.pose;
      object_marker.scale = object.dimensions;
      object_marker.color.r = 0;
      object_marker.color.g = 1;
      object_marker.color.b = 0;
      object_marker.color.a = 0.3;

      // Recongize the object
      string name;
      double confidence, object_x, object_y;
      recognizer_.Recognize(object, &name, &confidence);
      confidence = round(1000 * confidence) / 1000;
      object_x = std::min(object.dimensions.x, object.dimensions.y);
      object_y = std::max(object.dimensions.x, object.dimensions.y);

      std::stringstream ss;
      ss << name << " (" << confidence << ")" << " " << "(" << object_x << ", "  << object_y << ", " << object.dimensions.z << ")";

      // If the object is the handle, update the marker and name
      if (x_lo <= object_x && object_x <= x_hi &&
          y_lo <= object_y && object_y <= y_hi &&
          z_lo <= object.dimensions.z && object.dimensions.z <= z_hi)
      {
        // Identify 
        object_marker.color.r = 0;
        object_marker.color.g = 0;
        object_marker.color.b = 1;
        object_marker.id = 400;
        object_marker.ns = "tray handle";
        ss.str(std::string());
        std::cout << "x: " << object.dimensions.x << "y: " << object.dimensions.y << "z: " << object.dimensions.z << std::endl;
        ss << "handle" << " (" << confidence << ")" << " " << "(" << object_x << ", "  << object_y << ", " << object.dimensions.z << ")";
        std:cout << ss.str() << std::endl;
        std::cout << "handle pose: " << object_marker.pose << std::endl;

        // Get roll, pitch, yaw and print to stdout
        double x, y, z, w;
        double roll, pitch, yaw;
        double angle, angleshortestpath;
        tf::Vector3 vector;

        const double pi = boost::math::constants::pi<double>();
        x = object_marker.pose.orientation.x;
        y = object_marker.pose.orientation.y;
        z = object_marker.pose.orientation.z;
        w = object_marker.pose.orientation.w;
        tf::Quaternion q(x, y, z, w);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        angle = q.getAngle();
        angleshortestpath = q.getAngleShortestPath();
        vector = q.getAxis();

        // Fix yaw to prevent 180 degree rotation
        if (yawIsNegative and yaw > 0) {
          yaw -= pi;
        } else if (!yawIsNegative and yaw < 0) {
          yaw += pi;
        }
        if (object_x == object.dimensions.y) {
        std::cout << "flipped x and y" << std::endl;
          if (yawIsNegative) {
            yaw += pi/2;
          } else {
            yaw -= pi/2;
          }
        }
        q.setRPY(roll, pitch, yaw);
        object_marker.pose.orientation.x = q.x();
        object_marker.pose.orientation.y = q.y();
        object_marker.pose.orientation.z = q.z();
        object_marker.pose.orientation.w = q.w();
        std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
        std::cout << "angle: " << angle << ", angleshortestpath: " << angleshortestpath << ", vector: " << vector.getX() << ", " << vector.getY() << ", " << vector.getZ() << std::endl;

        // Verify it is update to expected Roll, Pitch, Yaw
        x = object_marker.pose.orientation.x;
        y = object_marker.pose.orientation.y;
        z = object_marker.pose.orientation.z;
        w = object_marker.pose.orientation.w;
        tf::Quaternion q2(x, y, z, w);
        tf::Matrix3x3 m2(q);
        m2.getRPY(roll, pitch, yaw);
        std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;

        // Adjust height of object marker
        object_marker.pose.position.z -= (object.dimensions.z - handle_z)/2.0;
        object_marker.scale.z = handle_z;

        // Fix the table marker so that it does not consume the handle
        table_marker.pose.position.z = object_marker.pose.position.z - (object_marker.scale.z / 2.0) - (table_marker.scale.z / 2.0);
      }

      // Create a marker for the recognition result
      visualization_msgs::Marker name_marker;
      name_marker.ns = "recognition";
      name_marker.id = i;
      name_marker.header.frame_id = "base_link";
      name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      name_marker.pose.position = object.pose.position;
      name_marker.pose.position.z += 0.1;
      name_marker.pose.orientation.w = 1;
      name_marker.scale.x = 0.025;
      name_marker.scale.y = 0.025;
      name_marker.scale.z = 0.025;
      name_marker.color.r = 0;
      name_marker.color.g = 0;
      name_marker.color.b = 1.0;
      name_marker.color.a = 1.0;
      name_marker.text = ss.str();

      // Publish the markers
      marker_pub_.publish(object_marker);
      marker_pub_.publish(name_marker);
    }

    marker_pub_.publish(table_marker);
    return;

    // int ret;

    // ret = SegmentSurfaceObjectsAndPublishMarkers(cloud);
    // if (ret == 1) {
    //   std::cerr << "Cannot segment table" << std::endl;
    //   return;
    // }
  }

  int Segmenter::SegmentTableAndPublishMarker(PointCloudC::Ptr input_cloud, visualization_msgs::Marker& table_marker) {
    pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
    SegmentSurface(input_cloud, table_inliers, coeff);
    if (table_inliers->indices.size() == 0) {
      return 1;
    }

    PointCloudC::Ptr table_cloud(new PointCloudC);
    pcl::ExtractIndices<PointC> extract;
    extract.setInputCloud(input_cloud);
    extract.setIndices(table_inliers);
    extract.filter(*table_cloud);

    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*table_cloud, msg_out);
    surface_points_pub_.publish(msg_out);

    // visualization_msgs::Marker table_marker;
    table_marker.ns = "table";
    table_marker.header.frame_id = "base_link";
    table_marker.type = visualization_msgs::Marker::CUBE;
    shape_msgs::SolidPrimitive table_shape;
    PointCloudC::Ptr extract_out_table(new PointCloudC());
    geometry_msgs::Pose table_pose;

    // simple_grasping::extractShape(*table_cloud, coeff, *extract_out_table, table_shape, table_pose);
    FitBox(*table_cloud, coeff, *extract_out_table, table_shape, table_pose);
    table_marker.pose = table_pose;
    table_marker.scale.x = table_shape.dimensions[0];
    table_marker.scale.y = table_shape.dimensions[1];
    table_marker.scale.z = table_shape.dimensions[2];
    table_marker.color.r = 0.5;
    table_marker.color.a = 0.8;
    // marker_pub_.publish(table_marker);
    return 0;
  }
  
  /*  Using rosparams tray_{min,max}_{x,y,z}, crops the input point cloud
   *  and sets a visual cube of the volume of the cropped cloud
   *  Returns a pointer to the cropped cloud
   */
  PointCloudC::Ptr Segmenter::CropTrayAndPublishMarker(PointCloudC::Ptr input_cloud) {
    string name_ = "tray";   
    double min_x, min_y, min_z, max_x, max_y, max_z;
    ros::param::param(name_ + "_min_x", min_x, 0.4);
    ros::param::param(name_ + "_min_y", min_y, -0.2);
    ros::param::param(name_ + "_min_z", min_z, 0.5);
    ros::param::param(name_ + "_max_x", max_x, 0.9);
    ros::param::param(name_ + "_max_y", max_y, 0.25);
    ros::param::param(name_ + "_max_z", max_z, 1.5);

    visualization_msgs::Marker tray_crop_marker;
    tray_crop_marker.ns = "tray_crop";
    tray_crop_marker.id = 69;
    tray_crop_marker.header.frame_id = "base_link";
    tray_crop_marker.type = visualization_msgs::Marker::CUBE;

    tray_crop_marker.color.r = 0.2;
    tray_crop_marker.color.g = 0.5;
    tray_crop_marker.color.b = 0.5;

    tray_crop_marker.pose.position.x = (min_x + max_x)/2;
    tray_crop_marker.pose.position.y = (min_y + max_y)/2;
    tray_crop_marker.pose.position.z = (min_z + max_z)/2;
    tray_crop_marker.pose.orientation.w = 1;

    tray_crop_marker.scale.x = (max_x - min_x);
    tray_crop_marker.scale.y = (max_y - min_y);
    tray_crop_marker.scale.z = (max_z - min_z);
    tray_crop_marker.color.a = 0.3;
    marker_pub_.publish(tray_crop_marker);

    Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
    Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);
    
    PointCloudC::Ptr tray_cropped_cloud(new PointCloudC());

    pcl::CropBox<PointC> crop;
    crop.setInputCloud(input_cloud);
    crop.setMin(min_pt);
    crop.setMax(max_pt);
    crop.filter(*tray_cropped_cloud);

    sensor_msgs::PointCloud2 tray_msg_out;
    pcl::toROSMsg(*tray_cropped_cloud, tray_msg_out);
    tray_crop_pub_.publish(tray_msg_out);
    return tray_cropped_cloud;
  }

}  // namespace perception
