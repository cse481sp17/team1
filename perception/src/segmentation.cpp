#include "perception/segmentation.h"

#include "pcl_conversions/pcl_conversions.h"
#include <iostream>
#include <string>

#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <pcl/filters/extract_indices.h>
#include "pcl/common/common.h"
#include <pcl/segmentation/extract_clusters.h>

#include "geometry_msgs/Pose.h"
#include "simple_grasping/shape_extraction.h"
#include "shape_msgs/SolidPrimitive.h"

#include "pcl_ros/transforms.h"
#include "tf/transform_listener.h"

#include "perception/crop.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"
#include "pcl/common/common.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;
using namespace std;
namespace perception {
void SegmentSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices, pcl::ModelCoefficients::Ptr coeff) {
  pcl::PointIndices indices_internal;
  pcl::SACSegmentation<PointC> seg;
  seg.setOptimizeCoefficients(true);
  // Search for a plane perpendicular to some axis (specified below).
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  // Set the distance to the plane for a point to be an inlier.
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud);

  // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
  Eigen::Vector3f axis;
  axis << 0, 0, 1;
  seg.setAxis(axis);
  seg.setEpsAngle(pcl::deg2rad(10.0));

  // coeff contains the coefficients of the plane:
  // ax + by + cz + d = 0
  //pcl::ModelCoefficients coeff;

  seg.segment(indices_internal, *coeff);

  double distance_above_plane;
  ros::param::param("distance_above_plane", distance_above_plane, 0.005);

  // Build custom indices that ignores points above the plane.
  /*for (size_t i = 0; i < cloud->size(); ++i) {
    const PointC& pt = cloud->points[i];
    float val = coeff.values[0] * pt.x + coeff.values[1] * pt.y +
                coeff.values[2] * pt.z + coeff.values[3];
    if (val <= distance_above_plane) {
      indices->indices.push_back(i);
    }
  }*/

  *indices = indices_internal;

  if (indices->indices.size() == 0) {
    ROS_ERROR("Unable to find surface.");
    return;
  }
}

void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions) {
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

void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr surface_indices,
                           std::vector<pcl::PointIndices>* object_indices,
                           pcl::ModelCoefficients::Ptr coeff) {
    pcl::ExtractIndices<PointC> extract;
    pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices());
    extract.setInputCloud(cloud);
    extract.setIndices(surface_indices);
    extract.setNegative(true);
    extract.filter(above_surface_indices->indices);

    ROS_INFO("There are %ld points above the table", above_surface_indices->indices.size());
    double cluster_tolerance;
    int min_cluster_size, max_cluster_size;
    ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.02);
    ros::param::param("ec_min_cluster_size", min_cluster_size, 30);
    ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);

    pcl::EuclideanClusterExtraction<PointC> euclid;
    euclid.setInputCloud(cloud);
    euclid.setIndices(above_surface_indices);
    euclid.setClusterTolerance(cluster_tolerance);
    euclid.setMinClusterSize(min_cluster_size);
    euclid.setMaxClusterSize(max_cluster_size);
    euclid.extract(*object_indices);

    // Find the size of the smallest and the largest object,
    // where size = number of points in the cluster
    size_t min_size = std::numeric_limits<size_t>::max();
    size_t max_size = std::numeric_limits<size_t>::min();
    for (size_t i = 0; i < object_indices->size(); ++i) {
      // TODO: implement this      
      size_t cluster_size = (*object_indices)[i].indices.size();
      if (min_size > cluster_size) {
        min_size = cluster_size;
      }
      if (max_size < cluster_size) {
        max_size = cluster_size;
      }
    }

    ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
             object_indices->size(), min_size, max_size);
}

int Segmenter::SegmentTableAndPublishMarker(PointCloudC::Ptr input_cloud, visualization_msgs::Marker::Ptr table_marker_ptr) {
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

  table_marker_ptr->ns = "table";
  table_marker_ptr->header.frame_id = "base_link";
  table_marker_ptr->type = visualization_msgs::Marker::CUBE;

  shape_msgs::SolidPrimitive table_shape;
  PointCloudC::Ptr extract_out_table(new PointCloudC());
  geometry_msgs::Pose table_pose;
  simple_grasping::extractShape(*table_cloud, coeff, *extract_out_table, table_shape, table_pose);
  table_marker_ptr->pose = table_pose;
  table_marker_ptr->scale.x = table_shape.dimensions[0];
  table_marker_ptr->scale.y = table_shape.dimensions[1];
  table_marker_ptr->scale.z = table_shape.dimensions[2];
  table_marker_ptr->color.r = 0.5;
  table_marker_ptr->color.a = 0.8;
  marker_pub_.publish(*table_marker_ptr);
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

Segmenter::Segmenter(const ros::Publisher& surface_points_pub, const ros::Publisher& marker_pub, const ros::Publisher& above_surface_pub, const ros::Publisher& tray_crop_pub)
    : surface_points_pub_(surface_points_pub), marker_pub_(marker_pub), above_surface_pub_(above_surface_pub), tray_crop_pub_(tray_crop_pub) {}

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
  PointCloudC::Ptr cloud_transformed_cleaned(new PointCloudC());
  std:vector<int> v;
  pcl::removeNaNFromPointCloud(*cloud_transformed, *cloud_transformed_cleaned, v);

  // crop the cloud to the estimated tray area
  PointCloudC::Ptr tray_cropped_cloud = CropTrayAndPublishMarker(cloud_transformed_cleaned);

  visualization_msgs::Marker::Ptr table_marker_ptr(new visualization_msgs::Marker());

  int ret = SegmentTableAndPublishMarker(cloud_transformed_cleaned, table_marker_ptr);
  if (ret == 1) {
    std::cerr << "Cannot segment table" << std::endl;
    return;
  }


  // Segment the surface above the cropped tray area
  pcl::PointIndices::Ptr tray_inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr tray_coeff(new pcl::ModelCoefficients());
  SegmentSurface(tray_cropped_cloud, tray_inliers, tray_coeff);
  if (tray_inliers->indices.size() == 0) {
    return;
  }

  // extract objects above the tray cropped cloud and publish to above_surface
  PointCloudC::Ptr above_surface_cloud(new PointCloudC);
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(tray_cropped_cloud);
  extract.setIndices(tray_inliers);
  extract.setNegative(true);
  extract.filter(*above_surface_cloud);
  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*above_surface_cloud, msg_out);
  above_surface_pub_.publish(msg_out);

  // Segment the surface objects above the tray crop
  std::vector<pcl::PointIndices> object_indices;
  SegmentSurfaceObjects(tray_cropped_cloud, tray_inliers, &object_indices, tray_coeff);

  for (size_t i = 0; i < object_indices.size(); ++i) {
    // Reify indices into a point cloud of the object.
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    *indices = object_indices[i];
    PointCloudC::Ptr object_cloud(new PointCloudC());
    // fill in object_cloud using indices
    pcl::ExtractIndices<PointC> extract_object_cloud;
    extract_object_cloud.setInputCloud(tray_cropped_cloud);
    extract_object_cloud.setIndices(indices);
    extract_object_cloud.filter(*object_cloud);

    // Publish a bounding box around it.
    visualization_msgs::Marker object_marker;
    object_marker.ns = "objects";
    object_marker.id = 2*i;
    object_marker.header.frame_id = "base_link";
    object_marker.type = visualization_msgs::Marker::CUBE;

    
    // Get object marker pose and scale
    PointCloudC::Ptr extract_out_object(new PointCloudC());
    shape_msgs::SolidPrimitive object_shape;
    geometry_msgs::Pose::Ptr object_pose(new geometry_msgs::Pose());
    simple_grasping::extractShape(*object_cloud, tray_coeff, *extract_out_object, object_shape, *object_pose);

    // Populate marker for bounding box
    object_marker.pose = *object_pose;

    // need to set object_marker.scale
    object_marker.scale.x = object_shape.dimensions[0];
    object_marker.scale.y = object_shape.dimensions[1];
    object_marker.scale.z = object_shape.dimensions[2];
    //std::cout << "object id: " << i << "x: " << object_marker.scale.x << "y: " << object_marker.scale.y << "z: " << object_marker.scale.z << std::endl;

    // Publish a text view above it
    visualization_msgs::Marker object_marker_text;
    object_marker_text.ns = "text";
    object_marker_text.id = 2*i + 1;
    object_marker_text.header.frame_id = "base_link";
    object_marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    // Populate marker for text box
    object_marker_text.pose = *object_pose;
    object_marker_text.text = boost::to_string(2*i + 1);
    // need to set object_marker.scale
    object_marker_text.scale.x = object_shape.dimensions[0];
    object_marker_text.scale.y = object_shape.dimensions[1];
    object_marker_text.scale.z = object_shape.dimensions[2];


   // std::cout << "object id: " << 2*i << "," << 2*i + 1 << " x: " << object_marker.scale.x << " y: " << object_marker.scale.y << " z: " << object_marker.scale.z << std::endl;

    double x = 0.075;
    double y = 0.030;
    double z = 0.037;

    double thres = 0.20;
    double z_thres = 0.275;

    double x_lo = x * (1.0 - thres);
    double x_hi = x * (1.0 + thres);
    double y_lo = y * (1.0 - thres);
    double y_hi = y * (1.0 + thres);
    double z_lo = z * (1.0 - z_thres);
    double z_hi = z * (1.0 + z_thres);

    /*std::cout << "x_lo: " << x_lo << " x_hi: " << x_hi << std::endl;
    std::cout << "y_lo: " << y_lo << " y_hi: " << y_hi << std::endl;
    std::cout << "z_lo: " << z_lo << " z_hi: " << z_hi << std::endl;
    */

    // Set the color for the bounding box
    if (x_lo <= object_marker.scale.x && object_marker.scale.x <= x_hi &&
        y_lo <= object_marker.scale.y && object_marker.scale.y <= y_hi &&
        z_lo <= object_marker.scale.z && object_marker.scale.z <= z_hi)
    {
      // Update object marker pose position to be exact z
      // table_marker.pose
      // table_marker.scale.z
      object_marker.pose.position.z = table_marker_ptr->pose.position.z + (table_marker_ptr->scale.z / 2.0) + (z / 2.0);
      object_marker.scale.z = z;
      object_marker.color.r = 0;
      object_marker.color.g = 0;
      object_marker.color.b = 1;
      object_marker.ns = "tray handle";
      std::cout << "handle pose: " << object_marker.pose << std::endl;

    } else {
      object_marker.color.r = 0;
      object_marker.color.g = 1;
      object_marker.color.b = 0;
    }

    object_marker.color.a = 0.3;

    // Set the color for the text box

    object_marker_text.color.r = 0;
    object_marker_text.color.g = 1;
    object_marker_text.color.b = 1;
    object_marker_text.color.a = 1;

    marker_pub_.publish(object_marker);
    marker_pub_.publish(object_marker_text);
  }
}
}  // namespace perception
