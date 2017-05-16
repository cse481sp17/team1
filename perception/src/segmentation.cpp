#include "perception/segmentation.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <iostream>

#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <pcl/filters/extract_indices.h>
#include "pcl/common/common.h"
#include "visualization_msgs/Marker.h"
#include <pcl/segmentation/extract_clusters.h>

#include "geometry_msgs/Pose.h"
#include "simple_grasping/shape_extraction.h"
#include "shape_msgs/SolidPrimitive.h"

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

Segmenter::Segmenter(const ros::Publisher& surface_points_pub, const ros::Publisher& marker_pub, const ros::Publisher& above_surface_pub)
    : surface_points_pub_(surface_points_pub), marker_pub_(marker_pub), above_surface_pub_(above_surface_pub) {}

void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr nan_cloud(new PointCloudC());
  pcl::fromROSMsg(msg, *nan_cloud);

  PointCloudC::Ptr cloud(new PointCloudC());
  std:vector<int> v;
  pcl::removeNaNFromPointCloud(*nan_cloud, *cloud, v);

  pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
  SegmentSurface(cloud, table_inliers, coeff);
  if (table_inliers->indices.size() == 0) {
    return;
  }

  PointCloudC::Ptr table_cloud(new PointCloudC);
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(table_inliers);
  extract.filter(*table_cloud);

  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*table_cloud, msg_out);
  surface_points_pub_.publish(msg_out);

  visualization_msgs::Marker table_marker;
  table_marker.ns = "table";
  table_marker.header.frame_id = "base_link";
  table_marker.type = visualization_msgs::Marker::CUBE;
  //GetAxisAlignedBoundingBox(table_cloud, &table_marker.pose, &table_marker.scale);
  shape_msgs::SolidPrimitive table_shape;
  PointCloudC::Ptr extract_out_table(new PointCloudC());
  geometry_msgs::Pose table_pose;
  simple_grasping::extractShape(*table_cloud, coeff, *extract_out_table, table_shape, table_pose);
  table_marker.pose = table_pose;
  table_marker.scale.x = table_shape.dimensions[0];
  table_marker.scale.y = table_shape.dimensions[1];
  table_marker.scale.z = table_shape.dimensions[2];
  table_marker.color.r = 0.5;
  table_marker.color.a = 0.8;
  marker_pub_.publish(table_marker);
  // We are reusing the extract object created earlier in the callback.

  std::vector<pcl::PointIndices> object_indices;

  SegmentSurfaceObjects(cloud, table_inliers, &object_indices, coeff);
  PointCloudC::Ptr cloud_out(new PointCloudC);
  extract.setNegative(true);
  extract.filter(*cloud_out);
  pcl::toROSMsg(*cloud_out, msg_out);
  above_surface_pub_.publish(msg_out);

  for (size_t i = 0; i < object_indices.size(); ++i) {
    // Reify indices into a point cloud of the object.
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    *indices = object_indices[i];
    PointCloudC::Ptr object_cloud(new PointCloudC());
    // TODO: fill in object_cloud using indices
    pcl::ExtractIndices<PointC> extract_object_cloud;
    extract_object_cloud.setInputCloud(cloud);
    extract_object_cloud.setIndices(indices);
    extract_object_cloud.filter(*object_cloud);

    // Publish a bounding box around it.
    visualization_msgs::Marker object_marker;
    object_marker.ns = "objects";
    object_marker.id = i;
    object_marker.header.frame_id = "base_link";
    object_marker.type = visualization_msgs::Marker::CUBE;

    PointCloudC::Ptr extract_out_object(new PointCloudC());
    shape_msgs::SolidPrimitive object_shape;
    geometry_msgs::Pose::Ptr object_pose(new geometry_msgs::Pose());
    simple_grasping::extractShape(*object_cloud, coeff, *extract_out_object, object_shape, *object_pose);
    object_marker.pose = *object_pose;
    // need to set object_marker.scale
    object_marker.scale.x = object_shape.dimensions[0];
    object_marker.scale.y = object_shape.dimensions[1];
    object_marker.scale.z = object_shape.dimensions[2];
    std::cout << "object id: " << i << "x: " << object_marker.scale.x << "y: " << object_marker.scale.y << "z: " << object_marker.scale.z << std::endl;
   // GetAxisAlignedBoundingBox(object_cloud, &object_marker.pose,
   //                             &object_marker.scale);

    double x = 0.075; //0.0703085;
    double y = 0.030; //0.0312478;
    double z = 0.037; //0.0374182;
    double thres = 0.17;
    // double vol = x * y * z;
    // double vol_lo = x * y * z * (1.0 - thres) * (1.0 - thres) * (1.0 - thres);
    // double vol_hi = x * y * z * (1.0 + thres) * (1.0 + thres) * (1.0 + thres);
    double x_lo = x * (1.0 - thres);
    double y_lo = y * (1.0 - thres);
    double z_lo = z * (1.0 - thres);
    double x_hi = x * (1.0 + thres);
    double y_hi = y * (1.0 + thres);
    double z_hi = z * (1.0 + thres);


    double object_vol = object_marker.scale.x * object_marker.scale.y * object_marker.scale.z;

    if (x_lo <= object_marker.scale.x && object_marker.scale.x <= x_hi &&
        y_lo <= object_marker.scale.y && object_marker.scale.y <= y_hi &&
        z_lo <= object_marker.scale.z && object_marker.scale.z <= z_hi)
    {
      object_marker.color.r = 0;
      object_marker.color.g = 0;
      object_marker.color.b = 1;
    } else {
      object_marker.color.r = 0;
      object_marker.color.g = 1;
      object_marker.color.b = 0;
    }
    
    object_marker.color.a = 0.3;
    marker_pub_.publish(object_marker);
  }
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
}  // namespace perception
