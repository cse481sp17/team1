#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "visualization_msgs/Marker.h"

#include "perception/object.h"
#include "perception/object_recognizer.h"

namespace perception {
// Finds the largest horizontal surface in the given point cloud.
// This is useful for adding a collision object to MoveIt.
//
// Args:
//  cloud: The point cloud to extract a surface from.
//  indices: The indices of points in the point cloud that correspond to the
//    surface. Empty if no surface was found.
void SegmentSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                    pcl::PointIndices::Ptr indices,
                    pcl::ModelCoefficients::Ptr coeff);

// Computes the axis-aligned bounding box of a point cloud.
//
// Args:
//  cloud: The point cloud
//  pose: The output pose. Because this is axis-aligned, the orientation is just
//    the identity. The position refers to the center of the box.
//  dimensions: The output dimensions, in meters.
void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               geometry_msgs::Pose* pose,
                               geometry_msgs::Vector3* dimensions);

void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr surface_indices,
                           std::vector<pcl::PointIndices>* object_indices,
                           pcl::ModelCoefficients::Ptr coeff);

void SegmentTabletopScene(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<Object>* objects);

class Segmenter {
 public:
  Segmenter(const ros::Publisher& surface_points_pub, const ros::Publisher& marker_pub, 
    const ros::Publisher& above_surface_pub, const ros::Publisher& tray_crop_pub, const ObjectRecognizer& recognizer);
  void Callback(const sensor_msgs::PointCloud2& msg);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr CropTrayAndPublishMarker(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);
  int SegmentTableAndPublishMarker(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, visualization_msgs::Marker::Ptr table_marker_ptr);
  int SegmentSurfaceObjectsAndPublishMarkers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);

 private:
  ros::Publisher surface_points_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher above_surface_pub_;
  ros::Publisher tray_crop_pub_;
  ObjectRecognizer recognizer_;
};
}  // namespace perception
