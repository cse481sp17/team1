#include "perception/crop.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"
#include "pcl/common/common.h"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/transforms.h"
#include "tf/transform_listener.h"
#include "rosbag/bag.h"
#include <string>
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
  Cropper::Cropper(const ros::Publisher& pub, const std::string& name) : pub_(pub), name_(name) {}

  void Cropper::Callback(const sensor_msgs::PointCloud2& msg) { 
      

  // here we want to check the frame_id of the pointcloud
  // and do a transformation to the base_link frame  
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
    PointCloudC::Ptr cloud(new PointCloudC());
    pcl::fromROSMsg(cloud_transform_msg, *cloud);

    PointCloudC::Ptr cropped_cloud(new PointCloudC());
    double min_x, min_y, min_z, max_x, max_y, max_z;
    ros::param::param(name_ + "_min_x", min_x, 0.3);
    ros::param::param(name_ + "_min_y", min_y, -1.0);
    ros::param::param(name_ + "_min_z", min_z, 0.5);
    ros::param::param(name_ + "_max_x", max_x, 0.9);
    ros::param::param(name_ + "_max_y", max_y, 1.0);
    ros::param::param(name_ + "_max_z", max_z, 1.5);
    Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
    Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);
    pcl::CropBox<PointC> crop;
    crop.setInputCloud(cloud);
    crop.setMin(min_pt);
    crop.setMax(max_pt);
    crop.filter(*cropped_cloud);

    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cropped_cloud, msg_out);

    PointC min_pcl;
    PointC max_pcl;
    pcl::getMinMax3D<PointC>(*cropped_cloud, min_pcl, max_pcl);
    ROS_INFO("min: %f, max: %f", min_pcl.x, max_pcl.x);

    pub_.publish(msg_out);
  }
}
