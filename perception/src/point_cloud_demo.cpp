#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "perception/segmentation.h"
#include "visualization_msgs/Marker.h"
#include "perception/crop.h"
#include "perception/downsampler.h"
#include "perception/object_recognizer.h"
#include "perception_msgs/ObjectFeatures.h"

int main(int argc, char** argv) {
  if (argc < 2) {
    ROS_INFO("Usage: rosrun perception point_cloud_demo DATA_DIR");
    ros::spinOnce();
  }
  std::string data_dir(argv[1]);

  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;

  //  Visualize cropped point cloud
  ros::Publisher crop_pub =
     nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  perception::Cropper cropper(crop_pub, "crop");

  ros::Subscriber crop_sub =
     nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);

  // ros::Publisher downsampler_pub = 
  //    nh.advertise<sensor_msgs::PointCloud2>("downsampled_and_cropped_cloud", 1, true);

  // perception::Downsampler downsampler(downsampler_pub);

  // ros::Subscriber downsampler_sub = 
  //    nh.subscribe("cropped_cloud", 1, &perception::Downsampler::Callback, &downsampler);

  ros::Publisher table_pub =
    nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);

  ros::Publisher marker_pub =
    nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

  // Create the object recognizer.
  std::vector<perception_msgs::ObjectFeatures> dataset;
  perception::LoadData(data_dir, &dataset);
  perception::ObjectRecognizer recognizer(dataset);

  perception::Segmenter segmenter(table_pub, marker_pub, recognizer);
  // ros::Subscriber segmenter_sub = nh.subscribe("downsampled_and_cropped_cloud", 1, &perception::Segmenter::Callback, &segmenter);
  ros::Subscriber segmenter_sub = nh.subscribe("cropped_cloud", 1, &perception::Segmenter::Callback, &segmenter);

  ros::spin();
  return 0;
}
