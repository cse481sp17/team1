#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <string>
namespace perception {
  class Cropper {
  public:
    Cropper(const ros::Publisher& pub, const std::string& name);
    void Callback(const sensor_msgs::PointCloud2& msg);

  private:
    ros::Publisher pub_;
    std::string name_;
  };
}  // namespace perception
