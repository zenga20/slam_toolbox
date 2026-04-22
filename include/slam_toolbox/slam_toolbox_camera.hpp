/*
 * slam_toolbox
 * Camera (D435) adaptation — uses /cameraDataLidar (Float32MultiArray)
 * instead of a hardware LaserScan topic.
 */

#ifndef SLAM_TOOLBOX__SLAM_TOOLBOX_CAMERA_HPP_
#define SLAM_TOOLBOX__SLAM_TOOLBOX_CAMERA_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "slam_toolbox/slam_toolbox_async.hpp"

namespace slam_toolbox
{

class CameraSlamToolbox : public AsynchronousSlamToolbox
{
public:
  explicit CameraSlamToolbox(rclcpp::NodeOptions options);

  // Shadows SlamToolbox::configure() — not virtual in Foxy, so called
  // explicitly from main() on the derived type.
  void configure();

protected:
  void cameraDataCallback(std_msgs::msg::Float32MultiArray::ConstSharedPtr msg);
  sensor_msgs::msg::LaserScan::SharedPtr toSyntheticScan(
    const std_msgs::msg::Float32MultiArray::ConstSharedPtr & msg,
    const rclcpp::Time & stamp) const;

private:
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr camera_sub_;

  std::string camera_frame_;
  std::string camera_topic_;
  double camera_angle_min_;
  double camera_angle_max_;
  double camera_range_min_;
  double camera_range_max_;
};

}  // namespace slam_toolbox

#endif  // SLAM_TOOLBOX__SLAM_TOOLBOX_CAMERA_HPP_
