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

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

protected:
  void cameraDataCallback(std_msgs::msg::Float32MultiArray::ConstSharedPtr msg);
  sensor_msgs::msg::LaserScan::SharedPtr toSyntheticScan(
    const std_msgs::msg::Float32MultiArray::ConstSharedPtr & msg,
    const rclcpp::Time & stamp) const;

private:
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr camera_sub_;

  std::string camera_frame_;
  double camera_angle_min_;
  double camera_angle_max_;
  double camera_range_min_;
  double camera_range_max_;
  std::string camera_topic_;
};

}  // namespace slam_toolbox

#endif  // SLAM_TOOLBOX__SLAM_TOOLBOX_CAMERA_HPP_
