/*
 * slam_toolbox
 * Camera (D435) adaptation — uses /cameraDataLidar (Float32MultiArray)
 * instead of a hardware LaserScan topic.
 */

#include <memory>
#include "slam_toolbox/slam_toolbox_camera.hpp"

namespace slam_toolbox
{

/*****************************************************************************/
CameraSlamToolbox::CameraSlamToolbox(rclcpp::NodeOptions options)
: AsynchronousSlamToolbox(options)
/*****************************************************************************/
{
}

/*****************************************************************************/
void CameraSlamToolbox::configure()
/*****************************************************************************/
{
  // Declare camera params before base configure() so they can be set via
  // the params file loaded at node startup.
  // D435 horizontal FOV ~86 degrees; defaults give +/-43 degrees (~+/-0.75 rad).
  camera_topic_     = this->declare_parameter("camera_topic",     std::string("/cameraDataLidar"));
  camera_frame_     = this->declare_parameter("camera_frame",     std::string("camera_link"));
  camera_angle_min_ = this->declare_parameter("camera_angle_min", -0.7505);
  camera_angle_max_ = this->declare_parameter("camera_angle_max",  0.7505);
  camera_range_min_ = this->declare_parameter("camera_range_min",  0.1);
  camera_range_max_ = this->declare_parameter("camera_range_max",  10.0);

  // Run base configure: setParams -> setROSInterfaces (creates LaserScan sub)
  // -> setSolver -> helpers -> threads.
  AsynchronousSlamToolbox::configure();

  // Replace the LaserScan message-filter subscription with our own
  // Float32MultiArray subscription.
  scan_filter_.reset();
  scan_filter_sub_.reset();

  camera_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    camera_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&CameraSlamToolbox::cameraDataCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "CameraSlamToolbox: subscribed to '%s' (frame '%s', FOV [%.3f, %.3f] rad)",
    camera_topic_.c_str(), camera_frame_.c_str(),
    camera_angle_min_, camera_angle_max_);
}

/*****************************************************************************/
sensor_msgs::msg::LaserScan::SharedPtr CameraSlamToolbox::toSyntheticScan(
  const std_msgs::msg::Float32MultiArray::ConstSharedPtr & msg,
  const rclcpp::Time & stamp) const
/*****************************************************************************/
{
  auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
  scan->header.stamp    = stamp;
  scan->header.frame_id = camera_frame_;

  const size_t n = msg->data.size();
  scan->angle_min       = camera_angle_min_;
  scan->angle_max       = camera_angle_max_;
  scan->angle_increment = (n > 1)
    ? (camera_angle_max_ - camera_angle_min_) / static_cast<double>(n - 1)
    : 0.0;
  scan->time_increment  = 0.0;
  scan->scan_time       = 0.033f;  // ~30 Hz
  scan->range_min       = static_cast<float>(camera_range_min_);
  scan->range_max       = static_cast<float>(camera_range_max_);
  scan->ranges          = msg->data;

  return scan;
}

/*****************************************************************************/
void CameraSlamToolbox::cameraDataCallback(
  std_msgs::msg::Float32MultiArray::ConstSharedPtr msg)
/*****************************************************************************/
{
  if (msg->data.empty()) {
    return;
  }

  const rclcpp::Time stamp = this->now();
  scan_timestamped = stamp;

  Pose2 pose;
  if (!pose_helper_->getOdomPose(pose, stamp)) {
    RCLCPP_WARN(get_logger(), "CameraSlamToolbox: failed to get odom pose; dropping scan");
    return;
  }

  auto scan = toSyntheticScan(msg, stamp);

  LaserRangeFinder * laser = getLaser(scan);
  if (!laser) {
    RCLCPP_WARN(
      get_logger(),
      "CameraSlamToolbox: failed to create laser device for frame '%s'; dropping scan",
      camera_frame_.c_str());
    return;
  }

  addScan(laser, scan, pose);
}

}  // namespace slam_toolbox
