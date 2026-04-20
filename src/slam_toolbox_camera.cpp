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
CallbackReturn CameraSlamToolbox::on_configure(
  const rclcpp_lifecycle::State & state)
/*****************************************************************************/
{
  // Run base configuration (SLAM internals: smapper, tf, laser_assistant, etc.)
  auto result = AsynchronousSlamToolbox::on_configure(state);
  if (result != CallbackReturn::SUCCESS) {
    return result;
  }

  // D435 horizontal FOV is ~86°; defaults give ±43° (≈ ±0.75 rad)
  if (!this->has_parameter("camera_frame")) {
    this->declare_parameter("camera_frame", std::string("camera_link"));
  }
  if (!this->has_parameter("camera_topic")) {
    this->declare_parameter("camera_topic", std::string("/cameraDataLidar"));
  }
  if (!this->has_parameter("camera_angle_min")) {
    this->declare_parameter("camera_angle_min", -0.7505);
  }
  if (!this->has_parameter("camera_angle_max")) {
    this->declare_parameter("camera_angle_max", 0.7505);
  }
  if (!this->has_parameter("camera_range_min")) {
    this->declare_parameter("camera_range_min", 0.1);
  }
  if (!this->has_parameter("camera_range_max")) {
    this->declare_parameter("camera_range_max", 10.0);
  }

  camera_frame_     = this->get_parameter("camera_frame").as_string();
  camera_topic_     = this->get_parameter("camera_topic").as_string();
  camera_angle_min_ = this->get_parameter("camera_angle_min").as_double();
  camera_angle_max_ = this->get_parameter("camera_angle_max").as_double();
  camera_range_min_ = this->get_parameter("camera_range_min").as_double();
  camera_range_max_ = this->get_parameter("camera_range_max").as_double();

  return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
CallbackReturn CameraSlamToolbox::on_activate(
  const rclcpp_lifecycle::State & state)
/*****************************************************************************/
{
  // Run base activation: creates all publishers, services, and the default
  // LaserScan subscription via setROSInterfaces().
  auto result = AsynchronousSlamToolbox::on_activate(state);
  if (result != CallbackReturn::SUCCESS) {
    return result;
  }

  // Replace the LaserScan message-filter subscription with our own
  // Float32MultiArray subscription.  The scan_filter_ / scan_filter_sub_
  // unique_ptrs live in SlamToolbox (protected), so resetting them here
  // cleanly tears down the LaserScan pipeline.
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

  return CallbackReturn::SUCCESS;
}

/*****************************************************************************/
CallbackReturn CameraSlamToolbox::on_deactivate(
  const rclcpp_lifecycle::State & state)
/*****************************************************************************/
{
  camera_sub_.reset();
  return AsynchronousSlamToolbox::on_deactivate(state);
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
  scan_header.stamp    = stamp;
  scan_header.frame_id = camera_frame_;

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

  if (shouldProcessScan(scan, pose)) {
    addScan(laser, scan, pose);
  }
}

}  // namespace slam_toolbox

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(slam_toolbox::CameraSlamToolbox)
