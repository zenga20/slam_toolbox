/*
 * slam_toolbox
 * Camera (D435) SLAM node entry point.
 */

#include <memory>
#include "slam_toolbox/slam_toolbox_camera.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<slam_toolbox::CameraSlamToolbox>(options);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
