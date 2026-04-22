/*
 * slam_toolbox
 * Camera (D435) SLAM node entry point.
 */

#include <sys/resource.h>
#include <memory>
#include "slam_toolbox/slam_toolbox_camera.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int stack_size = 40000000;
  {
    auto temp_node = std::make_shared<rclcpp::Node>("slam_toolbox");
    temp_node->declare_parameter("stack_size_to_use");
    if (temp_node->get_parameter("stack_size_to_use", stack_size)) {
      RCLCPP_INFO(temp_node->get_logger(), "Node using stack size %i", (int)stack_size);
      const rlim_t max_stack_size = stack_size;
      struct rlimit stack_limit;
      getrlimit(RLIMIT_STACK, &stack_limit);
      if (stack_limit.rlim_cur < max_stack_size) {
        stack_limit.rlim_cur = max_stack_size;
      }
      setrlimit(RLIMIT_STACK, &stack_limit);
    }
  }

  rclcpp::NodeOptions options;
  auto node = std::make_shared<slam_toolbox::CameraSlamToolbox>(options);
  node->configure();
  node->loadPoseGraphByParams();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
