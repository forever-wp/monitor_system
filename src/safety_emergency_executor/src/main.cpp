#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "safety_emergency_executor/safety_emergency_executor_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<safety_emergency_executor::SafetyEmergencyExecutorNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
