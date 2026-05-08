#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "algorithm_feedback_adapter/battery_feedback_bridge.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<algorithm_feedback_adapter::BatteryFeedbackBridge>());
  rclcpp::shutdown();
  return 0;
}
