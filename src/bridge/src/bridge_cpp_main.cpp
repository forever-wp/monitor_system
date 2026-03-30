#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "bridge/sample_feedback_bridge.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bridge::SampleFeedbackBridge>());
  rclcpp::shutdown();
  return 0;
}
