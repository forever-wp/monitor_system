#ifndef ALGORITHM_FEEDBACK_ADAPTER__EXAMPLE_BATTERY_FEEDBACK_ADAPTER_HPP_
#define ALGORITHM_FEEDBACK_ADAPTER__EXAMPLE_BATTERY_FEEDBACK_ADAPTER_HPP_

#include <vector>

#include <nav2_monitor/msg/algorithm_feedback.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include "algorithm_feedback_adapter/metric_sample.hpp"

namespace algorithm_feedback_adapter
{

class ExampleBatteryFeedbackAdapter : public rclcpp::Node
{
public:
  using InputMsg = sensor_msgs::msg::BatteryState;
  using OutputMsg = nav2_monitor::msg::AlgorithmFeedback;

  explicit ExampleBatteryFeedbackAdapter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  static std::vector<MetricSample> extract_metrics(const InputMsg & msg);

private:
  void on_msg(const InputMsg::SharedPtr msg);

  std::string input_topic_;
  std::string output_topic_;
  std::string module_name_;
  std::string topic_name_;

  rclcpp::Publisher<OutputMsg>::SharedPtr pub_;
  rclcpp::Subscription<InputMsg>::SharedPtr sub_;
};

}  // namespace algorithm_feedback_adapter

#endif  // ALGORITHM_FEEDBACK_ADAPTER__EXAMPLE_BATTERY_FEEDBACK_ADAPTER_HPP_
