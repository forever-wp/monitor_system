#ifndef BRIDGE__SAMPLE_FEEDBACK_BRIDGE_HPP_
#define BRIDGE__SAMPLE_FEEDBACK_BRIDGE_HPP_

#include <string>
#include <vector>

#include <nav2_monitor/msg/algorithm_feedback.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

namespace bridge
{

struct MetricSample
{
  std::string name;
  double value;
  bool valid;
};

class SampleFeedbackBridge : public rclcpp::Node
{
public:
  using InputMsg = sensor_msgs::msg::BatteryState;
  using OutputMsg = nav2_monitor::msg::AlgorithmFeedback;

  explicit SampleFeedbackBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

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

}  // namespace bridge

#endif  // BRIDGE__SAMPLE_FEEDBACK_BRIDGE_HPP_
