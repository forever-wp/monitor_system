#include "bridge/sample_feedback_bridge.hpp"

#include <builtin_interfaces/msg/time.hpp>

namespace bridge
{

SampleFeedbackBridge::SampleFeedbackBridge(const rclcpp::NodeOptions & options)
: Node("bridge_cpp_node", options)
{
  input_topic_ = this->declare_parameter<std::string>("input_topic", "/battery_state");
  output_topic_ = this->declare_parameter<std::string>(
    "output_topic", "/nav2_monitor/algorithm_feedback");
  module_name_ = this->declare_parameter<std::string>("module_name", "battery_node");
  topic_name_ = this->declare_parameter<std::string>("topic_name", input_topic_);

  pub_ = this->create_publisher<OutputMsg>(output_topic_, rclcpp::QoS(50));
  sub_ = this->create_subscription<InputMsg>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&SampleFeedbackBridge::on_msg, this, std::placeholders::_1));
}

std::vector<MetricSample> SampleFeedbackBridge::extract_metrics(const InputMsg & msg)
{
  const bool valid = msg.present;
  return {
    {"battery_percentage", static_cast<double>(msg.percentage), valid},
    {"battery_temperature", static_cast<double>(msg.temperature), valid},
    {"battery_voltage", static_cast<double>(msg.voltage), valid}
  };
}

void SampleFeedbackBridge::on_msg(const InputMsg::SharedPtr msg)
{
  const bool has_stamp = (msg->header.stamp.sec != 0) || (msg->header.stamp.nanosec != 0);
  builtin_interfaces::msg::Time stamp;
  if (has_stamp) {
    stamp = msg->header.stamp;
  } else {
    stamp = this->now();
  }

  for (const auto & metric : extract_metrics(*msg)) {
    OutputMsg fb;
    fb.stamp = stamp;
    fb.module_name = module_name_;
    fb.topic_name = topic_name_;
    fb.metric_name = metric.name;
    fb.value = metric.value;
    fb.valid = metric.valid;
    pub_->publish(fb);
  }
}

}  // namespace bridge
