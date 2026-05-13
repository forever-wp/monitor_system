#include <algorithm>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
std::string json_escape(const std::string & input)
{
  std::ostringstream oss;
  for (const auto ch : input) {
    switch (ch) {
      case '"': oss << "\\\""; break;
      case '\\': oss << "\\\\"; break;
      case '\n': oss << "\\n"; break;
      case '\r': oss << "\\r"; break;
      case '\t': oss << "\\t"; break;
      default: oss << ch; break;
    }
  }
  return oss.str();
}
}  // namespace

class BatteryMonitorNode : public rclcpp::Node
{
public:
  BatteryMonitorNode()
  : Node("battery_monitor")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/battery_state");
    publish_topic_ = declare_parameter<std::string>("publish_topic", "/monitor/battery_state");
    publish_rate_hz_ = std::max(0.5, declare_parameter<double>("publish_rate_hz", 2.0));
    stale_timeout_s_ = std::max(0.1, declare_parameter<double>("stale_timeout_s", 90.0));

    state_pub_ = create_publisher<std_msgs::msg::String>(
      publish_topic_, rclcpp::QoS(1).reliable().transient_local());
    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      input_topic_, rclcpp::QoS(10).reliable().durability_volatile(),
      [this](sensor_msgs::msg::BatteryState::SharedPtr msg) {
        last_msg_ = *msg;
        last_receive_time_ = now();
        has_data_ = true;
      });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publish_state(); });

    RCLCPP_INFO(
      get_logger(), "battery_monitor started: input=%s publish=%s",
      input_topic_.c_str(), publish_topic_.c_str());
  }

private:
  void publish_state()
  {
    const auto now_time = now();
    const double age_s = has_data_ ? (now_time - last_receive_time_).seconds() : -1.0;
    const bool stale = !has_data_ || age_s > stale_timeout_s_;
    const std::string state = stale ? "STALE" : "OK";

    std::ostringstream oss;
    oss << '{'
        << "\"stamp\":" << now_time.seconds() << ','
        << "\"source_module\":\"battery_monitor\","
        << "\"state\":\"" << state << "\","
        << "\"healthy\":" << (stale ? "false" : "true") << ','
        << "\"has_data\":" << (has_data_ ? "true" : "false") << ','
        << "\"stale\":" << (stale ? "true" : "false") << ','
        << "\"age_s\":" << age_s << ','
        << "\"stale_timeout_s\":" << stale_timeout_s_ << ','
        << "\"temperature\":" << (has_data_ ? last_msg_.temperature : 0.0F) << ','
        << "\"percentage\":" << (has_data_ ? last_msg_.percentage : 0.0F) << ','
        << "\"power_supply_status\":" << (has_data_ ? static_cast<int>(last_msg_.power_supply_status) : 0) << ','
        << "\"present\":" << (has_data_ && last_msg_.present ? "true" : "false") << ','
        << "\"summary\":\"" << json_escape(stale ? "battery state missing or stale" : "battery state normal") << "\""
        << '}';

    std_msgs::msg::String msg;
    msg.data = oss.str();
    state_pub_->publish(msg);
  }

  std::string input_topic_;
  std::string publish_topic_;
  double publish_rate_hz_{2.0};
  double stale_timeout_s_{90.0};
  bool has_data_{false};
  sensor_msgs::msg::BatteryState last_msg_;
  rclcpp::Time last_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
