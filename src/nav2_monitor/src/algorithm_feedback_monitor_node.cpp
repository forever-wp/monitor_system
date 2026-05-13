#include <algorithm>
#include <chrono>
#include <filesystem>
#include <sstream>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "nav2_monitor/fault_detector.hpp"
#include "nav2_monitor/config_profile_sync.hpp"
#include "nav2_monitor/monitor_data_store.hpp"
#include "nav2_monitor/monitor_state_json.hpp"
#include "nav2_monitor/msg/algorithm_feedback.hpp"

namespace
{
std::string resolve_config_path(const std::string & config_file)
{
  if (config_file.empty()) {
    return config_file;
  }

  namespace fs = std::filesystem;
  const fs::path input(config_file);
  if (input.is_absolute() && fs::exists(input)) {
    return input.string();
  }
  if (fs::exists(input)) {
    return fs::absolute(input).lexically_normal().string();
  }

  try {
    const fs::path package_share =
      ament_index_cpp::get_package_share_directory("nav2_monitor");
    const std::array<fs::path, 2> candidates = {
      package_share / input,
      package_share / "config" / input
    };
    for (const auto & candidate : candidates) {
      if (fs::exists(candidate)) {
        return candidate.lexically_normal().string();
      }
    }
  } catch (const std::exception &) {
  }

  return config_file;
}
}  // namespace

namespace nav2_monitor
{

class AlgorithmFeedbackMonitorNode : public rclcpp::Node
{
public:
  AlgorithmFeedbackMonitorNode()
  : Node("algorithm_feedback_monitor"), fault_detector_(this)
  {
    fault_config_path_ = declare_parameter<std::string>(
      "fault_config", "/opt/ry/config/Monitor/nav2_monitor/fault_detector_config.yaml");
    config_profile_topic_ = declare_parameter<std::string>(
      "config_profile_topic", "/monitor/config_profile");
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/nav2_monitor/algorithm_feedback");
    publish_topic_ = declare_parameter<std::string>(
      "publish_topic", "/monitor/feedback_state");
    check_rate_hz_ = std::max(1.0, declare_parameter<double>("check_rate_hz", 10.0));

    load_configuration();
    state_pub_ = create_publisher<std_msgs::msg::String>(
      publish_topic_, rclcpp::QoS(1).reliable().transient_local());
    config_profile_sub_ = create_subscription<std_msgs::msg::String>(
      config_profile_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        on_config_profile(msg);
      });
    feedback_sub_ = create_subscription<msg::AlgorithmFeedback>(
      input_topic_, rclcpp::QoS(50).reliable().durability_volatile(),
      [this](const msg::AlgorithmFeedback::SharedPtr msg) {
        on_feedback(msg);
      });

    const auto period = std::chrono::duration<double>(1.0 / check_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { evaluate_and_publish(); });

    RCLCPP_INFO(
      get_logger(), "algorithm_feedback_monitor started: input=%s publish=%s",
      input_topic_.c_str(), publish_topic_.c_str());
  }

private:
  void load_configuration()
  {
    resolved_fault_config_path_ = resolve_config_path(fault_config_path_);
    fault_detector_.load_config(resolved_fault_config_path_);
  }

  void on_config_profile(const std_msgs::msg::String::SharedPtr msg)
  {
    ConfigProfileUpdate update;
    if (!parse_config_profile_update(msg->data, update)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignore invalid config profile message: %s", msg->data.c_str());
      return;
    }

    const auto resolved = resolve_config_path(update.fault_config);
    if (update.fault_config == fault_config_path_ && resolved == resolved_fault_config_path_) {
      return;
    }

    fault_config_path_ = update.fault_config;
    load_configuration();
    RCLCPP_WARN(
      get_logger(),
      "Reload algorithm_feedback_monitor config profile: task=%s fault_config=%s",
      update.task_name.c_str(), fault_config_path_.c_str());
  }

  void on_feedback(const msg::AlgorithmFeedback::SharedPtr msg)
  {
    const bool has_stamp = (msg->stamp.sec != 0) || (msg->stamp.nanosec != 0);
    const rclcpp::Time stamp = has_stamp ? rclcpp::Time(msg->stamp) : now();
    data_store_.add_feedback_sample(
      msg->module_name,
      msg->topic_name,
      msg->metric_name,
      msg->value,
      msg->valid,
      stamp,
      now());
  }

  void evaluate_and_publish()
  {
    const auto now_time = now();
    auto faults = fault_detector_.detect_feedback_faults(data_store_, now_time);
    std::string state = "OK";
    std::string summary = "algorithm feedback normal";
    if (!faults.empty()) {
      state = fault_level_to_state_string(faults.front().level);
      summary = faults.front().reason;
    }

    std::ostringstream oss;
    oss << '{'
        << "\"stamp\":" << now_time.seconds() << ','
        << "\"source_module\":\"algorithm_feedback_monitor\","
        << "\"state\":\"" << state << "\","
        << "\"healthy\":" << (faults.empty() ? "true" : "false") << ','
        << "\"summary\":\"" << monitor_json_escape(summary) << "\","
        << "\"faults\":" << faults_to_json_array(faults)
        << '}';

    std_msgs::msg::String msg;
    msg.data = oss.str();
    state_pub_->publish(msg);
  }

  std::string fault_config_path_;
  std::string resolved_fault_config_path_;
  std::string config_profile_topic_;
  std::string input_topic_;
  std::string publish_topic_;
  double check_rate_hz_{10.0};
  FaultDetector fault_detector_;
  MonitorDataStore data_store_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_profile_sub_;
  rclcpp::Subscription<msg::AlgorithmFeedback>::SharedPtr feedback_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nav2_monitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_monitor::AlgorithmFeedbackMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
