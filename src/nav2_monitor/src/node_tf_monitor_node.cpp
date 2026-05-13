#include "nav2_monitor/node_tf_monitor_node.hpp"

#include <algorithm>
#include <filesystem>
#include <sstream>
#include <unordered_set>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "nav2_monitor/config_profile_sync.hpp"

namespace nav2_monitor
{

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

NodeTfMonitorNode::NodeTfMonitorNode()
: Node("node_tf_monitor"), fault_detector_(this)
{
  fault_config_path_ = declare_parameter<std::string>(
    "fault_config", "/opt/ry/config/Monitor/nav2_monitor/fault_detector_config.yaml");
  config_profile_topic_ = declare_parameter<std::string>(
    "config_profile_topic", "/monitor/config_profile");
  publish_topic_ = declare_parameter<std::string>("publish_topic", "/monitor/node_tf_state");
  scan_rate_hz_ = std::max(0.2, declare_parameter<double>("scan_rate_hz", 2.0));
  timeout_s_ = std::max(0.1, declare_parameter<double>("timeout_s", 5.0));

  load_configuration();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  state_pub_ = create_publisher<std_msgs::msg::String>(
    publish_topic_, rclcpp::QoS(1).reliable().transient_local());
  configure_profile_subscription();

  const auto period = std::chrono::duration<double>(1.0 / scan_rate_hz_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { publish_state(); });

  RCLCPP_INFO(
    get_logger(), "node_tf_monitor started: nodes=%zu transforms=%zu publish=%s",
    monitored_nodes_.size(), monitored_transforms_.size(), publish_topic_.c_str());
}

void NodeTfMonitorNode::load_configuration()
{
  resolved_fault_config_path_ = resolve_config_path(fault_config_path_);
  fault_detector_.load_config(resolved_fault_config_path_);
  monitored_nodes_ = fault_detector_.get_monitored_nodes();
  monitored_transforms_.clear();
  for (const auto & tf_str : fault_detector_.get_monitored_transforms()) {
    const auto pos = tf_str.find("->");
    if (pos != std::string::npos) {
      monitored_transforms_.push_back({tf_str.substr(0, pos), tf_str.substr(pos + 2)});
    }
  }
}

void NodeTfMonitorNode::configure_profile_subscription()
{
  config_profile_sub_ = create_subscription<std_msgs::msg::String>(
    config_profile_topic_,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&NodeTfMonitorNode::on_config_profile, this, std::placeholders::_1));
}

void NodeTfMonitorNode::on_config_profile(const std_msgs::msg::String::SharedPtr msg)
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
    "Reload node_tf_monitor config profile: task=%s nodes=%zu transforms=%zu",
    update.task_name.c_str(), monitored_nodes_.size(), monitored_transforms_.size());
}

void NodeTfMonitorNode::publish_state()
{
  const auto now_time = now();
  auto node_names = get_node_names();
  auto node_name_with_ns = get_node_graph_interface()->get_node_names_and_namespaces();
  std::unordered_set<std::string> graph_node_names;
  graph_node_names.reserve((node_names.size() + node_name_with_ns.size() * 2) * 2);
  for (const auto & name : node_names) {
    const auto normalized = normalize_graph_name(name);
    if (!normalized.empty()) {
      graph_node_names.insert(normalized);
      graph_node_names.insert(basename_graph_name(normalized));
    }
  }
  for (const auto & item : node_name_with_ns) {
    const auto normalized_name = normalize_graph_name(item.first);
    if (!normalized_name.empty()) {
      graph_node_names.insert(normalized_name);
      graph_node_names.insert(basename_graph_name(normalized_name));
    }
    std::string fq;
    if (item.second.empty() || item.second == "/") {
      fq = item.first;
    } else {
      fq = normalize_graph_name(item.second);
      if (!fq.empty() && fq.back() != '/') {
        fq += "/";
      }
      fq += item.first;
    }
    const auto normalized_fq = normalize_graph_name(fq);
    if (!normalized_fq.empty()) {
      graph_node_names.insert(normalized_fq);
      graph_node_names.insert(basename_graph_name(normalized_fq));
    }
  }

  std::ostringstream oss;
  oss << '{'
      << "\"stamp\":" << now_time.seconds() << ','
      << "\"source_module\":\"node_tf_monitor\","
      << "\"timeout_s\":" << timeout_s_ << ','
      << "\"nodes\":[";
  for (size_t i = 0; i < monitored_nodes_.size(); ++i) {
    if (i > 0) {
      oss << ',';
    }
    const auto & node = monitored_nodes_[i];
    const bool active =
      graph_node_names.count(normalize_graph_name(node)) > 0 ||
      graph_node_names.count(basename_graph_name(node)) > 0;
    oss << '{'
        << "\"name\":\"" << json_escape(node) << "\","
        << "\"active\":" << (active ? "true" : "false")
        << '}';
  }

  oss << "],\"transforms\":[";
  for (size_t i = 0; i < monitored_transforms_.size(); ++i) {
    if (i > 0) {
      oss << ',';
    }
    const auto & [src, tgt] = monitored_transforms_[i];
    bool available = false;
    double latency_ms = -1.0;
    try {
      const auto tf = tf_buffer_->lookupTransform(tgt, src, tf2::TimePointZero);
      available = true;
      latency_ms = (now_time - rclcpp::Time(tf.header.stamp)).seconds() * 1000.0;
    } catch (const std::exception &) {
    }
    oss << '{'
        << "\"name\":\"" << json_escape(src + "->" + tgt) << "\","
        << "\"source\":\"" << json_escape(src) << "\","
        << "\"target\":\"" << json_escape(tgt) << "\","
        << "\"available\":" << (available ? "true" : "false") << ','
        << "\"latency_ms\":" << latency_ms
        << '}';
  }
  oss << "]}";

  std_msgs::msg::String msg;
  msg.data = oss.str();
  state_pub_->publish(msg);
}

std::string NodeTfMonitorNode::json_escape(const std::string & input)
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

std::string NodeTfMonitorNode::normalize_graph_name(const std::string & name)
{
  size_t pos = 0;
  while (pos < name.size() && name[pos] == '/') {
    ++pos;
  }
  return pos >= name.size() ? std::string() : name.substr(pos);
}

std::string NodeTfMonitorNode::basename_graph_name(const std::string & name)
{
  const auto normalized = normalize_graph_name(name);
  const auto pos = normalized.rfind('/');
  return pos == std::string::npos ? normalized : normalized.substr(pos + 1);
}

}  // namespace nav2_monitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_monitor::NodeTfMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
