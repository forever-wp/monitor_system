#include "nav2_monitor/topic_frequency_monitor_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <map>
#include <sstream>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "nav2_monitor/config_profile_sync.hpp"

namespace nav2_monitor
{

namespace
{
std::string topic_param_key(const std::string & prefix, size_t index, const std::string & field)
{
  return prefix + "." + std::to_string(index) + "." + field;
}
}  // namespace

TopicFrequencyMonitorNode::TopicFrequencyMonitorNode()
: Node("topic_frequency_monitor")
{
  load_parameters();
  state_pub_ = create_publisher<std_msgs::msg::String>(publish_topic_, rclcpp::QoS(1).reliable().transient_local());
  configure_profile_subscription();
  subscribe_topics();

  const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz_));
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    [this]() { publish_states(); });

  RCLCPP_INFO(
    get_logger(), "topic_frequency_monitor started: watched=%zu publish_topic=%s rate=%.2fHz",
    watched_topics_.size(), publish_topic_.c_str(), publish_rate_hz_);
}

void TopicFrequencyMonitorNode::load_parameters()
{
  publish_topic_ = declare_parameter<std::string>("publish_topic", "/monitor/topic_states");
  fault_config_path_ = declare_parameter<std::string>(
    "fault_config", "/opt/ry/config/Monitor/nav2_monitor/fault_detector_config.yaml");
  config_profile_topic_ = declare_parameter<std::string>(
    "config_profile_topic", "/monitor/config_profile");
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
  frequency_window_s_ = std::max(0.1, declare_parameter<double>("frequency_window_s", 1.0));
  default_idle_timeout_s_ = std::max(0.05, declare_parameter<double>("default_idle_timeout_s", 0.5));
  min_idle_timeout_s_ = std::max(0.01, declare_parameter<double>("min_idle_timeout_s", 0.3));
  max_idle_timeout_s_ = std::max(min_idle_timeout_s_, declare_parameter<double>("max_idle_timeout_s", 2.0));

  load_watched_topics_from_fault_config();
  if (!watched_topics_.empty()) {
    return;
  }

  load_watched_topics_from_indexed_parameters();
}

void TopicFrequencyMonitorNode::load_watched_topics_from_fault_config()
{
  resolved_fault_config_path_ = resolve_config_path(fault_config_path_);
  std::map<std::string, WatchedTopic> by_topic;
  try {
    const auto config = YAML::LoadFile(resolved_fault_config_path_);
    if (!config["modules"] || !config["modules"].IsSequence()) {
      RCLCPP_WARN(
        get_logger(), "Fault config has no modules sequence for topic frequency monitor: %s",
        resolved_fault_config_path_.c_str());
      return;
    }

    for (const auto & module : config["modules"]) {
      if (!module["watch_topics"] || !module["watch_topics"].IsSequence()) {
        continue;
      }

      for (const auto & item : module["watch_topics"]) {
        if (!item["name"]) {
          continue;
        }

        WatchedTopic watched;
        watched.topic = item["name"].as<std::string>();
        if (watched.topic.empty()) {
          continue;
        }
        watched.min_hz = item["min_hz"] ? std::max(0.0, item["min_hz"].as<double>()) : 0.0;
        watched.idle_timeout_s = item["idle_timeout_s"] ?
          std::max(0.0, item["idle_timeout_s"].as<double>()) : compute_idle_timeout(watched.min_hz);
        watched.type = item["type"] ? item["type"].as<std::string>() : "";
        watched.type_discovered = !watched.type.empty();

        auto existing = by_topic.find(watched.topic);
        if (existing == by_topic.end()) {
          by_topic.emplace(watched.topic, std::move(watched));
          continue;
        }

        existing->second.min_hz = std::max(existing->second.min_hz, watched.min_hz);
        existing->second.idle_timeout_s =
          std::min(existing->second.idle_timeout_s, watched.idle_timeout_s);
        if (existing->second.type.empty() && !watched.type.empty()) {
          existing->second.type = watched.type;
          existing->second.type_discovered = true;
        }
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Failed to load watch_topics from fault_config=%s: %s",
      resolved_fault_config_path_.c_str(), e.what());
    watched_topics_.clear();
    return;
  }

  watched_topics_.clear();
  watched_topics_.reserve(by_topic.size());
  for (auto & item : by_topic) {
    watched_topics_.push_back(std::move(item.second));
  }
  RCLCPP_INFO(
    get_logger(), "Loaded %zu watched topics from fault_config=%s",
    watched_topics_.size(), resolved_fault_config_path_.c_str());
}

void TopicFrequencyMonitorNode::load_watched_topics_from_indexed_parameters()
{
  const auto count = declare_parameter<int64_t>("watched_topic_count", 0);
  watched_topics_.clear();
  for (int64_t i = 0; i < count; ++i) {
    WatchedTopic watched;
    const auto index = static_cast<size_t>(i);
    watched.topic = declare_parameter<std::string>(
      topic_param_key("watched_topics", index, "topic"), "");
    watched.type = declare_parameter<std::string>(
      topic_param_key("watched_topics", index, "type"), "");
    watched.min_hz = declare_parameter<double>(
      topic_param_key("watched_topics", index, "min_hz"), 0.0);
    watched.idle_timeout_s = declare_parameter<double>(
      topic_param_key("watched_topics", index, "idle_timeout_s"), 0.0);
    watched.type_discovered = !watched.type.empty();
    if (watched.idle_timeout_s <= 0.0) {
      watched.idle_timeout_s = compute_idle_timeout(watched.min_hz);
    }
    if (watched.topic.empty() || (requires_frequency(watched) && watched.type.empty())) {
      RCLCPP_WARN(
        get_logger(),
        "Skip watched_topics.%zu: topic is required; type is required only when min_hz > 0",
        index);
      continue;
    }
    watched_topics_.push_back(std::move(watched));
  }

  if (watched_topics_.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "No watched topics configured; set fault_config modules.watch_topics or watched_topic_count");
  }
}

void TopicFrequencyMonitorNode::configure_profile_subscription()
{
  config_profile_sub_ = create_subscription<std_msgs::msg::String>(
    config_profile_topic_, rclcpp::QoS(1).reliable().transient_local(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      on_config_profile(msg);
    });
}

void TopicFrequencyMonitorNode::on_config_profile(const std_msgs::msg::String::SharedPtr msg)
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
  load_watched_topics_from_fault_config();
  subscribe_topics();
  RCLCPP_WARN(
    get_logger(),
    "Reload topic_frequency_monitor config profile: task=%s watched=%zu fault_config=%s",
    update.task_name.c_str(), watched_topics_.size(), fault_config_path_.c_str());
}

void TopicFrequencyMonitorNode::subscribe_topics()
{
  const auto qos = build_subscription_qos();
  for (auto & watched : watched_topics_) {
    watched.subscription.reset();
    watched.receive_times.clear();
    watched.last_received = rclcpp::Time(0, 0, RCL_ROS_TIME);
    if (!requires_frequency(watched)) {
      RCLCPP_INFO(
        get_logger(),
        "Watching topic publisher only: topic=%s idle_timeout=%.2fs",
        watched.topic.c_str(), watched.idle_timeout_s);
      continue;
    }
    if (watched.type.empty()) {
      watched.type = resolve_topic_type(watched.topic);
      watched.type_discovered = !watched.type.empty();
    }
    if (watched.type.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Topic type not available yet, wait for ROS graph discovery: topic=%s min_hz=%.2f",
        watched.topic.c_str(), watched.min_hz);
      continue;
    }

    watched.subscription = create_generic_subscription(
      watched.topic,
      watched.type,
      qos,
      [this, topic = watched.topic](std::shared_ptr<rclcpp::SerializedMessage>) {
        on_topic_message(topic);
      });
    RCLCPP_INFO(
      get_logger(), "Watching topic frequency: topic=%s type=%s min_hz=%.2f idle_timeout=%.2fs",
      watched.topic.c_str(), watched.type.c_str(), watched.min_hz, watched.idle_timeout_s);
  }
}

void TopicFrequencyMonitorNode::on_topic_message(const std::string & topic)
{
  const auto now = this->now();
  for (auto & watched : watched_topics_) {
    if (watched.topic != topic) {
      continue;
    }
    watched.last_received = now;
    watched.receive_times.push_back(now);
    const auto oldest_allowed = now - rclcpp::Duration::from_seconds(frequency_window_s_);
    while (!watched.receive_times.empty() && watched.receive_times.front() < oldest_allowed) {
      watched.receive_times.pop_front();
    }
    return;
  }

  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 5000,
    "Received message for unconfigured topic state source: %s", topic.c_str());
}

void TopicFrequencyMonitorNode::publish_states()
{
  const auto now = this->now();
  refresh_topic_types_and_subscriptions();

  std::ostringstream oss;
  oss << '{'
      << "\"stamp\":" << now.seconds() << ','
      << "\"source_module\":\"topic_frequency_monitor\","
      << "\"items\":[";

  bool first = true;
  for (auto & watched : watched_topics_) {
    if (!first) {
      oss << ',';
    }
    first = false;

    const auto frequency = compute_frequency(watched, now);
    const bool has_publisher = this->count_publishers(watched.topic) > 0;
    const bool has_data = watched.last_received.nanoseconds() > 0;
    const auto age_s = has_data ? (now - watched.last_received).seconds() : -1.0;
    const bool frequency_required = requires_frequency(watched);
    const bool stale = frequency_required ? (!has_data || age_s > watched.idle_timeout_s) : !has_publisher;
    const bool low_frequency = frequency_required && !stale && frequency < watched.min_hz;

    oss << '{'
        << "\"topic\":\"" << json_escape(watched.topic) << "\","
        << "\"type\":\"" << json_escape(watched.type) << "\","
        << "\"frequency_required\":" << (frequency_required ? "true" : "false") << ','
        << "\"type_discovered\":" << (watched.type_discovered ? "true" : "false") << ','
        << "\"has_publisher\":" << (has_publisher ? "true" : "false") << ','
        << "\"has_data\":" << ((frequency_required ? has_data : has_publisher) ? "true" : "false") << ','
        << "\"stale\":" << (stale ? "true" : "false") << ','
        << "\"low_frequency\":" << (low_frequency ? "true" : "false") << ','
        << "\"frequency_hz\":" << frequency << ','
        << "\"min_hz\":" << watched.min_hz << ','
        << "\"age_s\":" << age_s << ','
        << "\"idle_timeout_s\":" << watched.idle_timeout_s
        << '}';
  }
  oss << "]}";

  std_msgs::msg::String msg;
  msg.data = oss.str();
  state_pub_->publish(msg);
}

void TopicFrequencyMonitorNode::refresh_topic_types_and_subscriptions()
{
  const auto qos = build_subscription_qos();
  for (auto & watched : watched_topics_) {
    if (!requires_frequency(watched) || watched.subscription || !watched.type.empty()) {
      continue;
    }
    watched.type = resolve_topic_type(watched.topic);
    watched.type_discovered = !watched.type.empty();
    if (watched.type.empty()) {
      continue;
    }
    watched.subscription = create_generic_subscription(
      watched.topic,
      watched.type,
      qos,
      [this, topic = watched.topic](std::shared_ptr<rclcpp::SerializedMessage>) {
        on_topic_message(topic);
      });
    RCLCPP_INFO(
      get_logger(), "Discovered and subscribed watched topic: topic=%s type=%s",
      watched.topic.c_str(), watched.type.c_str());
  }
}

std::string TopicFrequencyMonitorNode::resolve_topic_type(const std::string & topic) const
{
  const auto names_and_types = get_topic_names_and_types();
  const auto it = names_and_types.find(topic);
  if (it == names_and_types.end() || it->second.empty()) {
    return "";
  }
  if (it->second.size() > 1) {
    RCLCPP_WARN(
      get_logger(), "Topic has multiple types; use first discovered type: topic=%s type=%s",
      topic.c_str(), it->second.front().c_str());
  }
  return it->second.front();
}

std::string TopicFrequencyMonitorNode::resolve_config_path(const std::string & config_file) const
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

double TopicFrequencyMonitorNode::compute_frequency(WatchedTopic & watched, const rclcpp::Time & now) const
{
  const auto oldest_allowed = now - rclcpp::Duration::from_seconds(frequency_window_s_);
  while (!watched.receive_times.empty() && watched.receive_times.front() < oldest_allowed) {
    watched.receive_times.pop_front();
  }
  if (watched.receive_times.empty()) {
    return 0.0;
  }
  return static_cast<double>(watched.receive_times.size()) / frequency_window_s_;
}

double TopicFrequencyMonitorNode::compute_idle_timeout(double min_hz) const
{
  if (min_hz <= 0.0) {
    return default_idle_timeout_s_;
  }
  return std::clamp(3.0 / min_hz, min_idle_timeout_s_, max_idle_timeout_s_);
}

rclcpp::QoS TopicFrequencyMonitorNode::build_subscription_qos() const
{
  return rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
}

bool TopicFrequencyMonitorNode::requires_frequency(const WatchedTopic & watched)
{
  return watched.min_hz > 0.0;
}

std::string TopicFrequencyMonitorNode::json_escape(const std::string & input)
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

}  // namespace nav2_monitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_monitor::TopicFrequencyMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
