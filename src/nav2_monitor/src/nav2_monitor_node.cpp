#include "nav2_monitor/nav2_monitor_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/serialized_message.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <limits>

namespace nav2_monitor
{

namespace
{
uint8_t fault_level_to_msg(FaultLevel level)
{
  switch (level) {
    case FaultLevel::WARNING:
      return msg::FaultEvent::WARNING;
    case FaultLevel::ERROR:
      return msg::FaultEvent::ERROR;
    case FaultLevel::CRITICAL:
      return msg::FaultEvent::CRITICAL;
    case FaultLevel::NORMAL:
    default:
      return msg::FaultEvent::NORMAL;
  }
}

uint8_t action_to_msg(ActionType action)
{
  switch (action) {
    case ActionType::SUPERVISOR:
      return msg::FaultEvent::SUPERVISOR;
    case ActionType::SAFETY_SYSTEM:
      return msg::FaultEvent::SAFETY_SYSTEM;
    case ActionType::NONE:
    default:
      return msg::FaultEvent::NONE;
  }
}

std::string normalize_graph_name(const std::string & name)
{
  if (name.empty()) {
    return name;
  }

  size_t pos = 0;
  while (pos < name.size() && name[pos] == '/') {
    ++pos;
  }
  if (pos >= name.size()) {
    return "";
  }
  return name.substr(pos);
}

std::string basename_graph_name(const std::string & name)
{
  const std::string normalized = normalize_graph_name(name);
  if (normalized.empty()) {
    return normalized;
  }
  const size_t pos = normalized.rfind('/');
  if (pos == std::string::npos) {
    return normalized;
  }
  if (pos + 1 >= normalized.size()) {
    return "";
  }
  return normalized.substr(pos + 1);
}

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

  std::error_code ec;
  const fs::path cwd_path = fs::current_path(ec);
  if (!ec) {
    const fs::path cwd_candidate = cwd_path / input;
    if (fs::exists(cwd_candidate)) {
      return cwd_candidate.string();
    }
  }

  try {
    const fs::path share_dir(ament_index_cpp::get_package_share_directory("nav2_monitor"));
    const fs::path share_candidate = share_dir / input;
    if (fs::exists(share_candidate)) {
      return share_candidate.string();
    }
  } catch (...) {
  }

  return config_file;
}
}  // namespace

Nav2MonitorNode::Nav2MonitorNode()
: Node("nav2_monitor"), timeout_(5.0), safety_cooldown_s_(2.0), supervisor_cooldown_s_(5.0),
  sys_monitor_(), fault_detector_(this)
{
  timeout_ = this->declare_parameter<double>("timeout", 5.0);
  double scan_rate = this->declare_parameter<double>("scan_rate", 0.5);
  double check_rate = this->declare_parameter<double>("check_rate", 1.0);
  safety_cooldown_s_ = this->declare_parameter<double>("safety_cooldown_s", 2.0);
  supervisor_cooldown_s_ = this->declare_parameter<double>("supervisor_cooldown_s", 5.0);
  algorithm_feedback_topic_ = this->declare_parameter<std::string>(
    "algorithm_feedback_topic", "/nav2_monitor/algorithm_feedback");
  battery_state_topic_ = this->declare_parameter<std::string>(
    "battery_state_topic", "/battery_state");
  battery_state_timeout_s_ = std::max(
    1.0, this->declare_parameter<double>("battery_state_timeout_s", 90.0));
  base_frame_id_ = this->declare_parameter<std::string>("base_frame_id", "base_link");

  fallback_target_nodes_ = this->declare_parameter<std::vector<std::string>>(
    "target_nodes", std::vector<std::string>{});
  fallback_watch_topics_ = this->declare_parameter<std::vector<std::string>>(
    "watch_topics", std::vector<std::string>{});
  target_nodes_ = fallback_target_nodes_;
  watch_topics_ = fallback_watch_topics_;

  auto tf_strs = this->declare_parameter<std::vector<std::string>>(
    "target_transforms", std::vector<std::string>{});
  for (const auto & tf_str : tf_strs) {
    auto pos = tf_str.find("->");
    if (pos != std::string::npos) {
      target_transforms_.push_back({tf_str.substr(0, pos), tf_str.substr(pos + 2)});
    }
  }

  if (scan_rate <= 0.0) {
    scan_rate = 0.5;
  }
  if (check_rate <= 0.0) {
    check_rate = 1.0;
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  pub_ = this->create_publisher<msg::MonitorStatus>("/nav2_monitor/status", 10);
  fault_event_pub_ = this->create_publisher<msg::FaultEvent>("/nav2_monitor/fault_event", 10);
  supervisor_pub_ = this->create_publisher<msg::SupervisorCmd>("/supervisor/cmd", 10);
  fault_state_coordinator_.configure(this, "/safety_system/cmd");

  std::string config_file = this->declare_parameter<std::string>("fault_config", "");
  const std::string resolved_config_file = resolve_config_path(config_file);
  fault_detector_.set_feedback_default_max_stale(timeout_);
  if (!config_file.empty()) {
    RCLCPP_INFO(
      get_logger(), "fault_config param='%s', resolved='%s'",
      config_file.c_str(), resolved_config_file.c_str());
    std::ifstream config_stream(resolved_config_file);
    if (!config_stream.good()) {
      RCLCPP_ERROR(
        get_logger(),
        "fault_config not readable: %s (resolved: %s), fallback to target_nodes/watch_topics",
        config_file.c_str(), resolved_config_file.c_str());
    } else {
      fault_detector_.load_config(resolved_config_file);
    }
  }
  if (fault_detector_.has_module_configs()) {
    target_nodes_ = fault_detector_.get_monitored_nodes();
    watch_topics_ = fault_detector_.get_watched_topics();
    monitor_targets_from_fault_config_ = true;
    RCLCPP_INFO(
      get_logger(), "Monitor targets loaded from fault_config modules: %zu nodes, %zu topics",
      target_nodes_.size(), watch_topics_.size());
  } else {
    monitor_targets_from_fault_config_ = false;
    RCLCPP_INFO(
      get_logger(),
      "fault_config missing/empty modules, fallback to target_nodes/watch_topics params");
  }

  if (fault_detector_.chassis_stationary_enabled()) {
    const auto & cfg = fault_detector_.get_chassis_stationary_config();
    command_topic_ = cfg.command_topic;
    moto_topic_ = cfg.moto_topic;
    odom_topic_ = cfg.odom_topic;

    command_sub_ = this->create_subscription<std_msgs::msg::String>(
      command_topic_, rclcpp::QoS(20),
      std::bind(&Nav2MonitorNode::on_command, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Nav2MonitorNode::on_odom, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(), "Chassis stationary judge enabled: command=%s moto=%s odom=%s",
      command_topic_.c_str(), moto_topic_.c_str(), odom_topic_.c_str());
  }

  algorithm_feedback_sub_ = this->create_subscription<msg::AlgorithmFeedback>(
    algorithm_feedback_topic_, rclcpp::QoS(50),
    std::bind(&Nav2MonitorNode::on_algorithm_feedback, this, std::placeholders::_1));
  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_state_topic_, rclcpp::SensorDataQoS(),
    std::bind(&Nav2MonitorNode::on_battery_state, this, std::placeholders::_1));

  if (fault_detector_.collision_detection_enabled()) {
    const auto & cfg = fault_detector_.get_collision_detection_config();
    if (!cfg.scan_topic.empty()) {
      collision_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        cfg.scan_topic, rclcpp::SensorDataQoS(),
        std::bind(&Nav2MonitorNode::on_collision_scan, this, std::placeholders::_1));
    }
    if (!cfg.pointcloud_topic.empty()) {
      collision_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cfg.pointcloud_topic, rclcpp::SensorDataQoS(),
        std::bind(&Nav2MonitorNode::on_collision_pointcloud, this, std::placeholders::_1));
    }
    for (const auto & zone : cfg.zones) {
      if (!zone.visualize || zone.points.size() < 3) {
        continue;
      }
      auto qos = rclcpp::QoS(1).transient_local().reliable();
      collision_zone_pubs_[zone.name] = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
        zone.polygon_pub_topic, qos);
    }
    publish_collision_zones();
    RCLCPP_INFO(
      get_logger(), "Collision detection enabled: scan=%s pointcloud=%s",
      cfg.scan_topic.c_str(), cfg.pointcloud_topic.c_str());
  }

  std::string vehicle_status_file = this->declare_parameter<std::string>(
    "vehicle_status_file", "/home/ry/.ros/navigate_status/navigate_todoor_status.json");
  vehicle_monitor_ = std::make_unique<VehicleStatusMonitor>(vehicle_status_file);

  scan_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / scan_rate)),
    std::bind(&Nav2MonitorNode::scan_topology, this));

  check_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / check_rate)),
    std::bind(&Nav2MonitorNode::check_health, this));

  param_callback_ = this->add_on_set_parameters_callback(
    std::bind(&Nav2MonitorNode::on_parameter_change, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "nav2_monitor started");
}

double Nav2MonitorNode::parse_command_speed(const std::string & payload) const
{
  auto key_pos = payload.find("\"speed\"");
  if (key_pos == std::string::npos) {
    return 0.0;
  }
  auto colon_pos = payload.find(':', key_pos);
  if (colon_pos == std::string::npos) {
    return 0.0;
  }
  auto value_begin = payload.find_first_not_of(" \t\r\n", colon_pos + 1);
  if (value_begin == std::string::npos) {
    return 0.0;
  }
  auto value_end = payload.find_first_of(",}", value_begin);
  const std::string token = payload.substr(value_begin, value_end - value_begin);
  try {
    return std::stod(token);
  } catch (...) {
    return 0.0;
  }
}

bool Nav2MonitorNode::decode_moto_info(
  const rclcpp::SerializedMessage & msg, double & left_speed, double & right_speed) const
{
  const auto & serialized = msg.get_rcl_serialized_message();
  const auto * buffer = serialized.buffer;
  const size_t len = serialized.buffer_length;
  if (buffer == nullptr || len < 32) {
    return false;
  }

  const size_t offsets[] = {8, 4, 0};
  for (size_t offset : offsets) {
    if (len < offset + 32) {
      continue;
    }

    double candidate_left = 0.0;
    double candidate_right = 0.0;
    std::memcpy(&candidate_left, buffer + offset + 16, sizeof(double));
    std::memcpy(&candidate_right, buffer + offset + 24, sizeof(double));
    if (std::isfinite(candidate_left) && std::isfinite(candidate_right) &&
        std::fabs(candidate_left) < 1e6 && std::fabs(candidate_right) < 1e6) {
      left_speed = candidate_left;
      right_speed = candidate_right;
      return true;
    }
  }
  return false;
}

void Nav2MonitorNode::on_command(const std_msgs::msg::String::SharedPtr msg)
{
  const double speed = parse_command_speed(msg->data);
  const auto now = this->now();
  std::lock_guard<std::mutex> lock(mtx_);
  data_store_.set_command_speed(speed, now);
}

void Nav2MonitorNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & v = msg->twist.twist.linear;
  const double speed = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  const auto stamp = (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) ?
    rclcpp::Time(msg->header.stamp) : this->now();
  std::lock_guard<std::mutex> lock(mtx_);
  data_store_.set_odom_speed(speed, stamp);
}

void Nav2MonitorNode::on_battery_state(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  const auto has_stamp = (msg->header.stamp.sec != 0) || (msg->header.stamp.nanosec != 0);
  const auto stamp = has_stamp ? rclcpp::Time(msg->header.stamp) : this->now();
  std::lock_guard<std::mutex> lock(mtx_);
  data_store_.set_battery_state(msg->temperature, msg->percentage, stamp);
}

void Nav2MonitorNode::on_collision_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::vector<CollisionPoint> points;
  points.reserve(msg->ranges.size());

  double tx = 0.0;
  double ty = 0.0;
  double yaw = 0.0;
  bool have_tf = msg->header.frame_id.empty() || msg->header.frame_id == base_frame_id_;
  if (!have_tf) {
    try {
      auto tf = tf_buffer_->lookupTransform(base_frame_id_, msg->header.frame_id, tf2::TimePointZero);
      tx = tf.transform.translation.x;
      ty = tf.transform.translation.y;
      tf2::Quaternion q(
        tf.transform.rotation.x, tf.transform.rotation.y,
        tf.transform.rotation.z, tf.transform.rotation.w);
      double roll = 0.0;
      double pitch = 0.0;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      have_tf = true;
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Collision scan transform failed: %s", e.what());
      return;
    }
  }

  double angle = msg->angle_min;
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  for (const auto range : msg->ranges) {
    if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max) {
      angle += msg->angle_increment;
      continue;
    }

    const double sx = range * std::cos(angle);
    const double sy = range * std::sin(angle);
    CollisionPoint point;
    if (have_tf && msg->header.frame_id != base_frame_id_) {
      point.x = cos_yaw * sx - sin_yaw * sy + tx;
      point.y = sin_yaw * sx + cos_yaw * sy + ty;
    } else {
      point.x = sx;
      point.y = sy;
    }
    points.push_back(point);
    angle += msg->angle_increment;
  }

  const auto stamp = (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) ?
    rclcpp::Time(msg->header.stamp) : this->now();
  std::lock_guard<std::mutex> lock(mtx_);
  data_store_.set_collision_points("scan", points, stamp);
}

void Nav2MonitorNode::on_collision_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const auto & cfg = fault_detector_.get_collision_detection_config();
  std::vector<CollisionPoint> points;
  points.reserve(msg->width * msg->height);

  tf2::Transform transform;
  bool have_tf = msg->header.frame_id.empty() || msg->header.frame_id == base_frame_id_;
  if (!have_tf) {
    try {
      auto tf = tf_buffer_->lookupTransform(base_frame_id_, msg->header.frame_id, tf2::TimePointZero);
      tf2::Quaternion q(
        tf.transform.rotation.x, tf.transform.rotation.y,
        tf.transform.rotation.z, tf.transform.rotation.w);
      transform.setOrigin(tf2::Vector3(
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z));
      transform.setRotation(q);
      have_tf = true;
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Collision pointcloud transform failed: %s", e.what());
      return;
    }
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
      continue;
    }

    tf2::Vector3 point_xyz(*iter_x, *iter_y, *iter_z);
    if (have_tf && msg->header.frame_id != base_frame_id_) {
      point_xyz = transform * point_xyz;
    }

    if (point_xyz.z() < cfg.pointcloud_min_height || point_xyz.z() > cfg.pointcloud_max_height) {
      continue;
    }

    points.push_back(CollisionPoint{point_xyz.x(), point_xyz.y()});
  }

  const auto stamp = (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) ?
    rclcpp::Time(msg->header.stamp) : this->now();
  std::lock_guard<std::mutex> lock(mtx_);
  data_store_.set_collision_points("pointcloud", points, stamp);
}

void Nav2MonitorNode::try_subscribe_moto_topic()
{
  if (!fault_detector_.chassis_stationary_enabled() || moto_topic_.empty() || moto_sub_) {
    return;
  }

  const auto topic_types = this->get_topic_names_and_types();
  auto it = topic_types.find(moto_topic_);
  if (it == topic_types.end() || it->second.empty()) {
    return;
  }

  moto_topic_type_ = it->second.front();
  moto_sub_ = this->create_generic_subscription(
    moto_topic_, moto_topic_type_, rclcpp::QoS(50),
    [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      double left_speed = 0.0;
      double right_speed = 0.0;
      const bool ok = decode_moto_info(*msg, left_speed, right_speed);
      const auto now = this->now();
      {
        std::lock_guard<std::mutex> lock(mtx_);
        data_store_.set_moto_speed(left_speed, right_speed, ok, now);
      }
      if (!ok) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Failed to decode moto topic '%s' type '%s' as MotoInfo float64 fields",
          moto_topic_.c_str(), moto_topic_type_.c_str());
      }
    });

  RCLCPP_INFO(
    get_logger(), "Subscribed moto topic '%s' type '%s'",
    moto_topic_.c_str(), moto_topic_type_.c_str());
}

void Nav2MonitorNode::on_algorithm_feedback(const msg::AlgorithmFeedback::SharedPtr msg)
{
  const bool has_stamp = (msg->stamp.sec != 0) || (msg->stamp.nanosec != 0);
  rclcpp::Time stamp = has_stamp ? rclcpp::Time(msg->stamp) : this->now();
  std::lock_guard<std::mutex> lock(mtx_);
  data_store_.add_feedback_sample(
    msg->module_name, msg->topic_name, msg->metric_name, msg->value, msg->valid, stamp);
}

bool Nav2MonitorNode::should_publish_action(
  const std::string & module_name, ActionType action, const rclcpp::Time & now)
{
  double cooldown_s = supervisor_cooldown_s_;
  if (action == ActionType::SAFETY_SYSTEM) {
    cooldown_s = safety_cooldown_s_;
  } else if (action == ActionType::SUPERVISOR) {
    cooldown_s = supervisor_cooldown_s_;
  }

  const std::string key = module_name + ":" + std::to_string(static_cast<int>(action));
  auto it = last_action_publish_time_.find(key);
  if (it != last_action_publish_time_.end() && (now - it->second).seconds() < cooldown_s) {
    return false;
  }

  last_action_publish_time_[key] = now;
  return true;
}

void Nav2MonitorNode::subscribe_watch_topics()
{
  auto topics = this->get_topic_names_and_types();
  for (const auto & topic : watch_topics_) {
    if (topic_subs_.count(topic) || !topics.count(topic) || topics[topic].empty()) {
      continue;
    }

    std::string type = topics[topic][0];
    {
      std::lock_guard<std::mutex> lock(mtx_);
      topic_info_[topic].type = type;
    }

    auto sub = this->create_generic_subscription(
      topic, type, rclcpp::QoS(10),
      [this, topic](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        if (msg->size() == 0) {
          std::lock_guard<std::mutex> lock(mtx_);
          data_store_.add_watch_topic_sample(topic, this->now(), false);
          const auto * state = data_store_.get_watch_topic_state(topic);
          const size_t empty_count = state == nullptr ? 0U : state->empty_msg_count;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Topic '%s' empty msg (count: %zu)", topic.c_str(), empty_count);
          return;
        }

        auto now = this->now();
        std::lock_guard<std::mutex> lock(mtx_);
        data_store_.add_watch_topic_sample(topic, now, true);
      });
    topic_subs_[topic] = sub;
  }
}

void Nav2MonitorNode::scan_topology()
{
  auto now = this->now();
  auto nodes = this->get_node_names();
  auto node_name_with_ns = this->get_node_graph_interface()->get_node_names_and_namespaces();
  std::unordered_set<std::string> graph_node_names;
  graph_node_names.reserve((nodes.size() + node_name_with_ns.size() * 2) * 2);
  for (const auto & name : nodes) {
    const auto normalized = normalize_graph_name(name);
    if (!normalized.empty()) {
      graph_node_names.insert(normalized);
      graph_node_names.insert(basename_graph_name(normalized));
    }
  }
  for (const auto & item : node_name_with_ns) {
    const auto & name = item.first;
    const auto & node_ns = item.second;
    const auto normalized_name = normalize_graph_name(name);
    if (!normalized_name.empty()) {
      graph_node_names.insert(normalized_name);
      graph_node_names.insert(basename_graph_name(normalized_name));
    }

    std::string fq;
    if (node_ns.empty() || node_ns == "/") {
      fq = name;
    } else {
      fq = normalize_graph_name(node_ns);
      if (!fq.empty() && fq.back() != '/') {
        fq += "/";
      }
      fq += name;
    }
    const auto normalized_fq = normalize_graph_name(fq);
    if (!normalized_fq.empty()) {
      graph_node_names.insert(normalized_fq);
      graph_node_names.insert(basename_graph_name(normalized_fq));
    }
  }

  {
    std::lock_guard<std::mutex> lock(mtx_);
    for (const auto & node : target_nodes_) {
      const auto normalized_target = normalize_graph_name(node);
      const auto basename_target = basename_graph_name(node);
      if (
        graph_node_names.count(normalized_target) > 0 ||
        graph_node_names.count(basename_target) > 0)
      {
        data_store_.mark_node_seen(node, now);
      }
    }
    for (const auto & topic : watch_topics_) {
      data_store_.set_watch_topic_publisher(topic, this->count_publishers(topic) > 0);
    }
  }

  for (const auto & [src, tgt] : target_transforms_) {
    try {
      auto tf = tf_buffer_->lookupTransform(tgt, src, tf2::TimePointZero);
      std::lock_guard<std::mutex> lock(mtx_);
      tf_info_[{src, tgt}].last_update = now;
      tf_info_[{src, tgt}].latency_ms = (now - rclcpp::Time(tf.header.stamp)).seconds() * 1000.0;
    } catch (...) {
    }
  }

  subscribe_watch_topics();
  try_subscribe_moto_topic();
}

void Nav2MonitorNode::check_health()
{
  const auto now = this->now();
  const auto vehicle_status = vehicle_monitor_->get_status();

  msg::MonitorStatus status_msg;
  std::vector<FaultInfo> pending_faults;
  pending_faults.reserve(8);
  std::vector<FaultInfo> faults;
  std::vector<FaultEdgeEvent> fault_edge_events;
  std::optional<SafetyCommandUpdate> safety_update;

  {
    std::lock_guard<std::mutex> lock(mtx_);
    status_msg.all_ok = true;
    status_msg.monitored_nodes = target_nodes_;
    status_msg.monitored_topics = watch_topics_;

    for (const auto & node : target_nodes_) {
      if (data_store_.is_node_active(node, now, timeout_)) {
        status_msg.active_nodes.push_back(node);
      } else {
        status_msg.timeout_nodes.push_back(node);
        status_msg.all_ok = false;
      }
    }

    for (const auto & topic : watch_topics_) {
      const auto * info = data_store_.get_watch_topic_state(topic);
      if (info != nullptr && info->has_publisher && info->has_valid_data) {
        status_msg.active_topics.push_back(topic);
        status_msg.topic_frequencies.push_back(static_cast<float>(info->frequency));
      } else {
        status_msg.inactive_topics.push_back(topic);
        status_msg.topic_frequencies.push_back(0.0);
        status_msg.all_ok = false;
      }
    }

    for (const auto & [src, tgt] : target_transforms_) {
      std::string tf_str = src + "->" + tgt;
      status_msg.monitored_transforms.push_back(tf_str);
      if (tf_info_.count({src, tgt}) && (now - tf_info_[{src, tgt}].last_update).seconds() <= timeout_) {
        status_msg.available_transforms.push_back(tf_str);
        status_msg.transform_latencies_ms.push_back(tf_info_[{src, tgt}].latency_ms);
      } else {
        status_msg.stale_transforms.push_back(tf_str);
        status_msg.transform_latencies_ms.push_back(-1.0);
        status_msg.all_ok = false;
      }
    }

    status_msg.cpu_usage = sys_monitor_.get_cpu_usage();
    status_msg.mem_usage = sys_monitor_.get_mem_usage();
    status_msg.disk_usage = sys_monitor_.get_disk_usage();
    status_msg.cpu_temp = sys_monitor_.get_cpu_temp();
    status_msg.gpu_usage = sys_monitor_.get_gpu_usage();
    status_msg.gpu_temp = sys_monitor_.get_gpu_temp();
    status_msg.gpu_mem_usage = sys_monitor_.get_gpu_mem();

    status_msg.vehicle_status_valid = vehicle_status.valid;
    status_msg.vehicle_navigation_active = vehicle_status.navigation_active;
    status_msg.vehicle_navigation_succeeded = vehicle_status.navigation_succeeded;
    status_msg.vehicle_progress_percentage = vehicle_status.progress_percentage;
    status_msg.vehicle_simple_status = vehicle_status.simple_status;
    status_msg.vehicle_error_message = vehicle_status.error_message;

    const auto & battery_state = data_store_.get_battery_state();
    if (
      battery_state.has_data &&
      (now - battery_state.last_seen).seconds() <= battery_state_timeout_s_)
    {
      status_msg.battery_temperature = battery_state.temperature;
      status_msg.battery_percentage = battery_state.percentage;
    }

    faults = fault_detector_.detect_faults(data_store_, now);
    for (const auto & fault : faults) {
      if (fault.action == ActionType::SUPERVISOR && should_publish_action(fault.module_name, fault.action, now)) {
        pending_faults.push_back(fault);
      }
    }
  }

  auto state_update = fault_state_coordinator_.update(faults);
  fault_edge_events = std::move(state_update.edge_events);
  safety_update = std::move(state_update.safety_update);

  pub_->publish(status_msg);
  publish_collision_zones();

  if (safety_update.has_value()) {
    if (!safety_update->active) {
      RCLCPP_WARN(get_logger(), "Safety state recovered: %s", safety_update->reason.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Safety state updated: command=%d reason=%s",
        static_cast<int>(safety_update->command), safety_update->reason.c_str());
    }
  }

  for (const auto & edge_event : fault_edge_events) {
    msg::FaultEvent event;
    const auto now_ns = now.nanoseconds();
    event.stamp.sec = static_cast<int32_t>(now_ns / 1000000000LL);
    event.stamp.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);
    event.module_name = edge_event.fault.module_name;
    event.fault_level = fault_level_to_msg(edge_event.fault.level);
    event.reason = edge_event.fault.reason;
    event.action = action_to_msg(edge_event.fault.action);
    event.edge = edge_event.edge == FaultEdgeType::TRIGGER ?
      msg::FaultEvent::EDGE_TRIGGER : msg::FaultEvent::EDGE_RECOVER;
    fault_event_pub_->publish(event);
  }

  for (const auto & fault : pending_faults) {
    if (fault.action == ActionType::SUPERVISOR) {
      msg::SupervisorCmd cmd;
      cmd.module_name = fault.module_name;
      supervisor_pub_->publish(cmd);
      RCLCPP_WARN(
        get_logger(), "Supervisor restart: %s - %s",
        fault.module_name.c_str(), fault.reason.c_str());
    }
  }
}

void Nav2MonitorNode::publish_collision_zones()
{
  if (!fault_detector_.collision_detection_enabled()) {
    return;
  }

  const auto & cfg = fault_detector_.get_collision_detection_config();
  for (const auto & zone : cfg.zones) {
    if (!zone.visualize || zone.points.size() < 3) {
      continue;
    }

    const auto pub_it = collision_zone_pubs_.find(zone.name);
    if (pub_it == collision_zone_pubs_.end()) {
      continue;
    }

    geometry_msgs::msg::PolygonStamped polygon_msg;
    polygon_msg.header.stamp = now();
    polygon_msg.header.frame_id = base_frame_id_;
    polygon_msg.polygon.points.reserve(zone.points.size());
    for (const auto & point : zone.points) {
      geometry_msgs::msg::Point32 p;
      p.x = static_cast<float>(point.x);
      p.y = static_cast<float>(point.y);
      p.z = 0.0F;
      polygon_msg.polygon.points.push_back(p);
    }
    pub_it->second->publish(polygon_msg);
  }
}

rcl_interfaces::msg::SetParametersResult Nav2MonitorNode::on_parameter_change(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  std::lock_guard<std::mutex> lock(mtx_);
  for (const auto & param : params) {
    if (param.get_name() == "target_nodes") {
      fallback_target_nodes_ = param.as_string_array();
      if (monitor_targets_from_fault_config_) {
        RCLCPP_WARN(get_logger(), "Ignore target_nodes update: monitor targets are from fault_config modules");
      } else {
        target_nodes_ = fallback_target_nodes_;
        RCLCPP_INFO(get_logger(), "Updated target_nodes: %zu nodes", target_nodes_.size());
      }
    } else if (param.get_name() == "watch_topics") {
      fallback_watch_topics_ = param.as_string_array();
      if (monitor_targets_from_fault_config_) {
        RCLCPP_WARN(get_logger(), "Ignore watch_topics update: monitor targets are from fault_config modules");
      } else {
        watch_topics_ = fallback_watch_topics_;
        RCLCPP_INFO(get_logger(), "Updated watch_topics: %zu topics", watch_topics_.size());
      }
    } else if (param.get_name() == "target_transforms") {
      target_transforms_.clear();
      for (const auto & tf_str : param.as_string_array()) {
        auto pos = tf_str.find("->");
        if (pos != std::string::npos) {
          target_transforms_.push_back({tf_str.substr(0, pos), tf_str.substr(pos + 2)});
        }
      }
      RCLCPP_INFO(get_logger(), "Updated target_transforms: %zu transforms", target_transforms_.size());
    } else if (param.get_name() == "timeout") {
      timeout_ = param.as_double();
      fault_detector_.set_feedback_default_max_stale(timeout_);
      RCLCPP_INFO(get_logger(), "Updated timeout: %.1f seconds", timeout_);
    } else if (param.get_name() == "safety_cooldown_s") {
      safety_cooldown_s_ = std::max(0.0, param.as_double());
      RCLCPP_INFO(get_logger(), "Updated safety_cooldown_s: %.2f", safety_cooldown_s_);
    } else if (param.get_name() == "supervisor_cooldown_s") {
      supervisor_cooldown_s_ = std::max(0.0, param.as_double());
      RCLCPP_INFO(get_logger(), "Updated supervisor_cooldown_s: %.2f", supervisor_cooldown_s_);
    } else if (param.get_name() == "battery_state_timeout_s") {
      battery_state_timeout_s_ = std::max(1.0, param.as_double());
      RCLCPP_INFO(get_logger(), "Updated battery_state_timeout_s: %.2f", battery_state_timeout_s_);
    }
  }

  return result;
}

}  // namespace nav2_monitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_monitor::Nav2MonitorNode>());
  rclcpp::shutdown();
  return 0;
}
