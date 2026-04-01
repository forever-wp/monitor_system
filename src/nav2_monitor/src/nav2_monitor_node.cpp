#include "nav2_monitor/nav2_monitor_node.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <rclcpp/serialized_message.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <limits>
#include <sstream>
#include <yaml-cpp/yaml.h>

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

  return config_file;
}

std::map<std::string, std::string> default_task_status_code_mappings()
{
  return {
    {"100", "default"},
    {"101", "default"},
    {"102", "default"},
    {"103", "default"},
    {"109", "default"},
    {"200", "elevator"},
    {"201", "elevator"},
    {"202", "elevator"},
    {"203", "elevator"},
    {"209", "elevator"},
    {"300", "todoor"},
    {"301", "todoor"},
    {"302", "todoor"},
    {"303", "todoor"},
    {"304", "todoor"},
    {"400", "default"},
    {"401", "default"},
    {"402", "default"},
    {"403", "default"}
  };
}

bool extract_ultrasonic_distances(const YAML::Node & node, std::vector<double> & distances)
{
  distances.clear();
  if (!node || !node.IsSequence()) {
    return false;
  }

  size_t numeric_count = 0;
  for (const auto & item : node) {
    try {
      distances.push_back(item.as<double>());
      ++numeric_count;
    } catch (...) {
      if (distances.empty() && item.IsScalar()) {
        try {
          (void)item.as<bool>();
          continue;
        } catch (...) {
        }
      }
      distances.clear();
      return false;
    }
  }
  return numeric_count >= 8;
}

bool parse_ultrasonic_json_payload(
  const std::string & payload,
  const std::string & distances_key,
  std::vector<double> & distances)
{
  distances.clear();
  if (payload.empty()) {
    return false;
  }

  try {
    const auto root = YAML::Load(payload);
    if (root.IsMap()) {
      if (!distances_key.empty() && root[distances_key] &&
        extract_ultrasonic_distances(root[distances_key], distances)) {
        return true;
      }
      for (const auto & entry : root) {
        if (extract_ultrasonic_distances(entry.second, distances)) {
          return true;
        }
      }
    } else if (extract_ultrasonic_distances(root, distances)) {
      return true;
    }
  } catch (...) {
  }

  std::stringstream ss(payload);
  std::string token;
  std::vector<std::string> tokens;
  while (std::getline(ss, token, ',')) {
    const auto begin = token.find_first_not_of(" \t\r\n");
    const auto end = token.find_last_not_of(" \t\r\n");
    if (begin == std::string::npos) {
      continue;
    }
    tokens.push_back(token.substr(begin, end - begin + 1));
  }

  size_t start_idx = 0;
  if (!tokens.empty()) {
    std::string lowered = tokens.front();
    std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if ((lowered == "true" || lowered == "false") && tokens.size() >= 9) {
      start_idx = 1;
    }
  }
  if (tokens.size() < start_idx + 8) {
    return false;
  }

  try {
    for (size_t idx = 0; idx < 8; ++idx) {
      distances.push_back(std::stod(tokens[start_idx + idx]));
    }
    return true;
  } catch (...) {
    distances.clear();
    return false;
  }
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
  const auto fault_event_topic = this->declare_parameter<std::string>(
    "fault_event_topic", "/nav2_monitor/fault_event");
  const auto supervisor_cmd_topic = this->declare_parameter<std::string>(
    "supervisor_cmd_topic", "/supervisor/cmd");
  const auto safety_cmd_topic = this->declare_parameter<std::string>(
    "safety_cmd_topic", "/safety_system/cmd");
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
  fault_event_pub_ = this->create_publisher<msg::FaultEvent>(fault_event_topic, 10);
  supervisor_pub_ = this->create_publisher<std_msgs::msg::String>(supervisor_cmd_topic, 10);
  fault_state_coordinator_.configure(this, safety_cmd_topic);
  monitor_reporter_.configure(this);

  base_fault_config_path_ = this->declare_parameter<std::string>("fault_config", "");
  fault_config_reload_enabled_ = this->declare_parameter<bool>("fault_config_reload_enabled", true);
  current_nav_task_ = this->declare_parameter<std::string>("current_nav_task", "default");
  task_status_topic_ = this->declare_parameter<std::string>("task_status_topic", "/task_status_code");
  resolved_base_fault_config_path_ = resolve_config_path(base_fault_config_path_);
  load_task_fault_config_mappings();
  load_task_status_code_mappings();
  task_fault_config_selector_.update_current_task(current_nav_task_);
  fault_detector_.set_feedback_default_max_stale(timeout_);
  update_task_selected_fault_config(true);

  algorithm_feedback_sub_ = this->create_subscription<msg::AlgorithmFeedback>(
    algorithm_feedback_topic_, rclcpp::QoS(50),
    std::bind(&Nav2MonitorNode::on_algorithm_feedback, this, std::placeholders::_1));
  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
    battery_state_topic_, rclcpp::SensorDataQoS(),
    std::bind(&Nav2MonitorNode::on_battery_state, this, std::placeholders::_1));
  configure_task_status_subscription();

  std::string vehicle_status_file = this->declare_parameter<std::string>(
    "vehicle_status_file", "/home/ry/.ros/navigate_status/navigate_todoor_status.json");
  vehicle_monitor_ = std::make_unique<VehicleStatusMonitor>(vehicle_status_file);

  scan_topology();

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
  data_store_.set_command_speed(speed, now);
}

void Nav2MonitorNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & v = msg->twist.twist.linear;
  const double speed = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
  const auto stamp = (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) ?
    rclcpp::Time(msg->header.stamp) : this->now();
  data_store_.set_odom_speed(speed, stamp);
}

void Nav2MonitorNode::on_battery_state(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
  const auto has_stamp = (msg->header.stamp.sec != 0) || (msg->header.stamp.nanosec != 0);
  const auto stamp = has_stamp ? rclcpp::Time(msg->header.stamp) : this->now();
  data_store_.set_battery_state(msg->temperature, msg->percentage, stamp);
}

void Nav2MonitorNode::on_chassis_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const auto stamp = (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) ?
    rclcpp::Time(msg->header.stamp) : this->now();

  if (!chassis_imu_time_initialized_) {
    chassis_imu_last_stamp_ = stamp;
    chassis_imu_time_initialized_ = true;
    return;
  }

  const double dt = (stamp - chassis_imu_last_stamp_).seconds();
  chassis_imu_last_stamp_ = stamp;
  if (dt <= 0.0 || dt > 0.5) {
    return;
  }

  const double accel_x = msg->linear_acceleration.x;
  const double yaw_rate = msg->angular_velocity.z;
  const bool static_motion =
    std::fabs(chassis_imu_speed_estimate_) < chassis_imu_static_command_threshold_ &&
    std::fabs(yaw_rate) < chassis_imu_yaw_rate_threshold_;

  if (static_motion) {
    chassis_imu_bias_samples_.push_back(accel_x);
    if (static_cast<int>(chassis_imu_bias_samples_.size()) > chassis_imu_bias_calibration_samples_) {
      chassis_imu_bias_samples_.erase(chassis_imu_bias_samples_.begin());
    }

    if (static_cast<int>(chassis_imu_bias_samples_.size()) >= chassis_imu_bias_calibration_samples_) {
      double sum = 0.0;
      for (const auto sample : chassis_imu_bias_samples_) {
        sum += sample;
      }
      chassis_imu_acc_bias_ = sum / static_cast<double>(chassis_imu_bias_samples_.size());
      chassis_imu_bias_calibrated_ = true;
    }
  }

  if (chassis_imu_bias_calibrated_) {
    const double corrected_accel = accel_x - chassis_imu_acc_bias_;
    chassis_imu_speed_estimate_ += corrected_accel * dt;
    chassis_imu_speed_estimate_ *= chassis_imu_decay_rate_;
    chassis_imu_speed_estimate_ = std::clamp(chassis_imu_speed_estimate_, -3.0, 3.0);

    if (static_motion) {
      chassis_imu_speed_estimate_ *= 0.5;
      if (std::fabs(chassis_imu_speed_estimate_) < chassis_imu_speed_threshold_) {
        chassis_imu_speed_estimate_ = 0.0;
      }
    }
  }

  data_store_.set_imu_motion(chassis_imu_speed_estimate_, yaw_rate, stamp);
}

void Nav2MonitorNode::on_collision_prediction_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  data_store_.set_prediction_motion(
    msg->linear.x, msg->linear.y, msg->angular.z, this->now());
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
  data_store_.set_collision_points("pointcloud", points, stamp);
}

void Nav2MonitorNode::on_collision_ultrasonic(const std_msgs::msg::String::SharedPtr msg)
{
  const auto & cfg = fault_detector_.get_collision_detection_config();
  if (cfg.ultrasonic_sensors.empty()) {
    return;
  }

  std::vector<double> distances;
  if (!parse_ultrasonic_json_payload(msg->data, cfg.ultrasonic_distances_key, distances)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Collision ultrasonic parse failed: topic=%s payload='%s'",
      cfg.ultrasonic_topic.c_str(), msg->data.c_str());
    return;
  }

  std::vector<CollisionPoint> points;
  points.reserve(cfg.ultrasonic_sensors.size());
  for (const auto & sensor : cfg.ultrasonic_sensors) {
    if (!sensor.enabled || sensor.index >= distances.size()) {
      continue;
    }

    const double raw_distance = distances[sensor.index];
    if (!std::isfinite(raw_distance) || raw_distance <= 0.0) {
      continue;
    }

    if (raw_distance >= cfg.ultrasonic_out_of_range_value || raw_distance > sensor.max_distance) {
      continue;
    }

    const double effective_distance = std::max(raw_distance, cfg.ultrasonic_blind_distance);

    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const double yaw = sensor.yaw_deg * kDegToRad;
    points.push_back(CollisionPoint{
      sensor.x + effective_distance * std::cos(yaw),
      sensor.y + effective_distance * std::sin(yaw),
      sensor.weight});
  }

  data_store_.set_collision_points("ultrasonic", points, this->now());
}

void Nav2MonitorNode::try_subscribe_moto_topic()
{
  if (!fault_detector_.chassis_stationary_enabled() || moto_topic_.empty() || moto_sub_) {
    return;
  }
  if (!fault_detector_.get_chassis_stationary_config().imu_topic.empty()) {
    return;
  }

  const auto topic_types = this->get_topic_names_and_types();
  auto it = topic_types.find(moto_topic_);
  if (it == topic_types.end() || it->second.empty()) {
    return;
  }

  moto_topic_type_ = it->second.front();
  auto moto_fallback = rclcpp::SensorDataQoS();
  moto_fallback.keep_last(10);
  const auto moto_qos = build_topic_subscription_qos(moto_topic_, moto_fallback, 10);
  moto_sub_ = this->create_generic_subscription(
    moto_topic_, moto_topic_type_, moto_qos,
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
  data_store_.add_feedback_sample(
    msg->module_name, msg->topic_name, msg->metric_name, msg->value, msg->valid, stamp);
}

void Nav2MonitorNode::on_task_status(const master_interfaces::msg::TaskStatus::SharedPtr msg)
{
  const std::string raw_code = msg ? TaskStatusMessageAdapter::extract_code(*msg) : std::string();
  const std::string mapped_task = task_status_mapper_.resolve_task_for_code(raw_code);
  if (mapped_task.empty()) {
    RCLCPP_WARN(
      get_logger(), "Ignore task_status code '%s': no task mapping configured", raw_code.c_str());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (mapped_task == current_nav_task_) {
      RCLCPP_DEBUG(
        get_logger(), "Ignore task_status code '%s': current_nav_task already '%s'",
        raw_code.c_str(), current_nav_task_.c_str());
      return;
    }
    pending_task_switch_source_ = std::string("task_status code '") + raw_code + "'";
  }

  const auto result = this->set_parameters_atomically({rclcpp::Parameter("current_nav_task", mapped_task)});

  {
    std::lock_guard<std::mutex> lock(mtx_);
    pending_task_switch_source_.clear();
  }

  if (!result.successful) {
    RCLCPP_WARN(
      get_logger(), "Failed to apply task_status code '%s' -> '%s': %s",
      raw_code.c_str(), mapped_task.c_str(), result.reason.c_str());
  }
}

rclcpp::QoS Nav2MonitorNode::build_topic_subscription_qos(
  const std::string & topic, const rclcpp::QoS & fallback, size_t max_depth) const
{
  auto qos = rclcpp::QoS(std::max<size_t>(1, std::min<size_t>(fallback.get_rmw_qos_profile().depth, max_depth)));
  qos.history(rclcpp::HistoryPolicy::KeepLast);
  qos.reliability(fallback.reliability());
  qos.durability(fallback.durability());

  try {
    auto infos = this->get_publishers_info_by_topic(topic);
    if (infos.empty()) {
      return qos;
    }

    size_t depth = std::max<size_t>(1, qos.get_rmw_qos_profile().depth);
    bool use_reliable = qos.reliability() == rclcpp::ReliabilityPolicy::Reliable;
    bool use_transient_local = qos.durability() == rclcpp::DurabilityPolicy::TransientLocal;

    for (const auto & info : infos) {
      const auto & profile = info.qos_profile();
      depth = std::max<size_t>(depth, std::max<size_t>(1, profile.depth()));
      if (profile.reliability() == rclcpp::ReliabilityPolicy::Reliable) {
        use_reliable = true;
      } else {
        use_reliable = false;
      }
      if (profile.durability() == rclcpp::DurabilityPolicy::TransientLocal) {
        use_transient_local = true;
      } else {
        use_transient_local = false;
      }
    }

    depth = std::max<size_t>(1, std::min<size_t>(depth, max_depth));
    qos = rclcpp::QoS(depth);
    qos.history(rclcpp::HistoryPolicy::KeepLast);
    qos.reliability(use_reliable ? rclcpp::ReliabilityPolicy::Reliable : rclcpp::ReliabilityPolicy::BestEffort);
    qos.durability(use_transient_local ? rclcpp::DurabilityPolicy::TransientLocal : rclcpp::DurabilityPolicy::Volatile);
  } catch (const std::exception &) {
  }

  return qos;
}

rclcpp::QoS Nav2MonitorNode::build_watch_topic_qos(const std::string & topic, const std::string & type) const
{
  const bool is_imu = type == "sensor_msgs/msg/Imu" || topic.find("/imu") != std::string::npos;
  const bool is_scan = type == "sensor_msgs/msg/LaserScan" || topic == "/scan";
  const bool is_pointcloud = type == "sensor_msgs/msg/PointCloud2";
  const bool is_range = type == "sensor_msgs/msg/Range";
  const bool is_image = type == "sensor_msgs/msg/Image" || type == "sensor_msgs/msg/CompressedImage";

  if (is_imu || is_scan || is_pointcloud || is_range) {
    auto qos = rclcpp::SensorDataQoS();
    const size_t max_depth = is_imu ? 5 : (is_scan ? 10 : (is_pointcloud ? 3 : 5));
    qos.keep_last(max_depth);
    return build_topic_subscription_qos(topic, qos, max_depth);
  }

  if (is_image) {
    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(2);
    return build_topic_subscription_qos(topic, qos, 2);
  }

  return build_topic_subscription_qos(topic, rclcpp::QoS(10), 10);
}

rclcpp::Time Nav2MonitorNode::stamp_or_now(const builtin_interfaces::msg::Time & stamp) const
{
  const bool has_stamp = (stamp.sec != 0) || (stamp.nanosec != 0);
  return has_stamp ? rclcpp::Time(stamp) : this->now();
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

bool Nav2MonitorNode::update_current_nav_task_locked(
  const std::string & task_name,
  const std::string & change_source)
{
  const std::string normalized = task_name.empty() ? "default" : task_name;
  if (normalized == current_nav_task_) {
    RCLCPP_DEBUG(
      get_logger(), "Ignore current_nav_task update from %s: already '%s'",
      change_source.c_str(), current_nav_task_.c_str());
    return false;
  }

  current_nav_task_ = normalized;
  (void)task_fault_config_selector_.update_current_task(current_nav_task_);
  RCLCPP_INFO(
    get_logger(), "Updated current_nav_task via %s: %s",
    change_source.c_str(), current_nav_task_.c_str());
  return true;
}

void Nav2MonitorNode::load_task_status_code_mappings()
{
  const std::string prefix = "task_status_code_mappings.";
  const auto defaults = default_task_status_code_mappings();
  std::unordered_set<std::string> param_names;
  for (const auto & entry : defaults) {
    param_names.insert(prefix + entry.first);
  }

  const auto listed = this->list_parameters({prefix}, 2);
  for (const auto & name : listed.names) {
    if (name.rfind(prefix, 0) == 0) {
      param_names.insert(name);
    }
  }

  const auto & overrides = this->get_node_parameters_interface()->get_parameter_overrides();
  for (const auto & [name, _] : overrides) {
    if (name.rfind(prefix, 0) == 0) {
      param_names.insert(name);
    }
  }

  task_status_code_mappings_.clear();
  for (const auto & name : param_names) {
    const std::string code = name.substr(prefix.size());
    const auto default_it = defaults.find(code);
    const std::string default_value =
      (default_it != defaults.end()) ? default_it->second : std::string();
    if (!this->has_parameter(name)) {
      this->declare_parameter<std::string>(name, default_value);
    }

    std::string value;
    try {
      value = this->get_parameter(name).as_string();
    } catch (...) {
      value = default_value;
    }

    if (!value.empty()) {
      task_status_code_mappings_[code] = value;
    }
  }

  task_status_mapper_.configure(task_status_code_mappings_);
  RCLCPP_INFO(get_logger(), "Loaded %zu task_status code mappings", task_status_code_mappings_.size());
}

void Nav2MonitorNode::configure_task_status_subscription()
{
  task_status_sub_.reset();
  if (task_status_topic_.empty()) {
    RCLCPP_INFO(get_logger(), "task_status subscription disabled: empty topic");
    return;
  }

  task_status_sub_ = this->create_subscription<master_interfaces::msg::TaskStatus>(
    task_status_topic_, rclcpp::QoS(10),
    std::bind(&Nav2MonitorNode::on_task_status, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "Subscribed task_status topic: %s", task_status_topic_.c_str());
}

void Nav2MonitorNode::load_task_fault_config_mappings()
{
  const std::string prefix = "task_fault_configs.";
  std::unordered_set<std::string> param_names{
    prefix + "default",
    prefix + "todoor",
    prefix + "elevator",
    prefix + "reverse"
  };

  const auto listed = this->list_parameters({prefix}, 2);
  for (const auto & name : listed.names) {
    if (name.rfind(prefix, 0) == 0) {
      param_names.insert(name);
    }
  }

  const auto & overrides = this->get_node_parameters_interface()->get_parameter_overrides();
  for (const auto & [name, _] : overrides) {
    if (name.rfind(prefix, 0) == 0) {
      param_names.insert(name);
    }
  }

  task_fault_config_mappings_.clear();
  for (const auto & name : param_names) {
    const bool is_default = (name == prefix + std::string("default"));
    const std::string default_value = is_default ? base_fault_config_path_ : std::string();
    if (!this->has_parameter(name)) {
      this->declare_parameter<std::string>(name, default_value);
    }

    std::string value;
    try {
      value = this->get_parameter(name).as_string();
    } catch (...) {
      value = default_value;
    }

    task_fault_config_mappings_[name.substr(prefix.size())] = value;
  }

  const auto default_it = task_fault_config_mappings_.find("default");
  const std::string selector_default =
    (default_it != task_fault_config_mappings_.end() && !default_it->second.empty()) ?
    default_it->second : base_fault_config_path_;
  task_fault_config_selector_.configure(selector_default, task_fault_config_mappings_);
}

void Nav2MonitorNode::update_task_selected_fault_config(bool force_reload)
{
  const std::string selected_fault_config = task_fault_config_selector_.resolve_fault_config_for_task();
  const std::string resolved_selected_fault_config = resolve_config_path(selected_fault_config);
  const bool path_changed =
    selected_fault_config != fault_config_path_ ||
    resolved_selected_fault_config != resolved_fault_config_path_;

  if (!path_changed && !force_reload) {
    task_fault_config_selector_.clear_task_changed();
    return;
  }

  fault_config_path_ = selected_fault_config;
  resolved_fault_config_path_ = resolved_selected_fault_config;
  fault_config_watcher_.configure(fault_config_path_, resolved_fault_config_path_);

  RCLCPP_INFO(
    get_logger(),
    "Selected fault_config for task '%s': active='%s' resolved='%s'",
    task_fault_config_selector_.current_task().c_str(),
    fault_config_path_.c_str(),
    resolved_fault_config_path_.c_str());

  if (fault_config_path_.empty()) {
    apply_loaded_fault_config();
    fault_config_watcher_.sync_current_state();
  } else {
    (void)reload_fault_config_if_needed(true);
  }
  task_fault_config_selector_.clear_task_changed();
}

bool Nav2MonitorNode::reload_fault_config_if_needed(bool force)
{
  if (fault_config_path_.empty()) {
    return false;
  }

  if (!force) {
    if (!fault_config_reload_enabled_ || !fault_config_watcher_.enabled() ||
      !fault_config_watcher_.poll_changed()) {
      return false;
    }
    RCLCPP_INFO(
      get_logger(), "fault_config changed, reloading: param='%s', resolved='%s'",
      fault_config_path_.c_str(), resolved_fault_config_path_.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(), "fault_config param='%s', resolved='%s'",
      fault_config_path_.c_str(), resolved_fault_config_path_.c_str());
  }

  std::ifstream config_stream(resolved_fault_config_path_);
  if (!config_stream.good()) {
    RCLCPP_ERROR(
      get_logger(),
      "fault_config not readable: %s (resolved: %s), keep current config/fallback targets",
      fault_config_path_.c_str(), resolved_fault_config_path_.c_str());
  } else {
    fault_detector_.load_config(resolved_fault_config_path_);
  }

  apply_loaded_fault_config();
  fault_config_watcher_.sync_current_state();
  return true;
}

void Nav2MonitorNode::clear_watch_topic_subscriptions()
{
  topic_subs_.clear();
  topic_info_.clear();
}

void Nav2MonitorNode::apply_loaded_fault_config()
{
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!fault_config_path_.empty() && fault_detector_.has_module_configs()) {
      target_nodes_ = fault_detector_.get_monitored_nodes();
      watch_topics_ = fault_detector_.get_watched_topics();
      monitor_targets_from_fault_config_ = true;
      RCLCPP_INFO(
        get_logger(), "Monitor targets loaded from fault_config modules: %zu nodes, %zu topics",
        target_nodes_.size(), watch_topics_.size());
    } else {
      target_nodes_ = fallback_target_nodes_;
      watch_topics_ = fallback_watch_topics_;
      monitor_targets_from_fault_config_ = false;
      RCLCPP_INFO(
        get_logger(),
        "fault_config missing/empty modules, fallback to target_nodes/watch_topics params");
    }

    target_transforms_.clear();
    const auto & fault_transforms = fault_detector_.get_monitored_transforms();
    std::vector<std::string> fallback_transforms;
    const std::vector<std::string> * active_transforms = &fault_transforms;
    if (fault_config_path_.empty() || fault_transforms.empty()) {
      fallback_transforms = this->get_parameter("target_transforms").as_string_array();
      active_transforms = &fallback_transforms;
    }
    for (const auto & tf_str : *active_transforms) {
      auto pos = tf_str.find("->");
      if (pos != std::string::npos) {
        target_transforms_.push_back({tf_str.substr(0, pos), tf_str.substr(pos + 2)});
      }
    }

    clear_watch_topic_subscriptions();
  }

  configure_chassis_monitoring();
  configure_collision_monitoring();
  subscribe_watch_topics();
  try_subscribe_moto_topic();
}

void Nav2MonitorNode::configure_chassis_monitoring()
{
  command_sub_.reset();
  odom_sub_.reset();
  chassis_imu_sub_.reset();
  moto_sub_.reset();
  moto_topic_type_.clear();
  command_topic_.clear();
  moto_topic_.clear();
  odom_topic_.clear();
  chassis_imu_topic_.clear();
  chassis_imu_time_initialized_ = false;
  chassis_imu_speed_estimate_ = 0.0;
  chassis_imu_acc_bias_ = 0.0;
  chassis_imu_bias_calibrated_ = false;
  chassis_imu_bias_samples_.clear();

  if (fault_config_path_.empty() || !fault_detector_.chassis_stationary_enabled()) {
    return;
  }

  const auto & cfg = fault_detector_.get_chassis_stationary_config();
  command_topic_ = cfg.command_topic;
  moto_topic_ = cfg.moto_topic;
  odom_topic_ = cfg.odom_topic;
  chassis_imu_topic_ = cfg.imu_topic;
  chassis_imu_speed_threshold_ = cfg.imu_speed_threshold;
  chassis_imu_yaw_rate_threshold_ = cfg.imu_yaw_rate_threshold;
  chassis_imu_static_command_threshold_ = cfg.imu_static_command_threshold;
  chassis_imu_decay_rate_ = cfg.imu_decay_rate;
  chassis_imu_bias_calibration_samples_ = cfg.imu_bias_calibration_samples;

  const bool use_imu_as_truth = !chassis_imu_topic_.empty();

  if (!use_imu_as_truth && !command_topic_.empty()) {
    command_sub_ = this->create_subscription<std_msgs::msg::String>(
      command_topic_, rclcpp::QoS(20),
      std::bind(&Nav2MonitorNode::on_command, this, std::placeholders::_1));
  }
  if (!use_imu_as_truth && !odom_topic_.empty()) {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Nav2MonitorNode::on_odom, this, std::placeholders::_1));
  }
  if (!chassis_imu_topic_.empty()) {
    chassis_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      chassis_imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Nav2MonitorNode::on_chassis_imu, this, std::placeholders::_1));
  }
  RCLCPP_INFO(
    get_logger(), "Chassis stationary judge enabled: command=%s moto=%s odom=%s imu=%s",
    command_topic_.c_str(), moto_topic_.c_str(),
    odom_topic_.empty() ? "<disabled>" : odom_topic_.c_str(),
    chassis_imu_topic_.empty() ? "<disabled>" : chassis_imu_topic_.c_str());
}

void Nav2MonitorNode::configure_collision_monitoring()
{
  collision_prediction_cmd_vel_sub_.reset();
  collision_scan_sub_.reset();
  collision_pointcloud_sub_.reset();
  collision_ultrasonic_sub_.reset();
  collision_zone_pubs_.clear();
  collision_ttc_markers_pub_.reset();

  if (fault_config_path_.empty() || !fault_detector_.collision_detection_enabled()) {
    return;
  }

  const auto & cfg = fault_detector_.get_collision_detection_config();
  if (!cfg.prediction_speed_topic.empty()) {
    collision_prediction_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cfg.prediction_speed_topic, rclcpp::QoS(20),
      std::bind(&Nav2MonitorNode::on_collision_prediction_cmd_vel, this, std::placeholders::_1));
  }
  if (!cfg.scan_topic.empty()) {
    auto scan_fallback = rclcpp::SensorDataQoS();
    scan_fallback.keep_last(10);
    const auto scan_qos = build_topic_subscription_qos(cfg.scan_topic, scan_fallback, 10);
    collision_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      cfg.scan_topic, scan_qos,
      std::bind(&Nav2MonitorNode::on_collision_scan, this, std::placeholders::_1));
  }
  if (!cfg.pointcloud_topic.empty()) {
    auto pointcloud_fallback = rclcpp::SensorDataQoS();
    pointcloud_fallback.keep_last(50);
    const auto pointcloud_qos = build_topic_subscription_qos(cfg.pointcloud_topic, pointcloud_fallback, 3);
    collision_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cfg.pointcloud_topic, pointcloud_qos,
      std::bind(&Nav2MonitorNode::on_collision_pointcloud, this, std::placeholders::_1));
  }
  if (!cfg.ultrasonic_topic.empty()) {
    auto ultrasonic_fallback = rclcpp::QoS(10);
    ultrasonic_fallback.keep_last(10);
    const auto ultrasonic_qos = build_topic_subscription_qos(cfg.ultrasonic_topic, ultrasonic_fallback, 10);
    collision_ultrasonic_sub_ = this->create_subscription<std_msgs::msg::String>(
      cfg.ultrasonic_topic, ultrasonic_qos,
      std::bind(&Nav2MonitorNode::on_collision_ultrasonic, this, std::placeholders::_1));
  }
  for (const auto & zone : cfg.zones) {
    if (!zone.visualize || zone.points.size() < 3) {
      continue;
    }
    auto qos = rclcpp::QoS(1).transient_local().reliable();
    collision_zone_pubs_[zone.name] = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
      zone.polygon_pub_topic, qos);
  }
  if (cfg.ttc_visualization_enabled) {
    auto qos = rclcpp::QoS(1).transient_local().reliable();
    collision_ttc_markers_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/nav2_monitor/collision_ttc_markers", qos);
  }
  publish_collision_zones();
  publish_collision_ttc_markers();
  RCLCPP_INFO(
    get_logger(), "Collision detection enabled: scan=%s pointcloud=%s ultrasonic=%s",
    cfg.scan_topic.c_str(), cfg.pointcloud_topic.c_str(), cfg.ultrasonic_topic.c_str());
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

    const auto qos = build_watch_topic_qos(topic, type);

    if (type == "sensor_msgs/msg/Imu") {
      auto sub = this->create_subscription<sensor_msgs::msg::Imu>(
        topic, qos,
        [this, topic](const sensor_msgs::msg::Imu::SharedPtr msg) {
          data_store_.add_watch_topic_sample(topic, stamp_or_now(msg->header.stamp), true);
        });
      topic_subs_[topic] = sub;
      continue;
    }

    if (type == "sensor_msgs/msg/LaserScan") {
      auto sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        topic, qos,
        [this, topic](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          data_store_.add_watch_topic_sample(topic, stamp_or_now(msg->header.stamp), true);
        });
      topic_subs_[topic] = sub;
      continue;
    }

    if (type == "sensor_msgs/msg/PointCloud2") {
      auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, qos,
        [this, topic](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          data_store_.add_watch_topic_sample(topic, stamp_or_now(msg->header.stamp), true);
        });
      topic_subs_[topic] = sub;
      continue;
    }

    if (type == "nav_msgs/msg/Odometry") {
      auto sub = this->create_subscription<nav_msgs::msg::Odometry>(
        topic, qos,
        [this, topic](const nav_msgs::msg::Odometry::SharedPtr msg) {
          data_store_.add_watch_topic_sample(topic, stamp_or_now(msg->header.stamp), true);
        });
      topic_subs_[topic] = sub;
      continue;
    }

    auto sub = this->create_generic_subscription(
      topic, type, qos,
      [this, topic](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        if (msg->size() == 0) {
          data_store_.add_watch_topic_sample(topic, this->now(), false);
          const auto * state = data_store_.get_watch_topic_state(topic);
          const size_t empty_count = state == nullptr ? 0U : state->empty_msg_count;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Topic '%s' empty msg (count: %zu)", topic.c_str(), empty_count);
          return;
        }

        auto now = this->now();
        data_store_.add_watch_topic_sample(topic, now, true);
      });
    topic_subs_[topic] = sub;
  }
}

void Nav2MonitorNode::scan_topology()
{
  (void)reload_fault_config_if_needed(false);
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
      const bool require_frequency = fault_detector_.is_watch_topic_frequency_required(topic);
      const bool topic_present = info != nullptr && info->has_publisher;
      const bool topic_valid = topic_present && (!require_frequency || info->has_valid_data);
      if (topic_valid) {
        status_msg.active_topics.push_back(topic);
        status_msg.topic_frequencies.push_back(
          info != nullptr ? static_cast<float>(info->frequency) : 0.0F);
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
  monitor_reporter_.publish_heartbeat(status_msg, now);
  publish_collision_zones();
  publish_collision_ttc_markers();

  if (safety_update.has_value()) {
    nav2_monitor::msg::SafetyCmd reporter_safety_cmd;
    if (safety_update->active) {
      reporter_safety_cmd.action = static_cast<uint8_t>(safety_update->command);
      reporter_safety_cmd.slow_down_percentage = static_cast<float>(safety_update->slow_down_percentage);
      reporter_safety_cmd.reason = safety_update->reason;
    } else {
      reporter_safety_cmd.action = nav2_monitor::msg::SafetyCmd::RESUME;
      reporter_safety_cmd.slow_down_percentage = 0.0F;
      reporter_safety_cmd.reason = safety_update->reason;
    }
    monitor_reporter_.cache_safety_cmd(reporter_safety_cmd, now);
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
    monitor_reporter_.publish_fault_event_json(event, now);
  }

  for (const auto & fault : pending_faults) {
    if (fault.action == ActionType::SUPERVISOR) {
      std_msgs::msg::String cmd;
      std::ostringstream oss;
      oss << '{'
          << "\"module_name\":\"" << fault.module_name << "\","
          << "\"nodes_to_restart\":[],"
          << "\"reason\":\"" << fault.reason << "\"}";
      cmd.data = oss.str();
      supervisor_pub_->publish(cmd);
      monitor_reporter_.cache_supervisor_json(cmd.data, now);
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

void Nav2MonitorNode::publish_collision_ttc_markers()
{
  if (!collision_ttc_markers_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  const auto stamp = now();

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header.stamp = stamp;
  clear_marker.header.frame_id = base_frame_id_;
  clear_marker.ns = "collision_ttc";
  clear_marker.id = 0;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  const auto & vis = fault_detector_.get_collision_ttc_visualization();
  if (!vis.enabled || !vis.active) {
    collision_ttc_markers_pub_->publish(marker_array);
    return;
  }

  visualization_msgs::msg::Marker trajectory_marker;
  trajectory_marker.header.stamp = stamp;
  trajectory_marker.header.frame_id = base_frame_id_;
  trajectory_marker.ns = "collision_ttc_trajectory";
  trajectory_marker.id = 1;
  trajectory_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  trajectory_marker.action = visualization_msgs::msg::Marker::ADD;
  trajectory_marker.scale.x = 0.03;
  trajectory_marker.color.r = 0.1F;
  trajectory_marker.color.g = 0.9F;
  trajectory_marker.color.b = 0.2F;
  trajectory_marker.color.a = 0.95F;
  for (const auto & point : vis.trajectory_points) {
    geometry_msgs::msg::Point p;
    p.x = point.x;
    p.y = point.y;
    p.z = 0.03;
    trajectory_marker.points.push_back(p);
  }
  marker_array.markers.push_back(trajectory_marker);

  int footprint_marker_id = 100;
  for (const auto & polygon : vis.footprint_samples) {
    if (polygon.empty()) {
      continue;
    }
    visualization_msgs::msg::Marker footprint_marker;
    footprint_marker.header.stamp = stamp;
    footprint_marker.header.frame_id = base_frame_id_;
    footprint_marker.ns = "collision_ttc_footprint";
    footprint_marker.id = footprint_marker_id++;
    footprint_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    footprint_marker.action = visualization_msgs::msg::Marker::ADD;
    footprint_marker.scale.x = 0.01;
    footprint_marker.color.r = 0.2F;
    footprint_marker.color.g = 0.7F;
    footprint_marker.color.b = 1.0F;
    footprint_marker.color.a = 0.45F;
    for (const auto & point : polygon) {
      geometry_msgs::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = 0.01;
      footprint_marker.points.push_back(p);
    }
    geometry_msgs::msg::Point close_p;
    close_p.x = polygon.front().x;
    close_p.y = polygon.front().y;
    close_p.z = 0.01;
    footprint_marker.points.push_back(close_p);
    marker_array.markers.push_back(footprint_marker);
  }

  visualization_msgs::msg::Marker collision_point_marker;
  collision_point_marker.header.stamp = stamp;
  collision_point_marker.header.frame_id = base_frame_id_;
  collision_point_marker.ns = "collision_ttc_point";
  collision_point_marker.id = 2;
  collision_point_marker.type = visualization_msgs::msg::Marker::SPHERE;
  collision_point_marker.action = visualization_msgs::msg::Marker::ADD;
  collision_point_marker.pose.position.x = vis.collision_point.x;
  collision_point_marker.pose.position.y = vis.collision_point.y;
  collision_point_marker.pose.position.z = 0.05;
  collision_point_marker.pose.orientation.w = 1.0;
  collision_point_marker.scale.x = 0.08;
  collision_point_marker.scale.y = 0.08;
  collision_point_marker.scale.z = 0.08;
  collision_point_marker.color.r = 1.0F;
  collision_point_marker.color.g = 0.2F;
  collision_point_marker.color.b = 0.1F;
  collision_point_marker.color.a = 0.95F;
  marker_array.markers.push_back(collision_point_marker);

  visualization_msgs::msg::Marker text_marker;
  text_marker.header.stamp = stamp;
  text_marker.header.frame_id = base_frame_id_;
  text_marker.ns = "collision_ttc_text";
  text_marker.id = 3;
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::msg::Marker::ADD;
  text_marker.pose.position.x = vis.collision_point.x;
  text_marker.pose.position.y = vis.collision_point.y;
  text_marker.pose.position.z = 0.35;
  text_marker.pose.orientation.w = 1.0;
  text_marker.scale.z = 0.12;
  text_marker.color.r = 1.0F;
  text_marker.color.g = 1.0F;
  text_marker.color.b = 1.0F;
  text_marker.color.a = 1.0F;
  std::ostringstream oss;
  oss << vis.zone_name << " ttc=" << std::fixed << std::setprecision(2) << vis.ttc_s;
  if (vis.min_clearance >= 0.0) {
    oss << " clr=" << std::fixed << std::setprecision(2) << vis.min_clearance;
  }
  text_marker.text = oss.str();
  marker_array.markers.push_back(text_marker);

  collision_ttc_markers_pub_->publish(marker_array);
}

rcl_interfaces::msg::SetParametersResult Nav2MonitorNode::on_parameter_change(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool fault_config_changed = false;
  bool fault_config_cleared = false;
  bool task_status_subscription_changed = false;
  bool task_status_code_mappings_changed = false;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    for (const auto & param : params) {
      if (param.get_name() == "target_nodes") {
        fallback_target_nodes_ = param.as_string_array();
        if (monitor_targets_from_fault_config_) {
          RCLCPP_WARN(get_logger(), "Ignore target_nodes update: monitor targets are from fault_config modules");
        } else {
          target_nodes_ = fallback_target_nodes_;
          clear_watch_topic_subscriptions();
          RCLCPP_INFO(get_logger(), "Updated target_nodes: %zu nodes", target_nodes_.size());
        }
      } else if (param.get_name() == "watch_topics") {
        fallback_watch_topics_ = param.as_string_array();
        if (monitor_targets_from_fault_config_) {
          RCLCPP_WARN(get_logger(), "Ignore watch_topics update: monitor targets are from fault_config modules");
        } else {
          watch_topics_ = fallback_watch_topics_;
          clear_watch_topic_subscriptions();
          RCLCPP_INFO(get_logger(), "Updated watch_topics: %zu topics", watch_topics_.size());
        }
      } else if (param.get_name() == "fault_config") {
        base_fault_config_path_ = param.as_string();
        resolved_base_fault_config_path_ = resolve_config_path(base_fault_config_path_);
        fault_config_changed = true;
        fault_config_cleared = base_fault_config_path_.empty();
        RCLCPP_INFO(
          get_logger(), "Updated base fault_config: param='%s', resolved='%s'",
          base_fault_config_path_.c_str(), resolved_base_fault_config_path_.c_str());
      } else if (param.get_name() == "current_nav_task") {
        const std::string change_source = pending_task_switch_source_.empty() ?
          std::string("param current_nav_task") : pending_task_switch_source_;
        fault_config_changed =
          update_current_nav_task_locked(param.as_string(), change_source) || fault_config_changed;
      } else if (param.get_name() == "task_status_topic") {
        task_status_topic_ = param.as_string();
        task_status_subscription_changed = true;
        RCLCPP_INFO(get_logger(), "Updated task_status_topic: %s", task_status_topic_.c_str());
      } else if (param.get_name().rfind("task_status_code_mappings.", 0) == 0) {
        task_status_code_mappings_changed = true;
        RCLCPP_INFO(get_logger(), "Updated task status code mapping: %s", param.get_name().c_str());
      } else if (param.get_name().rfind("task_fault_configs.", 0) == 0) {
        fault_config_changed = true;
        RCLCPP_INFO(get_logger(), "Updated task fault config mapping: %s", param.get_name().c_str());
      } else if (param.get_name() == "fault_config_reload_enabled") {
        fault_config_reload_enabled_ = param.as_bool();
        RCLCPP_INFO(get_logger(), "Updated fault_config_reload_enabled: %s",
          fault_config_reload_enabled_ ? "true" : "false");
      } else if (param.get_name() == "target_transforms") {
        target_transforms_.clear();
        for (const auto & tf_str : param.as_string_array()) {
          auto pos = tf_str.find("->");
          if (pos != std::string::npos) {
            target_transforms_.push_back({tf_str.substr(0, pos), tf_str.substr(pos + 2)});
          }
        }
        if (!fault_config_path_.empty() && !fault_detector_.get_monitored_transforms().empty()) {
          RCLCPP_WARN(get_logger(), "target_transforms param updated but fault_config target_transforms takes precedence");
        } else {
          RCLCPP_INFO(get_logger(), "Updated target_transforms: %zu transforms", target_transforms_.size());
        }
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
  }

  if (task_status_code_mappings_changed) {
    load_task_status_code_mappings();
  }
  if (task_status_subscription_changed) {
    configure_task_status_subscription();
  }

  if (fault_config_changed) {
    load_task_fault_config_mappings();
    if (task_fault_config_selector_.current_task() != current_nav_task_) {
      (void)task_fault_config_selector_.update_current_task(current_nav_task_);
    }
    if (fault_config_cleared && task_fault_config_mappings_.find("default") == task_fault_config_mappings_.end()) {
      apply_loaded_fault_config();
      fault_config_watcher_.sync_current_state();
    } else {
      update_task_selected_fault_config(true);
    }
  } else {
    subscribe_watch_topics();
    try_subscribe_moto_topic();
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
