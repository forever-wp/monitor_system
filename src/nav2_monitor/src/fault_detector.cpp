#include "nav2_monitor/fault_detector.hpp"
#include "nav2_monitor/monitor_data_store.hpp"
#include "nav2_monitor/chassis_evaluator.hpp"
#include "nav2_monitor/collision_evaluator.hpp"
#include "nav2_monitor/feedback_rule_evaluator.hpp"
#include "nav2_monitor/watch_topic_evaluator.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace nav2_monitor
{

namespace
{
constexpr size_t kFeedbackWindowSize = 10;

std::string to_lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

bool parse_fault_level(const YAML::Node & node, FaultLevel & level)
{
  if (!node) {
    return false;
  }

  if (!node.IsScalar()) {
    return false;
  }

  const std::string raw = to_lower(node.as<std::string>());
  if (raw == "warning") {
    level = FaultLevel::WARNING;
    return true;
  }
  if (raw == "error") {
    level = FaultLevel::ERROR;
    return true;
  }
  if (raw == "critical") {
    level = FaultLevel::CRITICAL;
    return true;
  }
  if (raw == "normal") {
    level = FaultLevel::NORMAL;
    return true;
  }
  return false;
}

bool parse_action(const std::string & raw_action, ActionType & action)
{
  const std::string action_str = to_lower(raw_action);
  if (action_str == "nodemanager" || action_str == "node_manager" || action_str == "supervisor") {
    action = ActionType::SUPERVISOR;
    return true;
  }
  if (action_str == "safety_system") {
    action = ActionType::SAFETY_SYSTEM;
    return true;
  }
  if (action_str == "none") {
    action = ActionType::NONE;
    return true;
  }
  return false;
}

bool parse_safety_command(const YAML::Node & node, SafetyCommandType & command)
{
  if (!node || !node.IsScalar()) {
    return false;
  }

  try {
    const int raw_value = node.as<int>();
    switch (raw_value) {
      case 0:
        command = SafetyCommandType::NONE;
        return true;
      case 1:
        command = SafetyCommandType::SLOW_DOWN;
        return true;
      case 2:
        command = SafetyCommandType::SOFT_STOP;
        return true;
      case 3:
        command = SafetyCommandType::EMERGENCY_STOP;
        return true;
      default:
        return false;
    }
  } catch (...) {
  }

  const std::string raw = to_lower(node.as<std::string>());
  if (raw == "none" || raw == "0") {
    command = SafetyCommandType::NONE;
    return true;
  }
  if (raw == "slow_down" || raw == "slowdown" || raw == "1") {
    command = SafetyCommandType::SLOW_DOWN;
    return true;
  }
  if (raw == "soft_stop" || raw == "softstop" || raw == "2") {
    command = SafetyCommandType::SOFT_STOP;
    return true;
  }
  if (raw == "emergency_stop" || raw == "emergencystop" || raw == "3") {
    command = SafetyCommandType::EMERGENCY_STOP;
    return true;
  }

  return false;
}

std::vector<ActionType> parse_actions(
  const YAML::Node & actions_node, const std::string & scope, rclcpp::Logger logger)
{
  std::vector<ActionType> actions;
  if (!actions_node || !actions_node.IsSequence()) {
    return actions;
  }

  for (const auto & action_node : actions_node) {
    ActionType action = ActionType::NONE;
    const std::string action_str = action_node.as<std::string>();
    if (!parse_action(action_str, action)) {
      RCLCPP_ERROR(logger, "%s invalid action '%s'", scope.c_str(), action_str.c_str());
      continue;
    }
    if (std::find(actions.begin(), actions.end(), action) == actions.end()) {
      actions.push_back(action);
    }
  }
  return actions;
}

bool parse_bool(const YAML::Node & node, bool fallback)
{
  if (!node) {
    return fallback;
  }

  try {
    if (node.IsScalar()) {
      const std::string raw = to_lower(node.as<std::string>());
      if (raw == "1" || raw == "true" || raw == "yes" || raw == "on") {
        return true;
      }
      if (raw == "0" || raw == "false" || raw == "no" || raw == "off") {
        return false;
      }
    }
    return node.as<bool>();
  } catch (...) {
    return fallback;
  }
}

std::vector<CollisionPoint> make_forward_footprint_zone(
  const std::vector<CollisionPoint> & footprint,
  double forward_body_scale)
{
  if (footprint.size() < 3 || forward_body_scale <= 0.0) {
    return {};
  }

  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();

  for (const auto & point : footprint) {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }

  if (!std::isfinite(min_x) || !std::isfinite(max_x) || max_x <= min_x || max_y <= min_y) {
    return {};
  }

  const double body_length = max_x - min_x;
  const double front_x = max_x;
  const double forward_x = front_x + body_length * forward_body_scale;

  return {
    CollisionPoint{forward_x, max_y},
    CollisionPoint{forward_x, min_y},
    CollisionPoint{front_x, min_y},
    CollisionPoint{front_x, max_y}
  };
}

AutoFootprintZoneType parse_auto_footprint_zone_type(const YAML::Node & node)
{
  if (!node || !node.IsScalar()) {
    return AutoFootprintZoneType::NONE;
  }

  const std::string raw = to_lower(node.as<std::string>());
  if (raw == "front_slow" || raw == "slow") {
    return AutoFootprintZoneType::FRONT_SLOW;
  }
  if (raw == "front_stop" || raw == "stop") {
    return AutoFootprintZoneType::FRONT_STOP;
  }
  return AutoFootprintZoneType::NONE;
}

void apply_auto_footprint_zone_points(
  CollisionZoneConfig & zone,
  const CollisionDetectionConfig & cfg)
{
  if (!cfg.auto_footprint_zones_enabled ||
    zone.model != CollisionModelType::ZONE ||
    zone.auto_footprint_zone == AutoFootprintZoneType::NONE)
  {
    return;
  }

  double fast_scale = 0.0;
  double safe_scale = 0.0;
  if (zone.auto_footprint_zone == AutoFootprintZoneType::FRONT_SLOW) {
    fast_scale = 1.0;
    safe_scale = 0.5;
  } else if (zone.auto_footprint_zone == AutoFootprintZoneType::FRONT_STOP) {
    fast_scale = 0.5;
    safe_scale = 0.35;
  }

  zone.fast_points = make_forward_footprint_zone(cfg.footprint_points, fast_scale);
  zone.safe_points = make_forward_footprint_zone(cfg.footprint_points, safe_scale);
  zone.points = cfg.navigation_safe_mode_active && !zone.safe_points.empty() ?
    zone.safe_points : zone.fast_points;
}

void apply_auto_footprint_zones(CollisionDetectionConfig & cfg)
{
  for (auto & zone : cfg.zones) {
    apply_auto_footprint_zone_points(zone, cfg);
  }
}


std::vector<UltrasonicSensorConfig> make_default_ultrasonic_sensors()
{
  // 实际编号约定：1号=左前，之后顺时针。
  // 这里按 8 路环绕底盘的近似安装位姿内置默认布局：
  // 0 左前(45°), 1 正前(0°), 2 右前(-45°), 3 右侧(-90°),
  // 4 右后(-135°), 5 正后(180°), 6 左后(135°), 7 左侧(90°)
  return {
    UltrasonicSensorConfig{0, true, 0.18, 0.18, 45.0, 1.2, 0.9},
    UltrasonicSensorConfig{1, true, 0.24, 0.00, 0.0, 1.2, 1.0},
    UltrasonicSensorConfig{2, true, 0.18, -0.18, -45.0, 1.2, 0.9},
    UltrasonicSensorConfig{3, true, 0.00, -0.22, -90.0, 1.0, 0.55},
    UltrasonicSensorConfig{4, true, -0.18, -0.18, -135.0, 0.9, 0.25},
    UltrasonicSensorConfig{5, true, -0.24, 0.00, 180.0, 0.9, 0.2},
    UltrasonicSensorConfig{6, true, -0.18, 0.18, 135.0, 0.9, 0.25},
    UltrasonicSensorConfig{7, true, 0.00, 0.22, 90.0, 1.0, 0.55}
  };
}

void apply_ultrasonic_widget(
  const YAML::Node & widget_node,
  std::vector<UltrasonicSensorConfig> & sensors,
  rclcpp::Logger logger)
{
  if (!widget_node || !widget_node.IsSequence()) {
    return;
  }

  if (sensors.empty()) {
    sensors = make_default_ultrasonic_sensors();
  }

  if (widget_node.size() < sensors.size()) {
    RCLCPP_WARN(
      logger,
      "[collision_detection] ultrasonic_widget size=%zu < sensor_count=%zu, remaining sensors keep defaults",
      widget_node.size(), sensors.size());
  }

  const size_t count = std::min(widget_node.size(), sensors.size());
  for (size_t idx = 0; idx < count; ++idx) {
    try {
      sensors[idx].weight = std::clamp(widget_node[idx].as<double>(), 0.0, 1.0);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger, "[collision_detection] ultrasonic_widget[%zu] invalid: %s", idx, e.what());
    }
  }
}



}  // namespace

FaultDetector::FaultDetector(rclcpp::Node * node)
: node_(node), config_loaded_time_(node->now()), feedback_default_max_stale_s_(5.0),
  watch_topic_evaluator_(std::make_unique<WatchTopicEvaluator>()),
  feedback_rule_evaluator_(std::make_unique<FeedbackRuleEvaluator>()),
  chassis_evaluator_(std::make_unique<ChassisEvaluator>()),
  collision_evaluator_(std::make_unique<CollisionEvaluator>())
{
  chassis_cfg_.enabled = false;
  chassis_cfg_.module_name = "vehicle_state_judge";
  chassis_cfg_.command_topic = "/command";
  chassis_cfg_.moto_topic = "/moto_info";
  chassis_cfg_.odom_topic = "";
  chassis_cfg_.imu_topic = "";
  chassis_cfg_.source_timeout_s = 1.0;
  chassis_cfg_.idle_timeout_s = 30.0;
  chassis_cfg_.coast_grace_s = 1.0;
  chassis_cfg_.command_speed_threshold = 0.05;
  chassis_cfg_.moto_speed_threshold = 0.05;
  chassis_cfg_.odom_speed_threshold = 0.03;
  chassis_cfg_.imu_speed_threshold = 0.03;
  chassis_cfg_.imu_yaw_rate_threshold = 0.08;
  chassis_cfg_.imu_static_command_threshold = 0.02;
  chassis_cfg_.imu_bias_calibration_samples = 50;
  chassis_cfg_.imu_decay_rate = 0.98;
  chassis_cfg_.source_level = FaultLevel::ERROR;
  chassis_cfg_.anomaly_level = FaultLevel::ERROR;
  chassis_cfg_.idle_level = FaultLevel::WARNING;
  chassis_cfg_.safety_command = SafetyCommandType::SOFT_STOP;
  chassis_cfg_.safety_slow_down_percentage = 50.0;
  chassis_cfg_.source_actions = {ActionType::SUPERVISOR};
  chassis_cfg_.anomaly_actions = {ActionType::SUPERVISOR};
  chassis_cfg_.idle_actions = {ActionType::NONE};


  multi_value_cfg_.trigger_count = 2;
  multi_value_cfg_.recover_count = 2;
}

FaultDetector::~FaultDetector() = default;

void FaultDetector::set_feedback_default_max_stale(double default_max_stale_s)
{
  feedback_default_max_stale_s_ = std::max(0.0, default_max_stale_s);
}

bool FaultDetector::collision_detection_enabled() const
{
  return collision_cfg_.enabled;
}

const CollisionDetectionConfig & FaultDetector::get_collision_detection_config() const
{
  return collision_cfg_;
}

void FaultDetector::set_collision_navigation_safe_mode(bool safe_mode_active)
{
  if (collision_cfg_.navigation_safe_mode_active == safe_mode_active) {
    return;
  }

  collision_cfg_.navigation_safe_mode_active = safe_mode_active;
  apply_auto_footprint_zones(collision_cfg_);
}

const CollisionTtcVisualizationState & FaultDetector::get_collision_ttc_visualization() const
{
  static const CollisionTtcVisualizationState empty_state{};
  if (!collision_evaluator_) {
    return empty_state;
  }
  return collision_evaluator_->get_ttc_visualization();
}

bool FaultDetector::vehicle_state_judge_enabled() const
{
  return chassis_cfg_.enabled;
}

const ChassisStationaryConfig & FaultDetector::get_vehicle_state_judge_config() const
{
  return chassis_cfg_;
}

bool FaultDetector::chassis_stationary_enabled() const
{
  return vehicle_state_judge_enabled();
}

const ChassisStationaryConfig & FaultDetector::get_chassis_stationary_config() const
{
  return get_vehicle_state_judge_config();
}

bool FaultDetector::has_module_configs() const
{
  return !modules_.empty();
}

const std::vector<std::string> & FaultDetector::get_monitored_nodes() const
{
  return monitored_nodes_;
}

const std::vector<std::string> & FaultDetector::get_watched_topics() const
{
  return watched_topics_;
}

const std::vector<std::string> & FaultDetector::get_monitored_transforms() const
{
  return monitored_transforms_;
}

bool FaultDetector::is_watch_topic_frequency_required(const std::string & topic) const
{
  return get_watch_topic_min_hz(topic) > 0.0;
}

double FaultDetector::get_watch_topic_min_hz(const std::string & topic) const
{
  double min_hz = 0.0;
  for (const auto & module : modules_) {
    const auto it = module.watch_topic_min_hz.find(topic);
    if (it != module.watch_topic_min_hz.end()) {
      min_hz = std::max(min_hz, it->second);
    }
  }
  return min_hz;
}

const MultiValueJudgeConfig & FaultDetector::get_multi_value_judge_config() const
{
  return multi_value_cfg_;
}


void FaultDetector::load_config(const std::string & config_file)
{
  const auto prev_modules = modules_;
  const auto prev_monitored_nodes = monitored_nodes_;
  const auto prev_watched_topics = watched_topics_;
  const auto prev_monitored_transforms = monitored_transforms_;
  const auto prev_collision_cfg = collision_cfg_;
  const auto prev_chassis_cfg = chassis_cfg_;
  const auto prev_multi_value_cfg = multi_value_cfg_;
  const auto prev_combined_fault_rules = combined_fault_rules_;
  const auto prev_config_loaded_time = config_loaded_time_;

  auto restore_previous = [&]() {
    modules_ = prev_modules;
    monitored_nodes_ = prev_monitored_nodes;
    watched_topics_ = prev_watched_topics;
    monitored_transforms_ = prev_monitored_transforms;
    collision_cfg_ = prev_collision_cfg;
    chassis_cfg_ = prev_chassis_cfg;
    multi_value_cfg_ = prev_multi_value_cfg;
    combined_fault_rules_ = prev_combined_fault_rules;
    config_loaded_time_ = prev_config_loaded_time;
  };

  try {
    YAML::Node config = YAML::LoadFile(config_file);
    modules_.clear();
    monitored_nodes_.clear();
    watched_topics_.clear();
    monitored_transforms_.clear();
    config_loaded_time_ = node_->now();

    multi_value_cfg_.trigger_count = 2;
    multi_value_cfg_.recover_count = 2;
    combined_fault_rules_.clear();
    if (config["multi_value_judge"]) {
      const auto mv = config["multi_value_judge"];
      if (mv["trigger_count"]) {
        try {
          multi_value_cfg_.trigger_count = std::max(1, mv["trigger_count"].as<int>());
        } catch (...) {
          RCLCPP_ERROR(node_->get_logger(), "[multi_value_judge] invalid trigger_count, fallback to 2");
        }
      }
      if (mv["recover_count"]) {
        try {
          multi_value_cfg_.recover_count = std::max(1, mv["recover_count"].as<int>());
        } catch (...) {
          RCLCPP_ERROR(node_->get_logger(), "[multi_value_judge] invalid recover_count, fallback to 2");
        }
      }
    }

    if (config["combined_fault_rules"] && config["combined_fault_rules"].IsSequence()) {
      for (const auto & rule_node : config["combined_fault_rules"]) {
        try {
          CombinedFaultRuleConfig rule;
          if (!rule_node["name"] || !rule_node["name"].IsScalar()) {
            RCLCPP_ERROR(node_->get_logger(), "Skip combined fault rule: missing name");
            continue;
          }
          rule.name = rule_node["name"].as<std::string>();
          if (!rule_node["when_all"] || !rule_node["when_all"].IsSequence()) {
            RCLCPP_ERROR(
              node_->get_logger(),
              "[combined_fault_rules][%s] missing when_all sequence",
              rule.name.c_str());
            continue;
          }

          for (const auto & key_node : rule_node["when_all"]) {
            const auto fault_key = key_node.as<std::string>();
            if (!fault_key.empty()) {
              rule.when_all_fault_keys.push_back(fault_key);
            }
          }
          if (rule.when_all_fault_keys.empty()) {
            RCLCPP_ERROR(
              node_->get_logger(),
              "[combined_fault_rules][%s] when_all is empty",
              rule.name.c_str());
            continue;
          }

          if (!parse_fault_level(rule_node["level"], rule.level)) {
            rule.level = FaultLevel::ERROR;
          }
          rule.actions = parse_actions(
            rule_node["actions"],
            "[combined_fault_rules][" + rule.name + "]",
            node_->get_logger());
          SafetyCommandType safety_command = SafetyCommandType::NONE;
          if (parse_safety_command(rule_node["safety_system"], safety_command)) {
            rule.safety_command = safety_command;
          }
          if (rule_node["safety_slow_down_percentage"]) {
            rule.safety_slow_down_percentage = std::clamp(
              rule_node["safety_slow_down_percentage"].as<double>(), 0.0, 100.0);
          }
          if (rule_node["reason"] && rule_node["reason"].IsScalar()) {
            rule.reason = rule_node["reason"].as<std::string>();
          }

          combined_fault_rules_.push_back(std::move(rule));
        } catch (const std::exception & e) {
          RCLCPP_ERROR(node_->get_logger(), "Skip combined fault rule: %s", e.what());
        }
      }
    }

    if (config["target_transforms"] && config["target_transforms"].IsSequence()) {
      for (const auto & tf_node : config["target_transforms"]) {
        try {
          const std::string tf_str = tf_node.as<std::string>();
          if (tf_str.find("->") == std::string::npos) {
            RCLCPP_ERROR(node_->get_logger(), "Skip target_transform '%s': expected frame1->frame2", tf_str.c_str());
            continue;
          }
          monitored_transforms_.push_back(tf_str);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(node_->get_logger(), "Skip target_transform: invalid config (%s)", e.what());
        }
      }
    }

    if (config["collision_detection"]) {
      const auto cd = config["collision_detection"];
      try {
        if (cd["enabled"]) {
          if (cd["enabled"].IsScalar()) {
            const std::string raw = to_lower(cd["enabled"].as<std::string>());
            collision_cfg_.enabled = (raw == "1" || raw == "true" || raw == "yes");
          } else {
            collision_cfg_.enabled = cd["enabled"].as<bool>();
          }
        }
      } catch (...) {
        collision_cfg_.enabled = false;
      }
      if (cd["module_name"]) {
        collision_cfg_.module_name = cd["module_name"].as<std::string>();
      }
      if (cd["scan_topic"]) {
        collision_cfg_.scan_topic = cd["scan_topic"].as<std::string>();
      }
      if (cd["pointcloud_topic"]) {
        collision_cfg_.pointcloud_topic = cd["pointcloud_topic"].as<std::string>();
      }
      if (cd["voxel_topic"]) {
        collision_cfg_.voxel_topic = cd["voxel_topic"].as<std::string>();
      }
      if (cd["ultrasonic_topic"]) {
        collision_cfg_.ultrasonic_topic = cd["ultrasonic_topic"].as<std::string>();
      }
      if (cd["prediction_speed_topic"]) {
        collision_cfg_.prediction_speed_topic = cd["prediction_speed_topic"].as<std::string>();
      }
      if (cd["control_source_state_topic"]) {
        collision_cfg_.control_source_state_topic = cd["control_source_state_topic"].as<std::string>();
      }
      if (cd["prediction_speed_navigation_topic"]) {
        collision_cfg_.prediction_speed_navigation_topic =
          cd["prediction_speed_navigation_topic"].as<std::string>();
      }
      if (cd["prediction_speed_miniapp_topic"]) {
        collision_cfg_.prediction_speed_miniapp_topic =
          cd["prediction_speed_miniapp_topic"].as<std::string>();
      }
      if (cd["prediction_speed_remote_topic"]) {
        collision_cfg_.prediction_speed_remote_topic =
          cd["prediction_speed_remote_topic"].as<std::string>();
      }
      if (cd["prediction_speed_other_topic"]) {
        collision_cfg_.prediction_speed_other_topic =
          cd["prediction_speed_other_topic"].as<std::string>();
      }
      if (cd["ultrasonic_distances_key"]) {
        collision_cfg_.ultrasonic_distances_key = cd["ultrasonic_distances_key"].as<std::string>();
      }
      if (cd["ultrasonic_scene_flag_key"]) {
        collision_cfg_.ultrasonic_scene_flag_key = cd["ultrasonic_scene_flag_key"].as<std::string>();
      }
      if (cd["ttc_visualization_enabled"]) {
        try {
          if (cd["ttc_visualization_enabled"].IsScalar()) {
            const std::string raw = to_lower(cd["ttc_visualization_enabled"].as<std::string>());
            collision_cfg_.ttc_visualization_enabled = (raw == "1" || raw == "true" || raw == "yes");
          } else {
            collision_cfg_.ttc_visualization_enabled = cd["ttc_visualization_enabled"].as<bool>();
          }
        } catch (...) {
          collision_cfg_.ttc_visualization_enabled = false;
        }
      } else {
        collision_cfg_.ttc_visualization_enabled = false;
      }
      if (cd["ultrasonic_blind_distance"]) {
        collision_cfg_.ultrasonic_blind_distance = std::max(0.0, cd["ultrasonic_blind_distance"].as<double>());
      }
      if (cd["ultrasonic_out_of_range_value"]) {
        collision_cfg_.ultrasonic_out_of_range_value = std::max(
          collision_cfg_.ultrasonic_blind_distance,
          cd["ultrasonic_out_of_range_value"].as<double>());
      }
      if (cd["pointcloud_height_filter_enabled"]) {
        try {
          if (cd["pointcloud_height_filter_enabled"].IsScalar()) {
            const std::string raw =
              to_lower(cd["pointcloud_height_filter_enabled"].as<std::string>());
            collision_cfg_.pointcloud_height_filter_enabled =
              (raw == "1" || raw == "true" || raw == "yes");
          } else {
            collision_cfg_.pointcloud_height_filter_enabled =
              cd["pointcloud_height_filter_enabled"].as<bool>();
          }
        } catch (...) {
          collision_cfg_.pointcloud_height_filter_enabled = true;
        }
      }
      if (cd["pointcloud_min_height"]) {
        collision_cfg_.pointcloud_min_height = cd["pointcloud_min_height"].as<double>();
      }
      if (cd["pointcloud_max_height"]) {
        collision_cfg_.pointcloud_max_height = cd["pointcloud_max_height"].as<double>();
      }
      if (cd["source_timeout_s"]) {
        collision_cfg_.source_timeout_s = std::max(0.0, cd["source_timeout_s"].as<double>());
      }
      FaultLevel collision_source_level;
      if (parse_fault_level(cd["source_level"], collision_source_level)) {
        collision_cfg_.source_level = collision_source_level;
      }
      auto collision_source_actions = parse_actions(
        cd["source_actions"], "[collision_detection][source_actions]", node_->get_logger());
      if (!collision_source_actions.empty()) {
        collision_cfg_.source_actions = std::move(collision_source_actions);
      }
      if (cd["direction_speed_threshold"]) {
        collision_cfg_.direction_speed_threshold = std::max(
          0.0, cd["direction_speed_threshold"].as<double>());
      }
      if (cd["direction_confirm_count"]) {
        collision_cfg_.direction_confirm_count = std::max(
          size_t{1}, cd["direction_confirm_count"].as<size_t>());
      }
      if (cd["navigation_mode_switch_enabled"]) {
        collision_cfg_.navigation_mode_switch_enabled = parse_bool(
          cd["navigation_mode_switch_enabled"], false);
      }
      if (cd["navigation_mode_topic"]) {
        collision_cfg_.navigation_mode_topic = cd["navigation_mode_topic"].as<std::string>();
      }
      if (cd["navigation_fast_mode"]) {
        collision_cfg_.navigation_fast_mode = cd["navigation_fast_mode"].as<std::string>();
      }
      if (cd["navigation_safe_mode"]) {
        collision_cfg_.navigation_safe_mode = cd["navigation_safe_mode"].as<std::string>();
      }
      if (cd["navigation_safe_enter_duration_s"]) {
        collision_cfg_.navigation_safe_enter_duration_s =
          std::max(0.0, cd["navigation_safe_enter_duration_s"].as<double>());
      }
      if (cd["navigation_safe_clear_duration_s"]) {
        collision_cfg_.navigation_safe_clear_duration_s =
          std::max(0.0, cd["navigation_safe_clear_duration_s"].as<double>());
      }
      if (cd["navigation_safe_min_hold_s"]) {
        collision_cfg_.navigation_safe_min_hold_s =
          std::max(0.0, cd["navigation_safe_min_hold_s"].as<double>());
      }
      if (cd["navigation_mode_publish_cooldown_s"]) {
        collision_cfg_.navigation_mode_publish_cooldown_s =
          std::max(0.0, cd["navigation_mode_publish_cooldown_s"].as<double>());
      }
      if (cd["auto_footprint_zones_enabled"]) {
        collision_cfg_.auto_footprint_zones_enabled = parse_bool(
          cd["auto_footprint_zones_enabled"], false);
      }
      collision_cfg_.footprint_points.clear();
      if (cd["footprint_points"] && cd["footprint_points"].IsSequence()) {
        std::vector<double> footprint_values;
        for (const auto & point_node : cd["footprint_points"]) {
          footprint_values.push_back(point_node.as<double>());
        }
        if (footprint_values.size() >= 6 && footprint_values.size() % 2 == 0) {
          collision_cfg_.footprint_points.reserve(footprint_values.size() / 2);
          for (size_t idx = 0; idx < footprint_values.size(); idx += 2) {
            collision_cfg_.footprint_points.push_back(
              CollisionPoint{footprint_values[idx], footprint_values[idx + 1]});
          }
        }
      }
      collision_cfg_.ultrasonic_sensors = make_default_ultrasonic_sensors();
      if (cd["ultrasonic_sensors"] && cd["ultrasonic_sensors"].IsSequence()) {
        collision_cfg_.ultrasonic_sensors.clear();
        for (const auto & sensor_node : cd["ultrasonic_sensors"]) {
          try {
            UltrasonicSensorConfig sensor;
            if (sensor_node["index"]) {
              sensor.index = std::max(0, sensor_node["index"].as<int>());
            }
            if (sensor_node["enabled"]) {
              try {
                if (sensor_node["enabled"].IsScalar()) {
                  const std::string raw = to_lower(sensor_node["enabled"].as<std::string>());
                  sensor.enabled = (raw == "1" || raw == "true" || raw == "yes");
                } else {
                  sensor.enabled = sensor_node["enabled"].as<bool>();
                }
              } catch (...) {
                sensor.enabled = true;
              }
            }
            if (sensor_node["x"]) {
              sensor.x = sensor_node["x"].as<double>();
            }
            if (sensor_node["y"]) {
              sensor.y = sensor_node["y"].as<double>();
            }
            if (sensor_node["yaw_deg"]) {
              sensor.yaw_deg = sensor_node["yaw_deg"].as<double>();
            }
            if (sensor_node["max_distance"]) {
              sensor.max_distance = std::max(0.0, sensor_node["max_distance"].as<double>());
            }
            if (sensor_node["weight"]) {
              sensor.weight = std::max(0.0, sensor_node["weight"].as<double>());
            }
            collision_cfg_.ultrasonic_sensors.push_back(std::move(sensor));
          } catch (const std::exception & e) {
            RCLCPP_ERROR(node_->get_logger(), "Skip ultrasonic sensor config: %s", e.what());
          }
        }
      }
      apply_ultrasonic_widget(
        cd["ultrasonic_widget"], collision_cfg_.ultrasonic_sensors, node_->get_logger());
      collision_cfg_.zones.clear();
      if (cd["zones"] && cd["zones"].IsSequence()) {
        for (const auto & zone_node : cd["zones"]) {
          try {
            CollisionZoneConfig zone;
            zone.name = zone_node["name"].as<std::string>();
            if (zone_node["model"]) {
              const auto model = to_lower(zone_node["model"].as<std::string>());
              if (model == "ttc") {
                zone.model = CollisionModelType::TTC;
              } else if (model == "approach") {
                zone.model = CollisionModelType::TTC;
                RCLCPP_WARN(
                  node_->get_logger(),
                  "[collision_detection][%s] model=approach is deprecated; use model=ttc",
                  zone.name.c_str());
              }
            }
            if (zone_node["motion_direction"]) {
              const auto motion_direction = to_lower(zone_node["motion_direction"].as<std::string>());
              if (motion_direction == "forward") {
                zone.motion_direction = CollisionMotionDirectionType::FORWARD;
              } else if (motion_direction == "reverse") {
                zone.motion_direction = CollisionMotionDirectionType::REVERSE;
              } else {
                zone.motion_direction = CollisionMotionDirectionType::BOTH;
              }
            }
            zone.auto_footprint_zone = parse_auto_footprint_zone_type(
              zone_node["auto_footprint_zone"]);
            if (zone_node["enabled"]) {
              try {
                if (zone_node["enabled"].IsScalar()) {
                  const std::string raw = to_lower(zone_node["enabled"].as<std::string>());
                  zone.enabled = (raw == "1" || raw == "true" || raw == "yes");
                } else {
                  zone.enabled = zone_node["enabled"].as<bool>();
                }
              } catch (...) {
                zone.enabled = true;
              }
            }
            if (zone_node["min_points"]) {
              zone.min_points = std::max(0.1, zone_node["min_points"].as<double>());
            }
            if (!parse_fault_level(zone_node["level"], zone.level)) {
              zone.level = FaultLevel::ERROR;
            }
            if (zone_node["safety_system"]) {
              SafetyCommandType cmd;
              if (parse_safety_command(zone_node["safety_system"], cmd)) {
                zone.safety_command = cmd;
              }
            }
            if (zone_node["safety_slow_down_percentage"]) {
              zone.safety_slow_down_percentage = std::clamp(
                zone_node["safety_slow_down_percentage"].as<double>(), 0.0, 100.0);
            }
            if (zone_node["time_before_collision"]) {
              zone.time_before_collision = std::max(0.0, zone_node["time_before_collision"].as<double>());
            }
            if (zone_node["recover_time_before_collision"]) {
              zone.recover_time_before_collision = std::max(0.0, zone_node["recover_time_before_collision"].as<double>());
            }
            if (zone_node["min_hold_time_s"]) {
              zone.min_hold_time_s = std::max(0.0, zone_node["min_hold_time_s"].as<double>());
            }
            if (zone_node["ttc_horizon_s"]) {
              zone.ttc_horizon_s = std::max(0.0, zone_node["ttc_horizon_s"].as<double>());
            }
            if (zone_node["corridor_margin"]) {
              zone.corridor_margin = std::max(0.0, zone_node["corridor_margin"].as<double>());
            }
            if (zone_node["candidate_downsample_resolution"]) {
              zone.candidate_downsample_resolution = std::max(
                0.01, zone_node["candidate_downsample_resolution"].as<double>());
            }
            if (zone_node["simulation_time_step"]) {
              zone.simulation_time_step = std::max(0.01, zone_node["simulation_time_step"].as<double>());
            }
            zone.actions = parse_actions(
              zone_node["actions"],
              "[collision_detection][" + zone.name + "]", node_->get_logger());
            if (zone.actions.empty()) {
              zone.actions = {ActionType::SAFETY_SYSTEM};
            }
            if (zone_node["visualize"]) {
              try {
                if (zone_node["visualize"].IsScalar()) {
                  const std::string raw = to_lower(zone_node["visualize"].as<std::string>());
                  zone.visualize = (raw == "1" || raw == "true" || raw == "yes");
                } else {
                  zone.visualize = zone_node["visualize"].as<bool>();
                }
              } catch (...) {
                zone.visualize = true;
              }
            }
            if (zone_node["polygon_pub_topic"]) {
              zone.polygon_pub_topic = zone_node["polygon_pub_topic"].as<std::string>();
            }
            if (zone.polygon_pub_topic.empty()) {
              zone.polygon_pub_topic = "/nav2_monitor/collision_zone/" + zone.name;
            }
            if (zone_node["points"] && zone_node["points"].IsSequence()) {
              std::vector<double> points;
              for (const auto & point_node : zone_node["points"]) {
                points.push_back(point_node.as<double>());
              }
              if (points.size() >= 6 && points.size() % 2 == 0) {
                zone.points.reserve(points.size() / 2);
                for (size_t idx = 0; idx < points.size(); idx += 2) {
                  zone.points.push_back(CollisionPoint{points[idx], points[idx + 1]});
                }
              }
            }
            apply_auto_footprint_zone_points(zone, collision_cfg_);
            if (zone.model == CollisionModelType::TTC || zone.points.size() >= 3) {
              collision_cfg_.zones.push_back(std::move(zone));
            }
          } catch (const std::exception & e) {
            RCLCPP_ERROR(node_->get_logger(), "Skip collision zone: invalid config (%s)", e.what());
          }
        }
      }
    }

    const bool has_vehicle_state_judge = static_cast<bool>(config["vehicle_state_judge"]);
    const bool has_legacy_chassis_stationary = static_cast<bool>(config["chassis_stationary"]);
    if (has_vehicle_state_judge || has_legacy_chassis_stationary) {
      const auto cs = has_vehicle_state_judge ?
        config["vehicle_state_judge"] : config["chassis_stationary"];
      const std::string config_label = has_vehicle_state_judge ?
        "vehicle_state_judge" : "chassis_stationary";
      try {
        if (cs["enabled"]) {
          if (cs["enabled"].IsScalar()) {
            const std::string raw = to_lower(cs["enabled"].as<std::string>());
            chassis_cfg_.enabled = (raw == "1" || raw == "true" || raw == "yes");
          } else {
            chassis_cfg_.enabled = cs["enabled"].as<bool>();
          }
        }
      } catch (...) {
        chassis_cfg_.enabled = false;
      }

      if (cs["module_name"]) {
        chassis_cfg_.module_name = cs["module_name"].as<std::string>();
      }
      if (cs["command_topic"]) {
        chassis_cfg_.command_topic = cs["command_topic"].as<std::string>();
      }
      if (cs["moto_topic"]) {
        chassis_cfg_.moto_topic = cs["moto_topic"].as<std::string>();
      }
      if (cs["odom_topic"]) {
        chassis_cfg_.odom_topic = cs["odom_topic"].as<std::string>();
      }
      if (cs["imu_topic"]) {
        chassis_cfg_.imu_topic = cs["imu_topic"].as<std::string>();
      }
      if (cs["source_timeout_s"]) {
        chassis_cfg_.source_timeout_s = std::max(0.0, cs["source_timeout_s"].as<double>());
      }
      if (cs["idle_timeout_s"]) {
        chassis_cfg_.idle_timeout_s = std::max(0.0, cs["idle_timeout_s"].as<double>());
      }
      if (cs["coast_grace_s"]) {
        chassis_cfg_.coast_grace_s = std::max(0.0, cs["coast_grace_s"].as<double>());
      }
      if (cs["command_speed_threshold"]) {
        chassis_cfg_.command_speed_threshold = std::max(0.0, cs["command_speed_threshold"].as<double>());
      }
      if (cs["moto_speed_threshold"]) {
        chassis_cfg_.moto_speed_threshold = std::max(0.0, cs["moto_speed_threshold"].as<double>());
      }
      if (cs["odom_speed_threshold"]) {
        chassis_cfg_.odom_speed_threshold = std::max(0.0, cs["odom_speed_threshold"].as<double>());
      }
      if (cs["imu_speed_threshold"]) {
        chassis_cfg_.imu_speed_threshold = std::max(0.0, cs["imu_speed_threshold"].as<double>());
      }
      if (cs["imu_yaw_rate_threshold"]) {
        chassis_cfg_.imu_yaw_rate_threshold =
          std::max(0.0, cs["imu_yaw_rate_threshold"].as<double>());
      }
      if (cs["imu_static_command_threshold"]) {
        chassis_cfg_.imu_static_command_threshold =
          std::max(0.0, cs["imu_static_command_threshold"].as<double>());
      }
      if (cs["imu_bias_calibration_samples"]) {
        chassis_cfg_.imu_bias_calibration_samples =
          std::max(1, cs["imu_bias_calibration_samples"].as<int>());
      }
      if (cs["imu_decay_rate"]) {
        chassis_cfg_.imu_decay_rate =
          std::clamp(cs["imu_decay_rate"].as<double>(), 0.0, 1.0);
      }

      FaultLevel source_level;
      if (parse_fault_level(cs["source_level"], source_level)) {
        chassis_cfg_.source_level = source_level;
      }
      FaultLevel anomaly_level;
      if (parse_fault_level(cs["anomaly_level"], anomaly_level)) {
        chassis_cfg_.anomaly_level = anomaly_level;
      }
      FaultLevel idle_level;
      if (parse_fault_level(cs["idle_level"], idle_level)) {
        chassis_cfg_.idle_level = idle_level;
      }

      SafetyCommandType safety_command;
      if (parse_safety_command(cs["safety_system"], safety_command)) {
        chassis_cfg_.safety_command = safety_command;
      }
      if (cs["safety_slow_down_percentage"]) {
        chassis_cfg_.safety_slow_down_percentage =
          std::clamp(cs["safety_slow_down_percentage"].as<double>(), 0.0, 100.0);
      }

      auto source_actions = parse_actions(
        cs["source_actions"], "[" + config_label + "][source_actions]", node_->get_logger());
      if (!source_actions.empty()) {
        chassis_cfg_.source_actions = std::move(source_actions);
      }

      auto anomaly_actions = parse_actions(
        cs["anomaly_actions"], "[" + config_label + "][anomaly_actions]", node_->get_logger());
      if (!anomaly_actions.empty()) {
        chassis_cfg_.anomaly_actions = std::move(anomaly_actions);
      }

      auto idle_actions = parse_actions(
        cs["idle_actions"], "[" + config_label + "][idle_actions]", node_->get_logger());
      if (!idle_actions.empty()) {
        chassis_cfg_.idle_actions = std::move(idle_actions);
      }
    }

    if (!config["modules"] || !config["modules"].IsSequence()) {
      RCLCPP_ERROR(node_->get_logger(), "Invalid config: missing 'modules' sequence");
      restore_previous();
      return;
    }

    std::set<std::string> seen_nodes;
    std::set<std::string> seen_topics;

    for (const auto & mod : config["modules"]) {
      ModuleConfig mc;
      try {
        mc.name = mod["name"].as<std::string>();
        if (mod["nodemanager"]) {
          mc.enable_supervisor = mod["nodemanager"].as<int>() == 1;
        } else {
          mc.enable_supervisor = mod["supervisor"].as<int>() == 1;
        }
        mc.safety_command = SafetyCommandType::NONE;
        mc.safety_slow_down_percentage = 50.0;
      } catch (const std::exception & e) {
        RCLCPP_ERROR(node_->get_logger(), "Skip module: invalid base config (%s)", e.what());
        continue;
      }

      if (mod["safety_system"]) {
        SafetyCommandType safety_command;
        if (!parse_safety_command(mod["safety_system"], safety_command)) {
          RCLCPP_ERROR(
            node_->get_logger(), "Skip module '%s': invalid safety_system, expected 0-3",
            mc.name.c_str());
          continue;
        }
        mc.safety_command = safety_command;
      }
      if (mod["safety_slow_down_percentage"]) {
        mc.safety_slow_down_percentage =
          std::clamp(mod["safety_slow_down_percentage"].as<double>(), 0.0, 100.0);
      }

      if (mod["nodes"]) {
        for (const auto & n : mod["nodes"]) {
          const auto node_name = n.as<std::string>();
          mc.nodes.push_back(node_name);
          if (seen_nodes.insert(node_name).second) {
            monitored_nodes_.push_back(node_name);
          }
        }
      }

      if (mod["topics"]) {
        RCLCPP_ERROR(
          node_->get_logger(), "Module '%s' uses deprecated field 'topics'; use 'watch_topics'",
          mc.name.c_str());
      }

      if (mod["watch_topics"]) {
        for (const auto & t : mod["watch_topics"]) {
          const std::string topic = t["name"].as<std::string>();
          const double min_hz = t["min_hz"] ? std::max(0.0, t["min_hz"].as<double>()) : -1.0;
          mc.watch_topic_min_hz[topic] = min_hz;
          if (seen_topics.insert(topic).second) {
            watched_topics_.push_back(topic);
          }
        }
      }

      if (mod["feedback_topics"]) {
        RCLCPP_ERROR(
          node_->get_logger(), "Module '%s' uses deprecated field 'feedback_topics'; use 'feedback_rules'",
          mc.name.c_str());
      }

      if (mod["feedback_rules"] && mod["feedback_rules"].IsSequence()) {
        for (const auto & rule_node : mod["feedback_rules"]) {
          try {
            ModuleConfig::FeedbackRule rule;
            if (rule_node["topic_name"]) {
              RCLCPP_ERROR(
                node_->get_logger(), "[%s] deprecated field 'topic_name' in feedback rule; use 'source_topic'",
                mc.name.c_str());
            }

            if (!rule_node["source_topic"] || !rule_node["metric_name"]) {
              RCLCPP_ERROR(
                node_->get_logger(), "[%s] skip feedback rule: missing source_topic/metric_name", mc.name.c_str());
              continue;
            }

            rule.source_topic = rule_node["source_topic"].as<std::string>();
            rule.metric_name = rule_node["metric_name"].as<std::string>();
            rule.has_min_value = static_cast<bool>(rule_node["min_value"]);
            rule.has_max_value = static_cast<bool>(rule_node["max_value"]);
            rule.min_value = rule.has_min_value ? rule_node["min_value"].as<double>() : 0.0;
            rule.max_value = rule.has_max_value ? rule_node["max_value"].as<double>() : 0.0;
            rule.min_hz = rule_node["min_hz"] ? std::max(0.0, rule_node["min_hz"].as<double>()) : 0.0;
            rule.max_stale_s = rule_node["max_stale_s"] ?
              std::max(0.0, rule_node["max_stale_s"].as<double>()) : feedback_default_max_stale_s_;
            rule.has_safety_command_override = false;
            rule.safety_command = SafetyCommandType::NONE;
            rule.has_safety_slow_down_percentage = false;
            rule.safety_slow_down_percentage = 50.0;

            if (rule_node["safety_system"]) {
              SafetyCommandType safety_command;
              if (!parse_safety_command(rule_node["safety_system"], safety_command)) {
                RCLCPP_ERROR(
                  node_->get_logger(), "[%s][%s/%s] skip feedback rule: invalid safety_system, expected 0-3",
                  mc.name.c_str(), rule.source_topic.c_str(), rule.metric_name.c_str());
                continue;
              }
              rule.has_safety_command_override = true;
              rule.safety_command = safety_command;
            }

            if (rule_node["safety_slow_down_percentage"]) {
              rule.has_safety_slow_down_percentage = true;
              rule.safety_slow_down_percentage = std::clamp(
                rule_node["safety_slow_down_percentage"].as<double>(), 0.0, 100.0);
            }

            if (!parse_fault_level(rule_node["level"], rule.level)) {
              RCLCPP_ERROR(
                node_->get_logger(), "[%s][%s/%s] skip feedback rule: invalid level",
                mc.name.c_str(), rule.source_topic.c_str(), rule.metric_name.c_str());
              continue;
            }

            if (!rule.has_min_value && !rule.has_max_value) {
              RCLCPP_ERROR(
                node_->get_logger(), "[%s][%s/%s] skip feedback rule: min/max both missing",
                mc.name.c_str(), rule.source_topic.c_str(), rule.metric_name.c_str());
              continue;
            }

            if (rule.has_min_value && rule.has_max_value && rule.min_value > rule.max_value) {
              RCLCPP_ERROR(
                node_->get_logger(), "[%s][%s/%s] skip feedback rule: min_value > max_value",
                mc.name.c_str(), rule.source_topic.c_str(), rule.metric_name.c_str());
              continue;
            }

            auto actions = parse_actions(
              rule_node["actions"],
              "[" + mc.name + "][" + rule.source_topic + "/" + rule.metric_name + "]", node_->get_logger());
            if (actions.empty()) {
              RCLCPP_ERROR(
                node_->get_logger(), "[%s][%s/%s] skip feedback rule: no valid actions",
                mc.name.c_str(), rule.source_topic.c_str(), rule.metric_name.c_str());
              continue;
            }

            rule.actions = std::move(actions);
            mc.feedback_rules.push_back(std::move(rule));
          } catch (const std::exception & e) {
            RCLCPP_ERROR(
              node_->get_logger(), "[%s] skip feedback rule: invalid config (%s)", mc.name.c_str(), e.what());
          }
        }
      }

      modules_.push_back(std::move(mc));
    }

    if (modules_.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Invalid config: no valid modules after parsing");
      restore_previous();
      return;
    }

  watch_topic_evaluator_->set_multi_value_config(multi_value_cfg_);
  watch_topic_evaluator_->reset();
  feedback_rule_evaluator_->set_multi_value_config(multi_value_cfg_);
  feedback_rule_evaluator_->reset();
  chassis_evaluator_->set_logger(node_->get_logger());
  chassis_evaluator_->set_multi_value_config(multi_value_cfg_);
  chassis_evaluator_->reset();
  collision_evaluator_->set_multi_value_config(multi_value_cfg_);
    collision_evaluator_->reset();

    RCLCPP_INFO(
      node_->get_logger(),
      "Loaded %zu modules, monitored_nodes=%zu, monitored_topics=%zu, multi_value=%zu/%zu, vehicle_state_judge=%s",
      modules_.size(), monitored_nodes_.size(), watched_topics_.size(),
      multi_value_cfg_.trigger_count, multi_value_cfg_.recover_count,
      chassis_cfg_.enabled ? "enabled" : "disabled");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load config: %s", e.what());
    restore_previous();
  }
}

void FaultDetector::update_node_status(const std::map<std::string, bool> & node_active)
{
  const auto now = node_->now();
  for (const auto & [node, active] : node_active) {
    compatibility_store_.set_node_active(node, active, now);
  }
}

void FaultDetector::update_topic_freq(const std::map<std::string, double> & topic_freq)
{
  for (const auto & [topic, frequency] : topic_freq) {
    compatibility_store_.set_watch_topic_frequency(topic, frequency);
  }
}

void FaultDetector::update_feedback_sample(
  const std::string & module_name, const std::string & topic_name, const std::string & metric_name,
  double value, bool valid, const rclcpp::Time & stamp)
{
  compatibility_store_.add_feedback_sample(
    module_name, topic_name, metric_name, value, valid, stamp, node_->now());
}

void FaultDetector::update_command_speed(double speed, const rclcpp::Time & stamp)
{
  compatibility_store_.set_command_speed(speed, stamp);
}

void FaultDetector::update_moto_speed(
  double left_speed_rad, double right_speed_rad, const rclcpp::Time & stamp, bool valid)
{
  compatibility_store_.set_moto_speed(left_speed_rad, right_speed_rad, valid, stamp);
}

void FaultDetector::update_odom_speed(double linear_speed, const rclcpp::Time & stamp)
{
  compatibility_store_.set_odom_speed(linear_speed, stamp);
}

bool FaultDetector::check_module_nodes(
  const ModuleConfig & module, const MonitorDataStore & store, const rclcpp::Time & now) const
{
  for (const auto & node : module.nodes) {
    if (!store.is_node_active(node, now, feedback_default_max_stale_s_)) {
      return false;
    }
  }
  return true;
}

std::vector<FaultInfo> FaultDetector::detect_faults()
{
  return detect_faults(compatibility_store_, node_->now());
}

std::vector<FaultInfo> FaultDetector::detect_faults(
  const MonitorDataStore & store, const rclcpp::Time & now)
{
  std::vector<FaultInfo> faults;

  for (const auto & module : modules_) {
    FaultInfo base_fault;
    const std::string node_fault_key_prefix = module.name + "|node_inactive";
    base_fault.module_name = module.name;
    base_fault.timestamp = now;
      base_fault.level = FaultLevel::NORMAL;
      base_fault.fault_type = "node_inactive";
      base_fault.fault_model = "node";
      base_fault.fault_name = module.name;
      base_fault.action = ActionType::NONE;
    base_fault.safety_command = SafetyCommandType::NONE;
    base_fault.safety_slow_down_percentage = 0.0;

    if (!check_module_nodes(module, store, now)) {
      base_fault.level = FaultLevel::CRITICAL;
      base_fault.reason = "Node inactive";
      const bool has_action =
        module.enable_supervisor || module.safety_command != SafetyCommandType::NONE;
      if (module.safety_command != SafetyCommandType::NONE) {
        FaultInfo safety_fault = base_fault;
        safety_fault.fault_key = node_fault_key_prefix + "|action=" +
          std::to_string(static_cast<int>(ActionType::SAFETY_SYSTEM));
        safety_fault.action = ActionType::SAFETY_SYSTEM;
        safety_fault.safety_command = module.safety_command;
        safety_fault.safety_slow_down_percentage = module.safety_slow_down_percentage;
        faults.push_back(std::move(safety_fault));
      }
      if (module.enable_supervisor) {
        FaultInfo supervisor_fault = base_fault;
        supervisor_fault.fault_key = node_fault_key_prefix + "|action=" +
          std::to_string(static_cast<int>(ActionType::SUPERVISOR));
        supervisor_fault.action = ActionType::SUPERVISOR;
        faults.push_back(std::move(supervisor_fault));
      }
      if (!has_action) {
        base_fault.fault_key = node_fault_key_prefix + "|action=0";
        faults.push_back(std::move(base_fault));
      }
      continue;
    }

    auto feedback_faults = feedback_rule_evaluator_->evaluate(
      module, store, now, config_loaded_time_);
    faults.insert(
      faults.end(),
      std::make_move_iterator(feedback_faults.begin()),
      std::make_move_iterator(feedback_faults.end()));

    auto watch_faults = watch_topic_evaluator_->evaluate(module, store, now);
    faults.insert(
      faults.end(),
      std::make_move_iterator(watch_faults.begin()),
      std::make_move_iterator(watch_faults.end()));
  }

  if (chassis_cfg_.enabled) {
    auto chassis_faults = chassis_evaluator_->evaluate(chassis_cfg_, store, now);
    faults.insert(
      faults.end(),
      std::make_move_iterator(chassis_faults.begin()),
      std::make_move_iterator(chassis_faults.end()));
  }

  if (collision_cfg_.enabled) {
    auto collision_faults = collision_evaluator_->evaluate(collision_cfg_, store, now);
    faults.insert(
      faults.end(),
      std::make_move_iterator(collision_faults.begin()),
      std::make_move_iterator(collision_faults.end()));
  }

  const auto base_faults = faults;
  append_combined_faults(base_faults, now, faults);

  return faults;
}

std::vector<FaultInfo> FaultDetector::detect_feedback_faults(
  const MonitorDataStore & store,
  const rclcpp::Time & now)
{
  std::vector<FaultInfo> faults;
  for (const auto & module : modules_) {
    auto feedback_faults = feedback_rule_evaluator_->evaluate(
      module, store, now, config_loaded_time_);
    faults.insert(
      faults.end(),
      std::make_move_iterator(feedback_faults.begin()),
      std::make_move_iterator(feedback_faults.end()));
  }
  return faults;
}

std::vector<FaultInfo> FaultDetector::detect_collision_faults(
  const MonitorDataStore & store,
  const rclcpp::Time & now)
{
  if (!collision_cfg_.enabled) {
    return {};
  }
  return collision_evaluator_->evaluate(collision_cfg_, store, now);
}

void FaultDetector::append_combined_faults(
  const std::vector<FaultInfo> & base_faults,
  const rclcpp::Time & now,
  std::vector<FaultInfo> & faults) const
{
  if (combined_fault_rules_.empty()) {
    return;
  }

  std::set<std::string> active_fault_keys;
  for (const auto & fault : base_faults) {
    active_fault_keys.insert(fault.fault_key);
  }

  for (const auto & rule : combined_fault_rules_) {
    const bool matched = std::all_of(
      rule.when_all_fault_keys.begin(), rule.when_all_fault_keys.end(),
      [&active_fault_keys](const std::string & fault_key) {
        return active_fault_keys.count(fault_key) > 0;
      });
    if (!matched) {
      continue;
    }

    const std::string reason =
      rule.reason.empty() ? ("Combined fault triggered: " + rule.name) : rule.reason;
    if (rule.actions.empty()) {
      FaultInfo fault;
      fault.fault_key = "combined_fault|" + rule.name + "|action=0";
      fault.module_name = "combined_fault";
      fault.level = rule.level;
      fault.reason = reason;
      fault.fault_type = "combined_fault";
      fault.fault_model = "combined";
      fault.fault_name = rule.name;
      fault.action = ActionType::NONE;
      fault.safety_command = SafetyCommandType::NONE;
      fault.safety_slow_down_percentage = 0.0;
      fault.timestamp = now;
      faults.push_back(std::move(fault));
      continue;
    }

    for (const auto & action : rule.actions) {
      if (action == ActionType::SAFETY_SYSTEM && rule.safety_command == SafetyCommandType::NONE) {
        continue;
      }

      FaultInfo fault;
      fault.fault_key = "combined_fault|" + rule.name + "|action=" +
        std::to_string(static_cast<int>(action));
      fault.module_name = "combined_fault";
      fault.level = rule.level;
      fault.reason = reason;
      fault.fault_type = "combined_fault";
      fault.fault_model = "combined";
      fault.fault_name = rule.name;
      fault.action = action;
      fault.safety_command =
        action == ActionType::SAFETY_SYSTEM ? rule.safety_command : SafetyCommandType::NONE;
      fault.safety_slow_down_percentage =
        action == ActionType::SAFETY_SYSTEM ? rule.safety_slow_down_percentage : 0.0;
      fault.timestamp = now;
      faults.push_back(std::move(fault));
    }
  }
}


}  // namespace nav2_monitor
