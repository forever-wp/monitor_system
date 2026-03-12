#include "nav2_monitor/fault_detector.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
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
  if (action_str == "supervisor") {
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

double calc_frequency(const std::deque<rclcpp::Time> & msg_times)
{
  if (msg_times.size() < 2) {
    return 0.0;
  }
  const double span = (msg_times.back() - msg_times.front()).seconds();
  if (span <= 0.0) {
    return 0.0;
  }
  return static_cast<double>(msg_times.size() - 1) / span;
}

}  // namespace

FaultDetector::FaultDetector(rclcpp::Node * node)
: node_(node), config_loaded_time_(node->now()), feedback_default_max_stale_s_(5.0)
{
  chassis_cfg_.enabled = false;
  chassis_cfg_.module_name = "chassis_stationary";
  chassis_cfg_.command_topic = "/command";
  chassis_cfg_.moto_topic = "/moto_info";
  chassis_cfg_.odom_topic = "/odom";
  chassis_cfg_.source_timeout_s = 1.0;
  chassis_cfg_.idle_timeout_s = 30.0;
  chassis_cfg_.command_speed_threshold = 0.05;
  chassis_cfg_.moto_speed_threshold = 0.05;
  chassis_cfg_.odom_speed_threshold = 0.03;
  chassis_cfg_.anomaly_level = FaultLevel::ERROR;
  chassis_cfg_.idle_level = FaultLevel::WARNING;
  chassis_cfg_.safety_command = SafetyCommandType::SOFT_STOP;
  chassis_cfg_.safety_slow_down_percentage = 50.0;
  chassis_cfg_.anomaly_actions = {ActionType::SUPERVISOR};
  chassis_cfg_.idle_actions = {ActionType::NONE};

  chassis_state_.command_received = false;
  chassis_state_.command_speed = 0.0;
  chassis_state_.moto_received = false;
  chassis_state_.moto_valid = false;
  chassis_state_.left_speed_rad = 0.0;
  chassis_state_.right_speed_rad = 0.0;
  chassis_state_.odom_received = false;
  chassis_state_.odom_speed = 0.0;
  chassis_state_.idle_tracking = false;

  multi_value_cfg_.trigger_count = 2;
  multi_value_cfg_.recover_count = 2;
}

void FaultDetector::set_feedback_default_max_stale(double default_max_stale_s)
{
  feedback_default_max_stale_s_ = std::max(0.0, default_max_stale_s);
}

bool FaultDetector::chassis_stationary_enabled() const
{
  return chassis_cfg_.enabled;
}

const ChassisStationaryConfig & FaultDetector::get_chassis_stationary_config() const
{
  return chassis_cfg_;
}

bool FaultDetector::has_module_configs() const
{
  return !modules_.empty();
}

const std::vector<std::string> & FaultDetector::get_monitored_nodes() const
{
  return monitored_nodes_;
}

const std::vector<std::string> & FaultDetector::get_monitored_topics() const
{
  return monitored_topics_;
}

const MultiValueJudgeConfig & FaultDetector::get_multi_value_judge_config() const
{
  return multi_value_cfg_;
}

std::string FaultDetector::feedback_key(
  const std::string & module_name, const std::string & topic_name, const std::string & metric_name) const
{
  return module_name + "|feedback:" + topic_name + ":" + metric_name;
}

void FaultDetector::load_config(const std::string & config_file)
{
  const auto prev_modules = modules_;
  const auto prev_monitored_nodes = monitored_nodes_;
  const auto prev_monitored_topics = monitored_topics_;
  const auto prev_feedback_state = feedback_state_;
  const auto prev_feedback_judge_state = feedback_judge_state_;
  const auto prev_topic_judge_state = topic_judge_state_;
  const auto prev_chassis_judge_state = chassis_judge_state_;
  const auto prev_chassis_cfg = chassis_cfg_;
  const auto prev_multi_value_cfg = multi_value_cfg_;
  const auto prev_chassis_state = chassis_state_;
  const auto prev_config_loaded_time = config_loaded_time_;

  auto restore_previous = [&]() {
    modules_ = prev_modules;
    monitored_nodes_ = prev_monitored_nodes;
    monitored_topics_ = prev_monitored_topics;
    feedback_state_ = prev_feedback_state;
    feedback_judge_state_ = prev_feedback_judge_state;
    topic_judge_state_ = prev_topic_judge_state;
    chassis_judge_state_ = prev_chassis_judge_state;
    chassis_cfg_ = prev_chassis_cfg;
    multi_value_cfg_ = prev_multi_value_cfg;
    chassis_state_ = prev_chassis_state;
    config_loaded_time_ = prev_config_loaded_time;
  };

  try {
    YAML::Node config = YAML::LoadFile(config_file);
    modules_.clear();
    monitored_nodes_.clear();
    monitored_topics_.clear();
    feedback_state_.clear();
    feedback_judge_state_.clear();
    topic_judge_state_.clear();
    chassis_judge_state_.clear();
    config_loaded_time_ = node_->now();
    chassis_state_.idle_tracking = false;

    multi_value_cfg_.trigger_count = 2;
    multi_value_cfg_.recover_count = 2;
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

    if (config["chassis_stationary"]) {
      const auto cs = config["chassis_stationary"];
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
      if (cs["source_timeout_s"]) {
        chassis_cfg_.source_timeout_s = std::max(0.0, cs["source_timeout_s"].as<double>());
      }
      if (cs["idle_timeout_s"]) {
        chassis_cfg_.idle_timeout_s = std::max(0.0, cs["idle_timeout_s"].as<double>());
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

      auto anomaly_actions = parse_actions(
        cs["anomaly_actions"], "[chassis_stationary][anomaly_actions]", node_->get_logger());
      if (!anomaly_actions.empty()) {
        chassis_cfg_.anomaly_actions = std::move(anomaly_actions);
      }

      auto idle_actions = parse_actions(
        cs["idle_actions"], "[chassis_stationary][idle_actions]", node_->get_logger());
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
        mc.enable_supervisor = mod["supervisor"].as<int>() == 1;
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
        for (const auto & t : mod["topics"]) {
          const std::string topic = t["name"].as<std::string>();
          const double min_hz = t["min_hz"].as<double>();
          mc.topic_min_hz[topic] = min_hz;
          if (seen_topics.insert(topic).second) {
            monitored_topics_.push_back(topic);
          }
        }
      }

      if (mod["feedback_topics"] && mod["feedback_topics"].IsSequence()) {
        for (const auto & rule_node : mod["feedback_topics"]) {
          try {
            ModuleConfig::FeedbackRule rule;
            if (!rule_node["topic_name"] || !rule_node["metric_name"]) {
              RCLCPP_ERROR(
                node_->get_logger(), "[%s] skip feedback rule: missing topic_name/metric_name", mc.name.c_str());
              continue;
            }

            rule.topic_name = rule_node["topic_name"].as<std::string>();
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
                  mc.name.c_str(), rule.topic_name.c_str(), rule.metric_name.c_str());
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
                mc.name.c_str(), rule.topic_name.c_str(), rule.metric_name.c_str());
              continue;
            }

            if (!rule.has_min_value && !rule.has_max_value) {
              RCLCPP_ERROR(
                node_->get_logger(), "[%s][%s/%s] skip feedback rule: min/max both missing",
                mc.name.c_str(), rule.topic_name.c_str(), rule.metric_name.c_str());
              continue;
            }

            if (rule.has_min_value && rule.has_max_value && rule.min_value > rule.max_value) {
              RCLCPP_ERROR(
                node_->get_logger(), "[%s][%s/%s] skip feedback rule: min_value > max_value",
                mc.name.c_str(), rule.topic_name.c_str(), rule.metric_name.c_str());
              continue;
            }

            auto actions = parse_actions(
              rule_node["actions"],
              "[" + mc.name + "][" + rule.topic_name + "/" + rule.metric_name + "]", node_->get_logger());
            if (actions.empty()) {
              RCLCPP_ERROR(
                node_->get_logger(), "[%s][%s/%s] skip feedback rule: no valid actions",
                mc.name.c_str(), rule.topic_name.c_str(), rule.metric_name.c_str());
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

    RCLCPP_INFO(
      node_->get_logger(),
      "Loaded %zu modules, monitored_nodes=%zu, monitored_topics=%zu, multi_value=%zu/%zu, chassis_stationary=%s",
      modules_.size(), monitored_nodes_.size(), monitored_topics_.size(),
      multi_value_cfg_.trigger_count, multi_value_cfg_.recover_count,
      chassis_cfg_.enabled ? "enabled" : "disabled");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load config: %s", e.what());
    restore_previous();
  }
}

void FaultDetector::update_node_status(const std::map<std::string, bool> & node_active)
{
  node_status_ = node_active;
}

void FaultDetector::update_topic_freq(const std::map<std::string, double> & topic_freq)
{
  topic_freq_ = topic_freq;
}

void FaultDetector::update_feedback_sample(
  const std::string & module_name, const std::string & topic_name, const std::string & metric_name,
  double value, bool valid, const rclcpp::Time & stamp)
{
  const std::string key = feedback_key(module_name, topic_name, metric_name);
  auto & state = feedback_state_[key];
  if (!state.received) {
    state.first_seen = stamp;
  }
  state.received = true;
  state.last_value = value;
  state.last_valid = valid;
  state.last_seen = stamp;
  state.msg_times.push_back(stamp);
  if (state.msg_times.size() > kFeedbackWindowSize) {
    state.msg_times.pop_front();
  }
}

void FaultDetector::update_command_speed(double speed, const rclcpp::Time & stamp)
{
  chassis_state_.command_received = true;
  chassis_state_.command_speed = speed;
  chassis_state_.command_stamp = stamp;
}

void FaultDetector::update_moto_speed(
  double left_speed_rad, double right_speed_rad, const rclcpp::Time & stamp, bool valid)
{
  chassis_state_.moto_received = true;
  chassis_state_.moto_valid = valid;
  chassis_state_.left_speed_rad = left_speed_rad;
  chassis_state_.right_speed_rad = right_speed_rad;
  chassis_state_.moto_stamp = stamp;
}

void FaultDetector::update_odom_speed(double linear_speed, const rclcpp::Time & stamp)
{
  chassis_state_.odom_received = true;
  chassis_state_.odom_speed = linear_speed;
  chassis_state_.odom_stamp = stamp;
}

bool FaultDetector::check_module_nodes(const ModuleConfig & module)
{
  for (const auto & node : module.nodes) {
    auto it = node_status_.find(node);
    if (it == node_status_.end() || !it->second) {
      return false;
    }
  }
  return true;
}

void FaultDetector::append_feedback_faults(
  const ModuleConfig & module, const ModuleConfig::FeedbackRule & rule,
  const std::string & reason, std::vector<FaultInfo> & faults, const rclcpp::Time & now)
{
  const SafetyCommandType effective_safety_command =
    rule.has_safety_command_override ? rule.safety_command : module.safety_command;
  const double effective_slow_down_percentage =
    rule.has_safety_slow_down_percentage ? rule.safety_slow_down_percentage :
    module.safety_slow_down_percentage;

  for (const auto & action : rule.actions) {
    if (action == ActionType::SAFETY_SYSTEM && effective_safety_command == SafetyCommandType::NONE) {
      continue;
    }

    FaultInfo fault;
    fault.fault_key = feedback_key(module.name, rule.topic_name, rule.metric_name) +
      "|action=" + std::to_string(static_cast<int>(action));
    fault.module_name = module.name;
    fault.level = rule.level;
    fault.reason = reason;
    fault.action = action;
    fault.safety_command =
      action == ActionType::SAFETY_SYSTEM ? effective_safety_command : SafetyCommandType::NONE;
    fault.safety_slow_down_percentage =
      action == ActionType::SAFETY_SYSTEM ? effective_slow_down_percentage : 0.0;
    fault.timestamp = now;
    faults.push_back(std::move(fault));
  }
}

void FaultDetector::append_chassis_faults(
  const std::string & fault_key_prefix, FaultLevel level,
  const std::vector<ActionType> & actions, const std::string & reason,
  std::vector<FaultInfo> & faults, const rclcpp::Time & now)
{
  if (actions.empty()) {
    FaultInfo fault;
    fault.fault_key = fault_key_prefix + "|action=0";
    fault.module_name = chassis_cfg_.module_name;
    fault.level = level;
    fault.reason = reason;
    fault.action = ActionType::NONE;
    fault.safety_command = SafetyCommandType::NONE;
    fault.safety_slow_down_percentage = 0.0;
    fault.timestamp = now;
    faults.push_back(std::move(fault));
    return;
  }

  for (const auto & action : actions) {
    if (action == ActionType::SAFETY_SYSTEM && chassis_cfg_.safety_command == SafetyCommandType::NONE) {
      continue;
    }

    FaultInfo fault;
    fault.fault_key = fault_key_prefix + "|action=" + std::to_string(static_cast<int>(action));
    fault.module_name = chassis_cfg_.module_name;
    fault.level = level;
    fault.reason = reason;
    fault.action = action;
    fault.safety_command =
      action == ActionType::SAFETY_SYSTEM ? chassis_cfg_.safety_command : SafetyCommandType::NONE;
    fault.safety_slow_down_percentage =
      action == ActionType::SAFETY_SYSTEM ? chassis_cfg_.safety_slow_down_percentage : 0.0;
    fault.timestamp = now;
    faults.push_back(std::move(fault));
  }
}

bool FaultDetector::update_multi_value_state(
  const std::string & key, bool abnormal, const std::string & reason,
  std::map<std::string, RuleJudgeState> & states, std::string & active_reason)
{
  auto & state = states[key];
  if (abnormal) {
    state.abnormal_count++;
    state.normal_count = 0;
    if (!reason.empty()) {
      state.last_reason = reason;
    }
    if (!state.latched && state.abnormal_count >= multi_value_cfg_.trigger_count) {
      state.latched = true;
    }
  } else {
    state.normal_count++;
    state.abnormal_count = 0;
    if (state.latched && state.normal_count >= multi_value_cfg_.recover_count) {
      state.latched = false;
      state.last_reason.clear();
    }
  }

  if (!state.latched) {
    return false;
  }

  if (abnormal) {
    active_reason = reason;
  } else {
    active_reason = state.last_reason;
  }

  if (active_reason.empty()) {
    active_reason = "Fault latched";
  }

  return true;
}

std::vector<FaultInfo> FaultDetector::detect_faults()
{
  std::vector<FaultInfo> faults;
  const auto now = node_->now();

  for (const auto & module : modules_) {
    FaultInfo base_fault;
    const std::string node_fault_key_prefix = module.name + "|node_inactive";
    base_fault.module_name = module.name;
    base_fault.timestamp = now;
    base_fault.level = FaultLevel::NORMAL;
    base_fault.action = ActionType::NONE;
    base_fault.safety_command = SafetyCommandType::NONE;
    base_fault.safety_slow_down_percentage = 0.0;

    // 1. 节点优先级最高
    if (!check_module_nodes(module)) {
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

    // 2. 反馈 topic 阈值规则
    for (const auto & rule : module.feedback_rules) {
      const std::string key = feedback_key(module.name, rule.topic_name, rule.metric_name);
      const auto state_it = feedback_state_.find(key);

      bool abnormal = false;
      std::string reason;
      if (state_it == feedback_state_.end() || !state_it->second.received) {
        const double no_data_age = (now - config_loaded_time_).seconds();
        if (no_data_age > rule.max_stale_s) {
          std::ostringstream oss;
          oss << "Feedback missing: topic=" << rule.topic_name << " metric=" << rule.metric_name;
          abnormal = true;
          reason = oss.str();
        }
      } else {
        const auto & state = state_it->second;
        const double stale_age = (now - state.last_seen).seconds();
        if (stale_age > rule.max_stale_s) {
          std::ostringstream oss;
          oss << "Feedback stale: topic=" << rule.topic_name << " metric=" << rule.metric_name
              << " age=" << stale_age << "s max=" << rule.max_stale_s << "s";
          abnormal = true;
          reason = oss.str();
        } else if (!state.last_valid) {
          std::ostringstream oss;
          oss << "Feedback invalid: topic=" << rule.topic_name << " metric=" << rule.metric_name;
          abnormal = true;
          reason = oss.str();
        } else if (rule.has_min_value && state.last_value < rule.min_value) {
          std::ostringstream oss;
          oss << "Feedback low: topic=" << rule.topic_name << " metric=" << rule.metric_name
              << " value=" << state.last_value << " min=" << rule.min_value;
          abnormal = true;
          reason = oss.str();
        } else if (rule.has_max_value && state.last_value > rule.max_value) {
          std::ostringstream oss;
          oss << "Feedback high: topic=" << rule.topic_name << " metric=" << rule.metric_name
              << " value=" << state.last_value << " max=" << rule.max_value;
          abnormal = true;
          reason = oss.str();
        } else if (rule.min_hz > 0.0) {
          const double freq = calc_frequency(state.msg_times);
          const double max_interval = 1.0 / rule.min_hz;
          if (freq > 0.0) {
            if (freq < rule.min_hz) {
              std::ostringstream oss;
              oss << "Feedback frequency low: topic=" << rule.topic_name << " metric=" << rule.metric_name
                  << " hz=" << freq << " min_hz=" << rule.min_hz;
              abnormal = true;
              reason = oss.str();
            }
          } else if (stale_age > max_interval) {
            std::ostringstream oss;
            oss << "Feedback frequency insufficient: topic=" << rule.topic_name
                << " metric=" << rule.metric_name << " min_hz=" << rule.min_hz;
            abnormal = true;
            reason = oss.str();
          }
        }
      }

      std::string active_reason;
      if (update_multi_value_state(key, abnormal, reason, feedback_judge_state_, active_reason)) {
        append_feedback_faults(module, rule, active_reason, faults, now);
      }
    }

    // 3. 兼容老的 topic 频率规则
    for (const auto & [topic, min_hz] : module.topic_min_hz) {
      const auto it = topic_freq_.find(topic);
      const double hz = (it == topic_freq_.end()) ? 0.0 : it->second;
      const bool abnormal = hz < min_hz;
      std::string reason;
      if (abnormal) {
        std::ostringstream oss;
        oss << "Topic frequency low: topic=" << topic << " hz=" << hz << " min_hz=" << min_hz;
        reason = oss.str();
      }

      std::string active_reason;
      const std::string topic_key = module.name + "|" + topic;
      if (update_multi_value_state(topic_key, abnormal, reason, topic_judge_state_, active_reason)) {
        FaultInfo fault = base_fault;
        fault.fault_key = module.name + "|topic_legacy:" + topic + "|action=" +
          std::to_string(static_cast<int>(module.enable_supervisor ? ActionType::SUPERVISOR : ActionType::NONE));
        fault.level = FaultLevel::ERROR;
        fault.reason = active_reason;
        if (module.enable_supervisor) {
          fault.action = ActionType::SUPERVISOR;
        }
        faults.push_back(std::move(fault));
      }
    }
  }

  // 4. 底盘停留原地/反馈异常判断（全局）
  if (chassis_cfg_.enabled) {
    const bool command_fresh = chassis_state_.command_received &&
      (now - chassis_state_.command_stamp).seconds() <= chassis_cfg_.source_timeout_s;
    const bool moto_fresh = chassis_state_.moto_received &&
      (now - chassis_state_.moto_stamp).seconds() <= chassis_cfg_.source_timeout_s;
    const bool odom_fresh = chassis_state_.odom_received &&
      (now - chassis_state_.odom_stamp).seconds() <= chassis_cfg_.source_timeout_s;

    const bool command_has = command_fresh &&
      std::fabs(chassis_state_.command_speed) >= chassis_cfg_.command_speed_threshold;
    const bool moto_has = moto_fresh && chassis_state_.moto_valid &&
      std::max(std::fabs(chassis_state_.left_speed_rad), std::fabs(chassis_state_.right_speed_rad)) >=
      chassis_cfg_.moto_speed_threshold;
    const bool odom_has = odom_fresh &&
      std::fabs(chassis_state_.odom_speed) >= chassis_cfg_.odom_speed_threshold;

    bool anomaly_abnormal = false;
    bool idle_abnormal = false;
    std::string anomaly_reason;
    std::string idle_reason;

    if (command_has && !moto_has) {
      anomaly_abnormal = true;
      if (odom_has) {
        anomaly_reason = "Command active, moto inactive but odom moving (moto feedback abnormal)";
      } else {
        anomaly_reason = "Command active, moto inactive and odom not moving (chassis may be stuck)";
      }
      chassis_state_.idle_tracking = false;
    } else if (!command_has && moto_has) {
      anomaly_abnormal = true;
      anomaly_reason = "Moto active without command (chassis feedback abnormal)";
      chassis_state_.idle_tracking = false;
    } else if (!command_has && !moto_has) {
      if (!chassis_state_.idle_tracking) {
        chassis_state_.idle_tracking = true;
        chassis_state_.idle_start_time = now;
      } else if ((now - chassis_state_.idle_start_time).seconds() >= chassis_cfg_.idle_timeout_s) {
        idle_abnormal = true;
        idle_reason = "Command and moto inactive for too long (stationary timeout)";
      }
    } else {
      chassis_state_.idle_tracking = false;
    }

    std::string active_reason;
    const std::string anomaly_key = chassis_cfg_.module_name + "|chassis_anomaly";
    if (update_multi_value_state(
        anomaly_key, anomaly_abnormal, anomaly_reason, chassis_judge_state_, active_reason))
    {
      append_chassis_faults(
        anomaly_key, chassis_cfg_.anomaly_level, chassis_cfg_.anomaly_actions, active_reason,
        faults, now);
    }

    const std::string idle_key = chassis_cfg_.module_name + "|chassis_idle";
    if (update_multi_value_state(idle_key, idle_abnormal, idle_reason, chassis_judge_state_, active_reason)) {
      append_chassis_faults(
        idle_key, chassis_cfg_.idle_level, chassis_cfg_.idle_actions, active_reason, faults, now);
    }
  }

  return faults;
}

}  // namespace nav2_monitor
