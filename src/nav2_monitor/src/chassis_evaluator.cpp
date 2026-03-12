#include "nav2_monitor/chassis_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace nav2_monitor
{

void ChassisEvaluator::set_multi_value_config(const MultiValueJudgeConfig & config)
{
  multi_value_cfg_ = config;
}

void ChassisEvaluator::reset()
{
  judge_states_.clear();
  idle_tracking_ = false;
  idle_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

bool ChassisEvaluator::update_multi_value_state(
  const std::string & key,
  bool abnormal,
  const std::string & reason,
  std::string & active_reason) const
{
  auto & state = judge_states_[key];
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

  active_reason = abnormal ? reason : state.last_reason;
  if (active_reason.empty()) {
    active_reason = "Fault latched";
  }
  return true;
}

void ChassisEvaluator::append_chassis_faults(
  const ChassisStationaryConfig & cfg,
  const std::string & fault_key_prefix,
  FaultLevel level,
  const std::vector<ActionType> & actions,
  const std::string & reason,
  std::vector<FaultInfo> & faults,
  const rclcpp::Time & now) const
{
  if (actions.empty()) {
    FaultInfo fault;
    fault.fault_key = fault_key_prefix + "|action=0";
    fault.module_name = cfg.module_name;
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
    if (action == ActionType::SAFETY_SYSTEM && cfg.safety_command == SafetyCommandType::NONE) {
      continue;
    }

    FaultInfo fault;
    fault.fault_key = fault_key_prefix + "|action=" + std::to_string(static_cast<int>(action));
    fault.module_name = cfg.module_name;
    fault.level = level;
    fault.reason = reason;
    fault.action = action;
    fault.safety_command =
      action == ActionType::SAFETY_SYSTEM ? cfg.safety_command : SafetyCommandType::NONE;
    fault.safety_slow_down_percentage =
      action == ActionType::SAFETY_SYSTEM ? cfg.safety_slow_down_percentage : 0.0;
    fault.timestamp = now;
    faults.push_back(std::move(fault));
  }
}

std::vector<FaultInfo> ChassisEvaluator::evaluate(
  const ChassisStationaryConfig & cfg,
  const MonitorDataStore & store,
  const rclcpp::Time & now) const
{
  std::vector<FaultInfo> faults;
  const auto & chassis_state = store.get_chassis_state();
  const bool command_fresh = chassis_state.command_received &&
    (now - chassis_state.command_stamp).seconds() <= cfg.source_timeout_s;
  const bool moto_fresh = chassis_state.moto_received &&
    (now - chassis_state.moto_stamp).seconds() <= cfg.source_timeout_s;
  const bool odom_fresh = chassis_state.odom_received &&
    (now - chassis_state.odom_stamp).seconds() <= cfg.source_timeout_s;

  const bool command_has = command_fresh &&
    std::fabs(chassis_state.command_speed) >= cfg.command_speed_threshold;
  const bool moto_has = moto_fresh && chassis_state.moto_valid &&
    std::max(std::fabs(chassis_state.left_speed_rad), std::fabs(chassis_state.right_speed_rad)) >=
    cfg.moto_speed_threshold;
  const bool odom_has = odom_fresh &&
    std::fabs(chassis_state.odom_speed) >= cfg.odom_speed_threshold;

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
    idle_tracking_ = false;
  } else if (!command_has && moto_has) {
    anomaly_abnormal = true;
    anomaly_reason = "Moto active without command (chassis feedback abnormal)";
    idle_tracking_ = false;
  } else if (!command_has && !moto_has) {
    if (!idle_tracking_) {
      idle_tracking_ = true;
      idle_start_time_ = now;
    } else if ((now - idle_start_time_).seconds() >= cfg.idle_timeout_s) {
      idle_abnormal = true;
      idle_reason = "Command and moto inactive for too long (stationary timeout)";
    }
  } else {
    idle_tracking_ = false;
  }

  std::string active_reason;
  const std::string anomaly_key = cfg.module_name + "|chassis_anomaly";
  if (update_multi_value_state(anomaly_key, anomaly_abnormal, anomaly_reason, active_reason)) {
    append_chassis_faults(
      cfg, anomaly_key, cfg.anomaly_level, cfg.anomaly_actions, active_reason, faults, now);
  }

  const std::string idle_key = cfg.module_name + "|chassis_idle";
  if (update_multi_value_state(idle_key, idle_abnormal, idle_reason, active_reason)) {
    append_chassis_faults(
      cfg, idle_key, cfg.idle_level, cfg.idle_actions, active_reason, faults, now);
  }

  return faults;
}

}  // namespace nav2_monitor
