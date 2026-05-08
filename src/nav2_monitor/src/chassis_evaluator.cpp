#include "nav2_monitor/chassis_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

namespace nav2_monitor
{

void ChassisEvaluator::set_logger(const rclcpp::Logger & logger)
{
  logger_ = logger;
}

void ChassisEvaluator::set_multi_value_config(const MultiValueJudgeConfig & config)
{
  multi_value_cfg_ = config;
}

void ChassisEvaluator::reset()
{
  judge_states_.clear();
  idle_tracking_ = false;
  idle_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_idle_progress_bucket_ = -1;
  has_last_drive_request_time_ = false;
  last_drive_request_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
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
    fault.fault_type = fault_key_prefix.find("vehicle_state_source") != std::string::npos ?
      "vehicle_state_source" :
      (fault_key_prefix.find("vehicle_state_idle") != std::string::npos ?
      "vehicle_state_idle" : "vehicle_state_anomaly");
    fault.fault_model = "vehicle_state";
    fault.fault_name = fault.fault_type;
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
    fault.fault_type = fault_key_prefix.find("vehicle_state_source") != std::string::npos ?
      "vehicle_state_source" :
      (fault_key_prefix.find("vehicle_state_idle") != std::string::npos ?
      "vehicle_state_idle" : "vehicle_state_anomaly");
    fault.fault_model = "vehicle_state";
    fault.fault_name = fault.fault_type;
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
  const auto chassis_state = store.get_chassis_state();
  const bool imu_enabled = !cfg.imu_topic.empty();
  const bool odom_enabled = !cfg.odom_topic.empty();
  const bool command_fresh = chassis_state.command_received &&
    (now - chassis_state.command_stamp).seconds() <= cfg.source_timeout_s;
  const bool moto_fresh = chassis_state.moto_received &&
    (now - chassis_state.moto_stamp).seconds() <= cfg.source_timeout_s;
  const bool odom_fresh = odom_enabled && chassis_state.odom_received &&
    (now - chassis_state.odom_stamp).seconds() <= cfg.source_timeout_s;
  const bool imu_fresh = imu_enabled && chassis_state.imu_received &&
    (now - chassis_state.imu_stamp).seconds() <= cfg.source_timeout_s;

  const bool command_has = command_fresh &&
    std::fabs(chassis_state.command_speed) >= cfg.command_speed_threshold;
  const bool moto_has = moto_fresh && chassis_state.moto_valid &&
    std::max(std::fabs(chassis_state.left_speed_rad), std::fabs(chassis_state.right_speed_rad)) >=
    cfg.moto_speed_threshold;
  const bool odom_has = odom_fresh &&
    std::fabs(chassis_state.odom_speed) >= cfg.odom_speed_threshold;
  const bool imu_has = imu_fresh && (
    std::fabs(chassis_state.imu_speed_estimate) >= cfg.imu_speed_threshold ||
    std::fabs(chassis_state.imu_yaw_rate) >= cfg.imu_yaw_rate_threshold);
  const bool command_source_ready = command_fresh;
  const bool moto_available = moto_fresh && chassis_state.moto_valid;
  const bool actual_source_available = imu_fresh || odom_fresh || moto_available;
  const bool actual_motion_has = imu_has || odom_has || moto_has;
  std::vector<std::string> available_sources;
  std::vector<std::string> moving_sources;
  std::vector<std::string> stationary_sources;
  if (imu_fresh) {
    available_sources.push_back("IMU");
    (imu_has ? moving_sources : stationary_sources).push_back("IMU");
  }
  if (odom_fresh) {
    available_sources.push_back("raw odom");
    (odom_has ? moving_sources : stationary_sources).push_back("raw odom");
  }
  if (moto_available) {
    available_sources.push_back("moto feedback");
    (moto_has ? moving_sources : stationary_sources).push_back("moto feedback");
  }

  auto join_sources = [](const std::vector<std::string> & sources) {
    std::ostringstream oss;
    for (size_t idx = 0; idx < sources.size(); ++idx) {
      if (idx > 0) {
        oss << ",";
      }
      oss << sources[idx];
    }
    return oss.str();
  };
  const std::string available_source_names = join_sources(available_sources);
  const std::string moving_source_names = join_sources(moving_sources);
  const std::string stationary_source_names = join_sources(stationary_sources);

  bool source_abnormal = false;
  bool anomaly_abnormal = false;
  bool idle_abnormal = false;
  std::string source_reason;
  std::string anomaly_reason;
  std::string idle_reason;

  if (!command_source_ready) {
    source_abnormal = true;
    source_reason = chassis_state.command_received ?
      "Command source stale; skip vehicle state judgment" :
      "Command source missing; skip vehicle state judgment";
    idle_tracking_ = false;
  } else if (!actual_source_available) {
    source_abnormal = true;
    source_reason = "All motion feedback sources are missing or stale; skip vehicle state judgment";
    idle_tracking_ = false;
  } else {
    if (command_has) {
      has_last_drive_request_time_ = true;
      last_drive_request_time_ = now;
    }

    if (command_has && !actual_motion_has) {
      anomaly_abnormal = true;
      anomaly_reason =
        "Command active but vehicle is not moving according to " + available_source_names +
        " (possible emergency stop or chassis fault)";
      idle_tracking_ = false;
    } else if (!command_has && actual_motion_has) {
      const double coast_elapsed = has_last_drive_request_time_ ?
        (now - last_drive_request_time_).seconds() : cfg.coast_grace_s;
      const bool within_coast_grace = has_last_drive_request_time_ &&
        coast_elapsed <= cfg.coast_grace_s;
      if (!within_coast_grace) {
        anomaly_abnormal = true;
        anomaly_reason =
          "Vehicle still moving without command after coast grace (" +
          std::to_string(coast_elapsed) + "s, moving_sources=" + moving_source_names +
          ", possible chassis damage or feedback abnormal)";
        idle_tracking_ = false;
      } else {
        if (idle_tracking_) {
          RCLCPP_INFO(
            logger_,
            "[vehicle_state_judge] idle reset: coasting after command elapsed=%.2fs / %.2fs",
            coast_elapsed, cfg.coast_grace_s);
        }
        idle_tracking_ = false;
        last_idle_progress_bucket_ = -1;
      }
    } else if (!command_has && !actual_motion_has) {
      if (!idle_tracking_) {
        idle_tracking_ = true;
        idle_start_time_ = now;
        last_idle_progress_bucket_ = 0;
          RCLCPP_INFO(
            logger_,
            "[vehicle_state_judge] idle start: idle_timeout=%.1fs command=false motion=false sources=%s",
            cfg.idle_timeout_s,
            available_source_names.c_str());
      } else if ((now - idle_start_time_).seconds() >= cfg.idle_timeout_s) {
        idle_abnormal = true;
        idle_reason = "Vehicle stationary without command for too long";
        last_idle_progress_bucket_ = static_cast<int>(cfg.idle_timeout_s);
      } else {
        const int bucket = static_cast<int>((now - idle_start_time_).seconds());
        if (bucket > last_idle_progress_bucket_) {
          last_idle_progress_bucket_ = bucket;
          RCLCPP_INFO(
            logger_,
            "[vehicle_state_judge] idle counting: elapsed=%ds / %.1fs command=false motion=false sources=%s",
            bucket, cfg.idle_timeout_s,
            available_source_names.c_str());
        }
      }
    } else {
      if (idle_tracking_) {
        RCLCPP_INFO(
          logger_,
          "[vehicle_state_judge] idle reset: command=true motion=true moving_sources=%s stationary_sources=%s elapsed=%.2fs",
          moving_source_names.c_str(),
          stationary_source_names.c_str(),
          (now - idle_start_time_).seconds());
      }
      idle_tracking_ = false;
      last_idle_progress_bucket_ = -1;
    }
  }

  std::string active_reason;
  const std::string source_key = cfg.module_name + "|vehicle_state_source";
  if (update_multi_value_state(source_key, source_abnormal, source_reason, active_reason)) {
    append_chassis_faults(
      cfg, source_key, cfg.source_level, cfg.source_actions, active_reason, faults, now);
  }

  const std::string anomaly_key = cfg.module_name + "|vehicle_state_anomaly";
  if (update_multi_value_state(anomaly_key, anomaly_abnormal, anomaly_reason, active_reason)) {
    append_chassis_faults(
      cfg, anomaly_key, cfg.anomaly_level, cfg.anomaly_actions, active_reason, faults, now);
  }

  const std::string idle_key = cfg.module_name + "|vehicle_state_idle";
  if (update_multi_value_state(idle_key, idle_abnormal, idle_reason, active_reason)) {
    append_chassis_faults(
      cfg, idle_key, cfg.idle_level, cfg.idle_actions, active_reason, faults, now);
  }

  return faults;
}

}  // namespace nav2_monitor
