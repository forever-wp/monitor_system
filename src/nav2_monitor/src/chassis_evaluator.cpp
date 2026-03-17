#include "nav2_monitor/chassis_evaluator.hpp"

#include <algorithm>
#include <cmath>
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
  bool anomaly_abnormal = false;
  bool idle_abnormal = false;
  std::string anomaly_reason;
  std::string idle_reason;

  if (imu_enabled) {
    if (!imu_fresh) {
      anomaly_abnormal = true;
      anomaly_reason = "IMU motion source missing or stale";
      idle_tracking_ = false;
    } else if (imu_has) {
      if (idle_tracking_) {
        RCLCPP_INFO(
          logger_,
          "[chassis_idle] reset: imu_motion=true elapsed=%.2fs",
          (now - idle_start_time_).seconds());
      }
      idle_tracking_ = false;
      last_idle_progress_bucket_ = -1;
    } else {
      if (!idle_tracking_) {
        idle_tracking_ = true;
        idle_start_time_ = now;
        last_idle_progress_bucket_ = 0;
        RCLCPP_INFO(
          logger_,
          "[chassis_idle] start counting: idle_timeout=%.1fs imu_motion=false",
          cfg.idle_timeout_s);
      } else if ((now - idle_start_time_).seconds() >= cfg.idle_timeout_s) {
        idle_abnormal = true;
        idle_reason = "IMU indicates stationary for too long (stationary timeout)";
        last_idle_progress_bucket_ = static_cast<int>(cfg.idle_timeout_s);
      } else {
        const int bucket = static_cast<int>((now - idle_start_time_).seconds());
        if (bucket > last_idle_progress_bucket_) {
          last_idle_progress_bucket_ = bucket;
          RCLCPP_INFO(
            logger_,
            "[chassis_idle] counting: elapsed=%ds / %.1fs imu_motion=false",
            bucket, cfg.idle_timeout_s);
        }
      }
    }
  } else if (odom_enabled) {
    const bool drive_request_has = command_has || moto_has;
    if (drive_request_has && !odom_has) {
      anomaly_abnormal = true;
      if (command_has && moto_has) {
        anomaly_reason = "Drive request active but raw odom stationary (chassis may be stuck)";
      } else if (command_has) {
        anomaly_reason = "Command active but raw odom stationary (chassis may be stuck)";
      } else {
        anomaly_reason = "Moto active but raw odom stationary (feedback indicates motion but raw odom does not)";
      }
      idle_tracking_ = false;
    } else if (!drive_request_has && odom_has) {
      anomaly_abnormal = true;
      if (moto_has) {
        anomaly_reason = "Raw odom moving without command (unexpected chassis motion)";
      } else {
        anomaly_reason = "Raw odom moving without command or moto request (unexpected chassis motion)";
      }
      idle_tracking_ = false;
    } else if (!drive_request_has && !odom_has) {
      if (!idle_tracking_) {
        idle_tracking_ = true;
        idle_start_time_ = now;
        last_idle_progress_bucket_ = 0;
        RCLCPP_INFO(
          logger_,
          "[chassis_idle] start counting: idle_timeout=%.1fs drive_request=false raw_odom=false",
          cfg.idle_timeout_s);
      } else if ((now - idle_start_time_).seconds() >= cfg.idle_timeout_s) {
        idle_abnormal = true;
        idle_reason = "Raw odom stationary for too long without drive request (stationary timeout)";
        last_idle_progress_bucket_ = static_cast<int>(cfg.idle_timeout_s);
      } else {
        const int bucket = static_cast<int>((now - idle_start_time_).seconds());
        if (bucket > last_idle_progress_bucket_) {
          last_idle_progress_bucket_ = bucket;
          RCLCPP_INFO(
            logger_,
            "[chassis_idle] counting: elapsed=%ds / %.1fs drive_request=false raw_odom=false",
            bucket, cfg.idle_timeout_s);
        }
      }
    } else {
      if (idle_tracking_) {
        RCLCPP_INFO(
          logger_,
          "[chassis_idle] reset: drive_request=%s raw_odom=%s elapsed=%.2fs",
          drive_request_has ? "true" : "false",
          odom_has ? "true" : "false",
          (now - idle_start_time_).seconds());
      }
      idle_tracking_ = false;
      last_idle_progress_bucket_ = -1;
    }
  } else {
    if (command_has && !moto_has) {
      anomaly_abnormal = true;
      anomaly_reason = "Command active, moto inactive (raw odom disabled, chassis may be stuck or moto feedback abnormal)";
      idle_tracking_ = false;
    } else if (!command_has && moto_has) {
      anomaly_abnormal = true;
      anomaly_reason = "Moto active without command (raw odom disabled, feedback abnormal)";
      idle_tracking_ = false;
    } else if (!command_has && !moto_has) {
      if (!idle_tracking_) {
        idle_tracking_ = true;
        idle_start_time_ = now;
        last_idle_progress_bucket_ = 0;
        RCLCPP_INFO(
          logger_,
          "[chassis_idle] start counting: idle_timeout=%.1fs command_has=false moto_has=false raw_odom=disabled",
          cfg.idle_timeout_s);
      } else if ((now - idle_start_time_).seconds() >= cfg.idle_timeout_s) {
        idle_abnormal = true;
        idle_reason = "Command and moto inactive for too long (stationary timeout)";
        last_idle_progress_bucket_ = static_cast<int>(cfg.idle_timeout_s);
      } else {
        const int bucket = static_cast<int>((now - idle_start_time_).seconds());
        if (bucket > last_idle_progress_bucket_) {
          last_idle_progress_bucket_ = bucket;
          RCLCPP_INFO(
            logger_,
            "[chassis_idle] counting: elapsed=%ds / %.1fs command_has=false moto_has=false raw_odom=disabled",
            bucket, cfg.idle_timeout_s);
        }
      }
    } else {
      if (idle_tracking_) {
        RCLCPP_INFO(
          logger_,
          "[chassis_idle] reset: command_has=%s moto_has=%s raw_odom=disabled elapsed=%.2fs",
          command_has ? "true" : "false",
          moto_has ? "true" : "false",
          (now - idle_start_time_).seconds());
      }
      idle_tracking_ = false;
      last_idle_progress_bucket_ = -1;
    }
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
