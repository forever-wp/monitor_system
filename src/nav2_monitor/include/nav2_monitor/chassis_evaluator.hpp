#ifndef NAV2_MONITOR__CHASSIS_EVALUATOR_HPP_
#define NAV2_MONITOR__CHASSIS_EVALUATOR_HPP_

#include <map>
#include <string>
#include <vector>
#include <rclcpp/logger.hpp>

#include <rclcpp/rclcpp.hpp>

#include "nav2_monitor/fault_detector.hpp"
#include "nav2_monitor/monitor_data_store.hpp"

namespace nav2_monitor
{

class ChassisEvaluator
{
public:
  void set_logger(const rclcpp::Logger & logger);
  void set_multi_value_config(const MultiValueJudgeConfig & config);
  void reset();
  std::vector<FaultInfo> evaluate(
    const ChassisStationaryConfig & cfg,
    const MonitorDataStore & store,
    const rclcpp::Time & now) const;

private:
  struct RuleJudgeState
  {
    size_t abnormal_count{0};
    size_t normal_count{0};
    bool latched{false};
    std::string last_reason;
  };

  bool update_multi_value_state(
    const std::string & key,
    bool abnormal,
    const std::string & reason,
    std::string & active_reason) const;
  void append_chassis_faults(
    const ChassisStationaryConfig & cfg,
    const std::string & fault_key_prefix,
    FaultLevel level,
    const std::vector<ActionType> & actions,
    const std::string & reason,
    std::vector<FaultInfo> & faults,
    const rclcpp::Time & now) const;

  MultiValueJudgeConfig multi_value_cfg_{2, 2};
  rclcpp::Logger logger_{rclcpp::get_logger("chassis_evaluator")};
  mutable std::map<std::string, RuleJudgeState> judge_states_;
  mutable bool idle_tracking_{false};
  mutable rclcpp::Time idle_start_time_{0, 0, RCL_ROS_TIME};
  mutable int last_idle_progress_bucket_{-1};
  mutable bool has_last_drive_request_time_{false};
  mutable rclcpp::Time last_drive_request_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__CHASSIS_EVALUATOR_HPP_
