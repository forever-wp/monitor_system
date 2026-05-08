#ifndef NAV2_MONITOR__FEEDBACK_RULE_EVALUATOR_HPP_
#define NAV2_MONITOR__FEEDBACK_RULE_EVALUATOR_HPP_

#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "nav2_monitor/fault_detector.hpp"
#include "nav2_monitor/monitor_data_store.hpp"

namespace nav2_monitor
{

class FeedbackRuleEvaluator
{
public:
  void set_multi_value_config(const MultiValueJudgeConfig & config);
  void reset();
  std::vector<FaultInfo> evaluate(
    const ModuleConfig & module,
    const MonitorDataStore & store,
    const rclcpp::Time & now,
    const rclcpp::Time & config_loaded_time) const;

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
  void append_feedback_faults(
    const ModuleConfig & module,
    const ModuleConfig::FeedbackRule & rule,
    const std::string & reason,
    std::vector<FaultInfo> & faults,
    const rclcpp::Time & now) const;
  static double calc_receive_frequency(
    const std::deque<rclcpp::Time> & receive_times,
    const rclcpp::Time & now,
    double min_hz);
  static double receive_window_s(double min_hz);
  static double receive_gap_timeout_s(double min_hz);
  static std::string feedback_key(
    const std::string & module_name,
    const std::string & source_topic,
    const std::string & metric_name);

  MultiValueJudgeConfig multi_value_cfg_{2, 2};
  mutable std::map<std::string, RuleJudgeState> judge_states_;
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__FEEDBACK_RULE_EVALUATOR_HPP_
