#ifndef NAV2_MONITOR__WATCH_TOPIC_EVALUATOR_HPP_
#define NAV2_MONITOR__WATCH_TOPIC_EVALUATOR_HPP_

#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "nav2_monitor/fault_detector.hpp"
#include "nav2_monitor/monitor_data_store.hpp"

namespace nav2_monitor
{

class WatchTopicEvaluator
{
public:
  void set_multi_value_config(const MultiValueJudgeConfig & config);
  void reset();
  std::vector<FaultInfo> evaluate(
    const ModuleConfig & module,
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

  MultiValueJudgeConfig multi_value_cfg_{2, 2};
  mutable std::map<std::string, RuleJudgeState> judge_states_;
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__WATCH_TOPIC_EVALUATOR_HPP_
