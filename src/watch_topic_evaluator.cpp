#include "nav2_monitor/watch_topic_evaluator.hpp"

#include <sstream>
#include <utility>

namespace nav2_monitor
{

void WatchTopicEvaluator::set_multi_value_config(const MultiValueJudgeConfig & config)
{
  multi_value_cfg_ = config;
}

void WatchTopicEvaluator::reset()
{
  judge_states_.clear();
}

bool WatchTopicEvaluator::update_multi_value_state(
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

std::vector<FaultInfo> WatchTopicEvaluator::evaluate(
  const ModuleConfig & module,
  const MonitorDataStore & store,
  const rclcpp::Time & now) const
{
  std::vector<FaultInfo> faults;
  FaultInfo base_fault;
  base_fault.module_name = module.name;
  base_fault.timestamp = now;
  base_fault.level = FaultLevel::NORMAL;
  base_fault.action = ActionType::NONE;
  base_fault.safety_command = SafetyCommandType::NONE;
  base_fault.safety_slow_down_percentage = 0.0;

  for (const auto & [topic, min_hz] : module.watch_topic_min_hz) {
    const double hz = store.get_watch_topic_frequency(topic);
    const bool abnormal = hz < min_hz;
    std::string reason;
    if (abnormal) {
      std::ostringstream oss;
      oss << "Topic frequency low: topic=" << topic << " hz=" << hz << " min_hz=" << min_hz;
      reason = oss.str();
    }

    std::string active_reason;
    const std::string topic_key = module.name + "|" + topic;
    if (update_multi_value_state(topic_key, abnormal, reason, active_reason)) {
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

  return faults;
}

}  // namespace nav2_monitor
