#include "nav2_monitor/feedback_rule_evaluator.hpp"

#include <algorithm>
#include <sstream>
#include <utility>

namespace nav2_monitor
{

void FeedbackRuleEvaluator::set_multi_value_config(const MultiValueJudgeConfig & config)
{
  multi_value_cfg_ = config;
}

void FeedbackRuleEvaluator::reset()
{
  judge_states_.clear();
}

bool FeedbackRuleEvaluator::update_multi_value_state(
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

void FeedbackRuleEvaluator::append_feedback_faults(
  const ModuleConfig & module,
  const ModuleConfig::FeedbackRule & rule,
  const std::string & reason,
  std::vector<FaultInfo> & faults,
  const rclcpp::Time & now) const
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
    fault.fault_key = feedback_key(module.name, rule.source_topic, rule.metric_name) +
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

double FeedbackRuleEvaluator::calc_frequency(const std::deque<rclcpp::Time> & msg_times)
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

std::string FeedbackRuleEvaluator::feedback_key(
  const std::string & module_name,
  const std::string & source_topic,
  const std::string & metric_name)
{
  return module_name + "|feedback:" + source_topic + ":" + metric_name;
}

std::vector<FaultInfo> FeedbackRuleEvaluator::evaluate(
  const ModuleConfig & module,
  const MonitorDataStore & store,
  const rclcpp::Time & now,
  const rclcpp::Time & config_loaded_time) const
{
  std::vector<FaultInfo> faults;

  for (const auto & rule : module.feedback_rules) {
    const double max_stale_s = std::max(0.0, rule.max_stale_s);
    const std::string key = feedback_key(module.name, rule.source_topic, rule.metric_name);
    const auto * state = store.get_feedback_state(key);

    bool abnormal = false;
    std::string reason;
    if (state == nullptr || !state->received) {
      const double no_data_age = (now - config_loaded_time).seconds();
      if (no_data_age > max_stale_s) {
        std::ostringstream oss;
        oss << "Feedback missing: source_topic=" << rule.source_topic << " metric=" << rule.metric_name;
        abnormal = true;
        reason = oss.str();
      }
    } else {
      const double stale_age = (now - state->last_seen).seconds();
      if (stale_age > max_stale_s) {
        std::ostringstream oss;
        oss << "Feedback stale: source_topic=" << rule.source_topic << " metric=" << rule.metric_name
            << " age=" << stale_age << "s max=" << max_stale_s << "s";
        abnormal = true;
        reason = oss.str();
      } else if (!state->last_valid) {
        std::ostringstream oss;
        oss << "Feedback invalid: source_topic=" << rule.source_topic << " metric=" << rule.metric_name;
        abnormal = true;
        reason = oss.str();
      } else if (rule.has_min_value && state->last_value < rule.min_value) {
        std::ostringstream oss;
        oss << "Feedback low: source_topic=" << rule.source_topic << " metric=" << rule.metric_name
            << " value=" << state->last_value << " min=" << rule.min_value;
        abnormal = true;
        reason = oss.str();
      } else if (rule.has_max_value && state->last_value > rule.max_value) {
        std::ostringstream oss;
        oss << "Feedback high: source_topic=" << rule.source_topic << " metric=" << rule.metric_name
            << " value=" << state->last_value << " max=" << rule.max_value;
        abnormal = true;
        reason = oss.str();
      } else if (rule.min_hz > 0.0) {
        const double freq = calc_frequency(state->msg_times);
        const double max_interval = 1.0 / rule.min_hz;
        if (freq > 0.0) {
          if (freq < rule.min_hz) {
            std::ostringstream oss;
            oss << "Feedback frequency low: source_topic=" << rule.source_topic << " metric=" << rule.metric_name
                << " hz=" << freq << " min_hz=" << rule.min_hz;
            abnormal = true;
            reason = oss.str();
          }
        } else if (stale_age > max_interval) {
          std::ostringstream oss;
          oss << "Feedback frequency insufficient: source_topic=" << rule.source_topic
              << " metric=" << rule.metric_name << " min_hz=" << rule.min_hz;
          abnormal = true;
          reason = oss.str();
        }
      }
    }

    std::string active_reason;
    if (update_multi_value_state(key, abnormal, reason, active_reason)) {
      append_feedback_faults(module, rule, active_reason, faults, now);
    }
  }

  return faults;
}

}  // namespace nav2_monitor
