#include "nav2_monitor/event_codex_arbiter.hpp"

#include <algorithm>
#include <sstream>

namespace nav2_monitor
{

namespace
{
constexpr int kSingleRulePriority = 100;
constexpr int kCombinedRulePriority = 1000;

bool time_is_set(const rclcpp::Time & time)
{
  return time.nanoseconds() != 0;
}

double elapsed_s(const rclcpp::Time & now, const rclcpp::Time & since)
{
  if (!time_is_set(since)) {
    return 0.0;
  }
  return (now - since).seconds();
}

EventCodexConditionConfig exact_key_condition(const std::string & key)
{
  EventCodexConditionConfig condition;
  condition.event_key = key;
  return condition;
}

std::vector<EventCodexConditionConfig> effective_conditions(
  const std::vector<EventCodexConditionConfig> & configured_conditions,
  const std::vector<std::string> & exact_keys)
{
  if (!configured_conditions.empty()) {
    return configured_conditions;
  }

  std::vector<EventCodexConditionConfig> conditions;
  conditions.reserve(exact_keys.size());
  for (const auto & key : exact_keys) {
    conditions.push_back(exact_key_condition(key));
  }
  return conditions;
}

bool condition_matches(
  const std::string & key,
  const FaultInfo & fact,
  const EventCodexConditionConfig & condition)
{
  if (!condition.event_key.empty() && key != condition.event_key) {
    return false;
  }
  if (!condition.module_name.empty() && fact.module_name != condition.module_name) {
    return false;
  }
  if (!condition.fault_type.empty() && fact.fault_type != condition.fault_type) {
    return false;
  }
  if (!condition.fault_model.empty() && fact.fault_model != condition.fault_model) {
    return false;
  }
  if (!condition.fault_name.empty() && fact.fault_name != condition.fault_name) {
    return false;
  }
  if (!condition.fault_key_prefix.empty() &&
    key.rfind(condition.fault_key_prefix, 0) != 0)
  {
    return false;
  }
  if (!condition.fault_key_contains.empty() &&
    key.find(condition.fault_key_contains) == std::string::npos)
  {
    return false;
  }
  if (condition.action_set && fact.action != condition.action) {
    return false;
  }
  return true;
}

std::string condition_label(const EventCodexConditionConfig & condition)
{
  if (!condition.event_key.empty()) {
    return condition.event_key;
  }

  std::vector<std::string> parts;
  auto add_part = [&parts](const std::string & name, const std::string & value) {
      if (!value.empty()) {
        parts.push_back(name + "=" + value);
      }
    };
  add_part("module", condition.module_name);
  add_part("fault_type", condition.fault_type);
  add_part("fault_model", condition.fault_model);
  add_part("fault_name", condition.fault_name);
  add_part("prefix", condition.fault_key_prefix);
  add_part("contains", condition.fault_key_contains);
  if (condition.action_set) {
    parts.push_back("action=" + std::to_string(static_cast<int>(condition.action)));
  }

  std::ostringstream oss;
  for (size_t i = 0; i < parts.size(); ++i) {
    if (i > 0) {
      oss << ",";
    }
    oss << parts[i];
  }
  return oss.str();
}
}

void EventCodexArbiter::set_combined_fault_rules(
  const std::vector<CombinedFaultRuleConfig> & rules)
{
  combined_rules_ = rules;
  std::set<std::string> valid_rule_ids;
  for (const auto & rule : combined_rules_) {
    valid_rule_ids.insert("combined|" + rule.name);
  }
  for (auto it = rule_states_.begin(); it != rule_states_.end(); ) {
    if (valid_rule_ids.count(it->first) == 0 && it->first.rfind("single|", 0) != 0) {
      it = rule_states_.erase(it);
    } else {
      ++it;
    }
  }
}

std::string EventCodexArbiter::fallback_fault_key(const FaultInfo & fault)
{
  return fault.module_name + "|" + std::to_string(static_cast<int>(fault.action)) + "|" + fault.reason;
}

std::string EventCodexArbiter::format_recover_reason(
  const FaultInfo & fault,
  const std::string & fault_key)
{
  return "RECOVER fault_key=" + fault_key + "; previous_reason=" + fault.reason;
}

bool EventCodexArbiter::is_vehicle_state_judge_fault(const FaultInfo & fault)
{
  return fault.module_name == "vehicle_state_judge" ||
         fault.fault_key.find("vehicle_state_judge|") == 0 ||
         fault.fault_key.find("|vehicle_state_") != std::string::npos ||
         fault.fault_model == "vehicle_state";
}

bool EventCodexArbiter::has_action(const EventCodexSelectedRule & rule, ActionType action)
{
  return std::find(rule.actions.begin(), rule.actions.end(), action) != rule.actions.end();
}

int EventCodexArbiter::fault_level_priority(FaultLevel level)
{
  switch (level) {
    case FaultLevel::CRITICAL:
      return 3;
    case FaultLevel::ERROR:
      return 2;
    case FaultLevel::WARNING:
      return 1;
    case FaultLevel::NORMAL:
    default:
      return 0;
  }
}

int EventCodexArbiter::safety_command_priority(SafetyCommandType command)
{
  switch (command) {
    case SafetyCommandType::EMERGENCY_STOP:
      return 3;
    case SafetyCommandType::SOFT_STOP:
      return 2;
    case SafetyCommandType::SLOW_DOWN:
      return 1;
    case SafetyCommandType::NONE:
    default:
      return 0;
  }
}

int EventCodexArbiter::execution_level(const Candidate & candidate)
{
  int level = fault_level_priority(candidate.rule.level) * 10;
  if (has_action(candidate.rule, ActionType::SAFETY_SYSTEM)) {
    level += 100 + safety_command_priority(candidate.rule.safety_command);
  }
  if (has_action(candidate.rule, ActionType::SUPERVISOR)) {
    level += 20;
  }
  if (candidate.rule.manual_takeover) {
    level += 5;
  }
  return level;
}

std::string EventCodexArbiter::join_strings(
  const std::vector<std::string> & values,
  const std::string & sep)
{
  std::ostringstream oss;
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      oss << sep;
    }
    oss << values[i];
  }
  return oss.str();
}

void EventCodexArbiter::append_unique(std::vector<std::string> & values, const std::string & value)
{
  if (!value.empty() && std::find(values.begin(), values.end(), value) == values.end()) {
    values.push_back(value);
  }
}

bool EventCodexArbiter::update_candidate_runtime(
  Candidate & candidate,
  bool matched,
  const rclcpp::Time & now)
{
  auto & state = rule_states_[candidate.rule.rule_id];
  candidate.was_active = state.active;

  if (matched) {
    state.first_clear_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
    if (!state.active) {
      if (!time_is_set(state.first_match_time)) {
        state.first_match_time = now;
      }
      if (elapsed_s(now, state.first_match_time) + 1e-9 < candidate.rule.enter_hold_s) {
        candidate.ready = false;
        return false;
      }
      state.active = true;
      state.activated_time = now;
    }
    return true;
  }

  state.first_match_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  if (!state.active) {
    return false;
  }

  if (elapsed_s(now, state.activated_time) + 1e-9 < candidate.rule.min_hold_s) {
    candidate.ready = true;
    candidate.clearing = false;
    return true;
  }

  if (!time_is_set(state.first_clear_time)) {
    state.first_clear_time =
      state.activated_time + rclcpp::Duration::from_seconds(candidate.rule.min_hold_s);
  }
  if (elapsed_s(now, state.first_clear_time) + 1e-9 < candidate.rule.clear_hold_s) {
    candidate.ready = true;
    candidate.clearing = true;
    return true;
  }

  state.active = false;
  state.activated_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  state.first_clear_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  return false;
}

std::vector<EventCodexArbiter::Candidate> EventCodexArbiter::build_candidates(
  const std::map<std::string, FaultInfo> & active_facts,
  const rclcpp::Time & now)
{
  std::vector<Candidate> candidates;

  for (const auto & [key, fact] : active_facts) {
    EventCodexSelectedRule rule;
    rule.rule_id = "single|" + key;
    rule.rule_name = key;
    rule.event_keys = {key};
    rule.module_names = {fact.module_name};
    rule.level = fact.level;
    rule.reason = fact.reason;
    rule.report_reason = fact.reason;
    rule.combined = false;
    rule.manual_takeover = is_vehicle_state_judge_fault(fact);
    if (fact.action != ActionType::NONE) {
      rule.actions.push_back(fact.action);
    }
    if (fact.action == ActionType::SAFETY_SYSTEM && fact.safety_command != SafetyCommandType::NONE) {
      rule.safety_command = fact.safety_command;
      rule.safety_slow_down_percentage = fact.safety_slow_down_percentage;
    }
    if (rule.actions.empty() && !rule.manual_takeover) {
      continue;
    }

    Candidate candidate;
    candidate.rule = std::move(rule);
    candidate.facts = {fact};
    candidate.priority = kSingleRulePriority;
    candidate.execution_level = execution_level(candidate);
    candidate.was_active = previous_selected_rule_ids_.count(candidate.rule.rule_id) > 0;
    candidates.push_back(std::move(candidate));
  }

  for (const auto & config : combined_rules_) {
    const auto all_conditions =
      effective_conditions(config.when_all_conditions, config.when_all_fault_keys);
    const auto any_conditions =
      effective_conditions(config.when_any_conditions, config.when_any_fault_keys);

    bool all_matched = true;
    std::vector<FaultInfo> matched_facts;
    std::set<std::string> matched_keys;
    matched_facts.reserve(all_conditions.size() + any_conditions.size());
    for (const auto & condition : all_conditions) {
      bool condition_matched = false;
      for (const auto & [key, fact] : active_facts) {
        if (!condition_matches(key, fact, condition)) {
          continue;
        }
        condition_matched = true;
        if (matched_keys.insert(key).second) {
          matched_facts.push_back(fact);
        }
      }
      if (!condition_matched) {
        all_matched = false;
        break;
      }
    }
    bool any_matched = any_conditions.empty();
    for (const auto & condition : any_conditions) {
      for (const auto & [key, fact] : active_facts) {
        if (!condition_matches(key, fact, condition)) {
          continue;
        }
        any_matched = true;
        if (matched_keys.insert(key).second) {
          matched_facts.push_back(fact);
        }
      }
    }
    const size_t required_count = config.min_match_count > 0 ? config.min_match_count :
      (!any_conditions.empty() && all_conditions.empty() ? 1 : 0);
    const bool matched =
      all_matched && any_matched && (required_count == 0 || matched_keys.size() >= required_count);

    EventCodexSelectedRule rule;
    rule.rule_id = "combined|" + config.name;
    rule.rule_name = config.name;
    rule.event_keys.assign(matched_keys.begin(), matched_keys.end());
    if (rule.event_keys.empty()) {
      for (const auto & condition : all_conditions) {
        append_unique(rule.event_keys, condition_label(condition));
      }
      for (const auto & condition : any_conditions) {
        append_unique(rule.event_keys, condition_label(condition));
      }
    }
    rule.level = config.level;
    rule.actions = config.actions;
    rule.safety_command = config.safety_command;
    rule.safety_slow_down_percentage = config.safety_slow_down_percentage;
    rule.reason = config.reason.empty() ? ("Combined fault triggered: " + config.name) : config.reason;
    rule.report_reason = config.report_reason.empty() ? rule.reason : config.report_reason;
    rule.combined = true;
    rule.manual_takeover = config.manual_takeover;
    rule.enter_hold_s = config.enter_hold_s;
    rule.clear_hold_s = config.clear_hold_s;
    rule.min_hold_s = config.min_hold_s;
    rule.nodemanager_modules = config.nodemanager_modules;
    for (const auto & fact : matched_facts) {
      append_unique(rule.module_names, fact.module_name);
      rule.manual_takeover = rule.manual_takeover || is_vehicle_state_judge_fault(fact);
    }
    if (rule.actions.empty() && !rule.manual_takeover) {
      rule.actions.push_back(ActionType::NONE);
    }

    Candidate candidate;
    candidate.rule = std::move(rule);
    candidate.facts = std::move(matched_facts);
    candidate.priority = config.priority > 0 ? config.priority : kCombinedRulePriority;
    candidate.execution_level = execution_level(candidate);
    candidate.was_active = previous_selected_rule_ids_.count(candidate.rule.rule_id) > 0;
    if (update_candidate_runtime(candidate, matched, now)) {
      candidates.push_back(std::move(candidate));
    }
  }

  return candidates;
}

std::vector<EventCodexArbiter::Candidate> EventCodexArbiter::select_winners(
  std::vector<Candidate> candidates) const
{
  std::sort(
    candidates.begin(), candidates.end(),
    [](const Candidate & lhs, const Candidate & rhs) {
      if (lhs.priority != rhs.priority) {
        return lhs.priority > rhs.priority;
      }
      if (lhs.execution_level != rhs.execution_level) {
        return lhs.execution_level > rhs.execution_level;
      }
      if (lhs.rule.event_keys.size() != rhs.rule.event_keys.size()) {
        return lhs.rule.event_keys.size() > rhs.rule.event_keys.size();
      }
      if (lhs.was_active != rhs.was_active) {
        return lhs.was_active;
      }
      return lhs.rule.rule_id < rhs.rule.rule_id;
    });

  std::vector<Candidate> winners;
  std::set<std::string> covered_event_keys;
  for (auto & candidate : candidates) {
    bool intersects = false;
    for (const auto & key : candidate.rule.event_keys) {
      if (covered_event_keys.count(key) > 0) {
        intersects = true;
        break;
      }
    }
    if (intersects) {
      continue;
    }

    for (const auto & key : candidate.rule.event_keys) {
      covered_event_keys.insert(key);
    }
    winners.push_back(std::move(candidate));
  }

  std::sort(
    winners.begin(), winners.end(),
    [](const Candidate & lhs, const Candidate & rhs) {
      return lhs.rule.rule_id < rhs.rule.rule_id;
    });
  return winners;
}

EventExecutionPlan EventCodexArbiter::build_plan(const std::vector<Candidate> & winners) const
{
  EventExecutionPlan plan;
  std::vector<std::string> signature_parts;
  std::map<std::string, EventNodeManagerDecision> nodemanager_by_module;

  bool safety_active = false;
  SafetyCommandType safety_command = SafetyCommandType::NONE;
  double safety_slow_down_percentage = 0.0;
  std::string safety_reason;

  for (const auto & candidate : winners) {
    const auto & rule = candidate.rule;
    plan.selected_rules.push_back(rule);
    signature_parts.push_back(rule.rule_id);
    for (const auto & key : rule.event_keys) {
      append_unique(plan.active_event_keys, key);
    }

    if (has_action(rule, ActionType::SAFETY_SYSTEM) &&
      rule.safety_command != SafetyCommandType::NONE)
    {
      const int current_priority = safety_command_priority(safety_command);
      const int next_priority = safety_command_priority(rule.safety_command);
      const bool prefer_slow_percent =
        next_priority == current_priority &&
        rule.safety_command == SafetyCommandType::SLOW_DOWN &&
        (!safety_active || rule.safety_slow_down_percentage < safety_slow_down_percentage);
      if (!safety_active || next_priority > current_priority || prefer_slow_percent) {
        safety_active = true;
        safety_command = rule.safety_command;
        safety_slow_down_percentage =
          rule.safety_command == SafetyCommandType::SLOW_DOWN ? rule.safety_slow_down_percentage : 0.0;
        safety_reason = rule.reason;
      }
    }

    if (has_action(rule, ActionType::SUPERVISOR)) {
      std::vector<std::string> target_modules = rule.nodemanager_modules;
      if (target_modules.empty()) {
        target_modules = rule.module_names;
      }
      if (target_modules.empty()) {
        target_modules.push_back(rule.combined ? "combined_fault" : "unknown");
      }
      for (const auto & module_name : target_modules) {
        auto & decision = nodemanager_by_module[module_name];
        decision.module_name = module_name;
        decision.level = rule.level;
        append_unique(decision.rule_ids, rule.rule_id);
        for (const auto & key : rule.event_keys) {
          append_unique(decision.fault_keys, key);
        }
        if (decision.reason.empty()) {
          decision.reason = rule.reason;
        } else if (decision.reason.find(rule.reason) == std::string::npos) {
          decision.reason += " | " + rule.reason;
        }
      }
    }

    if (rule.manual_takeover) {
      EventHumanTakeoverDecision takeover;
      takeover.fault_key = join_strings(rule.event_keys, ";");
      takeover.module_name = !rule.module_names.empty() ? rule.module_names.front() : "unknown";
      takeover.rule_id = rule.rule_id;
      takeover.level = rule.level;
      takeover.reason = rule.reason;
      plan.human_takeovers.push_back(std::move(takeover));
    }
  }

  if (safety_active) {
    plan.safety_update = SafetyCommandUpdate{
      true,
      safety_command,
      safety_slow_down_percentage,
      safety_reason};
  } else {
    plan.safety_update = SafetyCommandUpdate{
      false,
      SafetyCommandType::NONE,
      0.0,
      "All safety events recovered"};
  }

  for (auto & [_, decision] : nodemanager_by_module) {
    plan.nodemanager_decisions.push_back(std::move(decision));
  }

  std::sort(signature_parts.begin(), signature_parts.end());
  plan.signature = join_strings(signature_parts, "|");
  plan.plan_id = plan.signature.empty() ? "plan:none" : "plan:" + plan.signature;
  return plan;
}

EventArbitrationResult EventCodexArbiter::update(const std::vector<FaultInfo> & facts)
{
  return update(facts, rclcpp::Clock(RCL_ROS_TIME).now());
}

EventArbitrationResult EventCodexArbiter::update(
  const std::vector<FaultInfo> & facts,
  const rclcpp::Time & now)
{
  EventArbitrationResult result;
  std::map<std::string, FaultInfo> current_facts;
  for (const auto & fact : facts) {
    const std::string key = fact.fault_key.empty() ? fallback_fault_key(fact) : fact.fault_key;
    FaultInfo normalized = fact;
    normalized.fault_key = key;
    current_facts[key] = std::move(normalized);
  }

  for (const auto & [key, fact] : current_facts) {
    if (active_facts_.count(key) == 0) {
      result.edge_events.push_back(FaultEdgeEvent{fact, FaultEdgeType::TRIGGER});
    }
  }
  for (const auto & [key, fact] : active_facts_) {
    if (current_facts.count(key) == 0) {
      FaultInfo recover_fault = fact;
      recover_fault.reason = format_recover_reason(fact, key);
      result.edge_events.push_back(FaultEdgeEvent{recover_fault, FaultEdgeType::RECOVER});
    }
  }

  const auto candidates = build_candidates(current_facts, now);
  const auto winners = select_winners(candidates);
  result.plan = build_plan(winners);
  result.plan_changed = result.plan.signature != last_plan_signature_;

  previous_selected_rule_ids_.clear();
  for (const auto & rule : result.plan.selected_rules) {
    previous_selected_rule_ids_.insert(rule.rule_id);
  }
  active_facts_ = std::move(current_facts);
  last_plan_signature_ = result.plan.signature;
  return result;
}

}  // namespace nav2_monitor
