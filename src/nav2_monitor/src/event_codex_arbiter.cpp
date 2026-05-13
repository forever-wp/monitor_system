#include "nav2_monitor/event_codex_arbiter.hpp"

#include <algorithm>
#include <sstream>

namespace nav2_monitor
{

namespace
{
constexpr int kSingleRulePriority = 100;
constexpr int kCombinedRulePriority = 1000;
}

void EventCodexArbiter::set_combined_fault_rules(
  const std::vector<CombinedFaultRuleConfig> & rules)
{
  combined_rules_ = rules;
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

std::vector<EventCodexArbiter::Candidate> EventCodexArbiter::build_candidates(
  const std::map<std::string, FaultInfo> & active_facts) const
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
    bool matched = true;
    std::vector<FaultInfo> matched_facts;
    matched_facts.reserve(config.when_all_fault_keys.size());
    for (const auto & fault_key : config.when_all_fault_keys) {
      const auto it = active_facts.find(fault_key);
      if (it == active_facts.end()) {
        matched = false;
        break;
      }
      matched_facts.push_back(it->second);
    }
    if (!matched || matched_facts.empty()) {
      continue;
    }

    EventCodexSelectedRule rule;
    rule.rule_id = "combined|" + config.name;
    rule.rule_name = config.name;
    rule.event_keys = config.when_all_fault_keys;
    rule.level = config.level;
    rule.actions = config.actions;
    rule.safety_command = config.safety_command;
    rule.safety_slow_down_percentage = config.safety_slow_down_percentage;
    rule.reason = config.reason.empty() ? ("Combined fault triggered: " + config.name) : config.reason;
    rule.combined = true;
    rule.manual_takeover = config.manual_takeover;
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
    candidates.push_back(std::move(candidate));
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
      const std::string module_name =
        rule.combined ? std::string("combined_fault") :
        (!rule.module_names.empty() ? rule.module_names.front() : std::string("unknown"));
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

  const auto candidates = build_candidates(current_facts);
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
