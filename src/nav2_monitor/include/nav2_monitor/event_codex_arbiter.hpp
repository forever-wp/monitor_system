#ifndef NAV2_MONITOR__EVENT_CODEX_ARBITER_HPP_
#define NAV2_MONITOR__EVENT_CODEX_ARBITER_HPP_

#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "nav2_monitor/event_types.hpp"
#include "nav2_monitor/fault_detector.hpp"

namespace nav2_monitor
{

struct EventCodexSelectedRule
{
  std::string rule_id;
  std::string rule_name;
  std::vector<std::string> event_keys;
  std::vector<std::string> module_names;
  FaultLevel level{FaultLevel::NORMAL};
  std::vector<ActionType> actions;
  SafetyCommandType safety_command{SafetyCommandType::NONE};
  double safety_slow_down_percentage{0.0};
  std::string reason;
  bool combined{false};
  bool manual_takeover{false};
};

struct EventNodeManagerDecision
{
  std::string module_name;
  std::vector<std::string> fault_keys;
  std::vector<std::string> rule_ids;
  FaultLevel level{FaultLevel::NORMAL};
  std::string reason;
};

struct EventHumanTakeoverDecision
{
  std::string fault_key;
  std::string module_name;
  std::string rule_id;
  FaultLevel level{FaultLevel::NORMAL};
  std::string reason;
};

struct EventExecutionPlan
{
  std::string plan_id;
  std::string signature;
  std::vector<EventCodexSelectedRule> selected_rules;
  std::optional<SafetyCommandUpdate> safety_update;
  std::vector<EventNodeManagerDecision> nodemanager_decisions;
  std::vector<EventHumanTakeoverDecision> human_takeovers;
};

struct EventArbitrationResult
{
  std::vector<FaultEdgeEvent> edge_events;
  EventExecutionPlan plan;
  bool plan_changed{false};
};

class EventCodexArbiter
{
public:
  EventCodexArbiter() = default;

  void set_combined_fault_rules(const std::vector<CombinedFaultRuleConfig> & rules);
  EventArbitrationResult update(const std::vector<FaultInfo> & facts);

private:
  struct Candidate
  {
    EventCodexSelectedRule rule;
    std::vector<FaultInfo> facts;
    int priority{0};
    int execution_level{0};
    bool was_active{false};
  };

  static std::string fallback_fault_key(const FaultInfo & fault);
  static std::string format_recover_reason(const FaultInfo & fault, const std::string & fault_key);
  static bool is_vehicle_state_judge_fault(const FaultInfo & fault);
  static bool has_action(const EventCodexSelectedRule & rule, ActionType action);
  static int fault_level_priority(FaultLevel level);
  static int safety_command_priority(SafetyCommandType command);
  static int execution_level(const Candidate & candidate);
  static std::string join_strings(const std::vector<std::string> & values, const std::string & sep);
  static void append_unique(std::vector<std::string> & values, const std::string & value);

  std::vector<Candidate> build_candidates(
    const std::map<std::string, FaultInfo> & active_facts) const;
  std::vector<Candidate> select_winners(std::vector<Candidate> candidates) const;
  EventExecutionPlan build_plan(const std::vector<Candidate> & winners) const;

  std::vector<CombinedFaultRuleConfig> combined_rules_;
  std::map<std::string, FaultInfo> active_facts_;
  std::set<std::string> previous_selected_rule_ids_;
  std::string last_plan_signature_;
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__EVENT_CODEX_ARBITER_HPP_
