#include "nav2_monitor/fault_state_coordinator.hpp"

#include <cmath>

#include <nav2_monitor/msg/safety_cmd.hpp>

namespace nav2_monitor
{

void FaultStateCoordinator::configure(rclcpp::Node * node, const std::string & safety_cmd_topic)
{
  node_ = node;
  if (node_ != nullptr) {
    auto safety_pub = node_->create_publisher<nav2_monitor::msg::SafetyCmd>(safety_cmd_topic, 10);
    publish_safety_update_fn_ = [safety_pub](const SafetyCommandUpdate & safety_update) {
      nav2_monitor::msg::SafetyCmd msg;
      if (safety_update.active) {
        switch (safety_update.command) {
          case SafetyCommandType::SLOW_DOWN:
            msg.action = nav2_monitor::msg::SafetyCmd::SLOW_DOWN;
            break;
          case SafetyCommandType::SOFT_STOP:
            msg.action = nav2_monitor::msg::SafetyCmd::SOFT_STOP;
            break;
          case SafetyCommandType::EMERGENCY_STOP:
            msg.action = nav2_monitor::msg::SafetyCmd::EMERGENCY_STOP;
            break;
          case SafetyCommandType::NONE:
          default:
            msg.action = nav2_monitor::msg::SafetyCmd::RESUME;
            break;
        }
        msg.slow_down_percentage = static_cast<float>(safety_update.slow_down_percentage);
        msg.reason = safety_update.reason;
      } else {
        msg.action = nav2_monitor::msg::SafetyCmd::RESUME;
        msg.slow_down_percentage = 0.0F;
        msg.reason = safety_update.reason;
      }
      safety_pub->publish(msg);
    };
  }
}

std::string FaultStateCoordinator::fallback_fault_key(const FaultInfo & fault)
{
  return fault.module_name + "|" + std::to_string(static_cast<int>(fault.action)) + "|" + fault.reason;
}

std::string FaultStateCoordinator::format_recover_reason(
  const FaultInfo & fault, const std::string & fault_key)
{
  return "RECOVER fault_key=" + fault_key + "; previous_reason=" + fault.reason;
}

int FaultStateCoordinator::safety_command_priority(SafetyCommandType command)
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

FaultStateUpdate FaultStateCoordinator::update(const std::vector<FaultInfo> & faults)
{
  FaultStateUpdate update_result;
  std::map<std::string, FaultInfo> current_faults;

  bool next_safety_active = false;
  SafetyCommandType next_safety_command = SafetyCommandType::NONE;
  double next_safety_slow_down_percentage = 0.0;
  std::string next_safety_reason;

  for (const auto & fault : faults) {
    const std::string key = fault.fault_key.empty() ? fallback_fault_key(fault) : fault.fault_key;
    current_faults[key] = fault;

    if (fault.action == ActionType::SAFETY_SYSTEM && fault.safety_command != SafetyCommandType::NONE) {
      const int current_priority = safety_command_priority(next_safety_command);
      const int fault_priority = safety_command_priority(fault.safety_command);
      if (
        !next_safety_active || fault_priority > current_priority ||
        (fault_priority == current_priority && fault.safety_command == SafetyCommandType::SLOW_DOWN &&
        fault.safety_slow_down_percentage < next_safety_slow_down_percentage))
      {
        next_safety_active = true;
        next_safety_command = fault.safety_command;
        next_safety_slow_down_percentage =
          fault.safety_command == SafetyCommandType::SLOW_DOWN ? fault.safety_slow_down_percentage : 0.0;
        next_safety_reason = fault.reason;
      }
    }
  }

  for (const auto & [key, fault] : current_faults) {
    if (active_faults_.count(key) == 0) {
      update_result.edge_events.push_back(FaultEdgeEvent{fault, FaultEdgeType::TRIGGER});
    }
  }

  for (const auto & [key, fault] : active_faults_) {
    if (current_faults.count(key) == 0) {
      FaultInfo recover_fault = fault;
      recover_fault.reason = format_recover_reason(fault, key);
      update_result.edge_events.push_back(FaultEdgeEvent{recover_fault, FaultEdgeType::RECOVER});
    }
  }

  const bool same_safety_percentage =
    next_safety_command != SafetyCommandType::SLOW_DOWN ||
    std::fabs(next_safety_slow_down_percentage - safety_slow_down_percentage_) < 1e-6;
  const bool same_safety_state =
    next_safety_active == safety_active_ &&
    next_safety_command == safety_command_ &&
    same_safety_percentage;

  if (!same_safety_state) {
    update_result.safety_update = SafetyCommandUpdate{
      next_safety_active,
      next_safety_command,
      next_safety_slow_down_percentage,
      next_safety_active ? next_safety_reason : "All safety faults recovered"};
    if (publish_safety_update_fn_) {
      publish_safety_update_fn_(*update_result.safety_update);
    }
  }

  active_faults_ = std::move(current_faults);
  safety_active_ = next_safety_active;
  safety_command_ = next_safety_command;
  safety_slow_down_percentage_ = next_safety_slow_down_percentage;

  return update_result;
}

}  // namespace nav2_monitor
