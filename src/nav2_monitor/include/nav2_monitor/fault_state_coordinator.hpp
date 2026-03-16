#ifndef NAV2_MONITOR__FAULT_STATE_COORDINATOR_HPP_
#define NAV2_MONITOR__FAULT_STATE_COORDINATOR_HPP_

#include <optional>
#include <string>
#include <vector>
#include <map>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include "nav2_monitor/fault_detector.hpp"

namespace nav2_monitor
{

enum class FaultEdgeType
{
  NONE = 0,
  TRIGGER = 1,
  RECOVER = 2
};

struct FaultEdgeEvent
{
  FaultInfo fault;
  FaultEdgeType edge{FaultEdgeType::NONE};
};

struct SafetyCommandUpdate
{
  bool active{false};
  SafetyCommandType command{SafetyCommandType::NONE};
  double slow_down_percentage{0.0};
  std::string reason;
};

struct FaultStateUpdate
{
  std::vector<FaultEdgeEvent> edge_events;
  std::optional<SafetyCommandUpdate> safety_update;
};

class FaultStateCoordinator
{
public:
  FaultStateCoordinator() = default;

  void configure(rclcpp::Node * node, const std::string & safety_cmd_topic = "/safety_system/cmd");
  FaultStateUpdate update(const std::vector<FaultInfo> & faults);

private:
  static std::string fallback_fault_key(const FaultInfo & fault);
  static std::string format_recover_reason(const FaultInfo & fault, const std::string & fault_key);
  static int safety_command_priority(SafetyCommandType command);

  rclcpp::Node * node_{nullptr};
  std::function<void(const SafetyCommandUpdate &)> publish_safety_update_fn_;
  std::map<std::string, FaultInfo> active_faults_;
  bool safety_active_{false};
  SafetyCommandType safety_command_{SafetyCommandType::NONE};
  double safety_slow_down_percentage_{0.0};
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__FAULT_STATE_COORDINATOR_HPP_
