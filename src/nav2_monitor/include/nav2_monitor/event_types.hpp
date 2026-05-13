#ifndef NAV2_MONITOR__EVENT_TYPES_HPP_
#define NAV2_MONITOR__EVENT_TYPES_HPP_

#include <optional>
#include <string>
#include <vector>

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

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__EVENT_TYPES_HPP_
