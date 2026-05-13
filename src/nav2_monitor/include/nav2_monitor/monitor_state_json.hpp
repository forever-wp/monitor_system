#ifndef NAV2_MONITOR__MONITOR_STATE_JSON_HPP_
#define NAV2_MONITOR__MONITOR_STATE_JSON_HPP_

#include <sstream>
#include <string>
#include <vector>

#include "nav2_monitor/fault_detector.hpp"

namespace nav2_monitor
{

inline std::string monitor_json_escape(const std::string & input)
{
  std::ostringstream oss;
  for (const auto ch : input) {
    switch (ch) {
      case '"': oss << "\\\""; break;
      case '\\': oss << "\\\\"; break;
      case '\n': oss << "\\n"; break;
      case '\r': oss << "\\r"; break;
      case '\t': oss << "\\t"; break;
      default: oss << ch; break;
    }
  }
  return oss.str();
}

inline std::string fault_level_to_state_string(FaultLevel level)
{
  switch (level) {
    case FaultLevel::WARNING:
      return "WARNING";
    case FaultLevel::ERROR:
      return "ERROR";
    case FaultLevel::CRITICAL:
      return "CRITICAL";
    case FaultLevel::NORMAL:
    default:
      return "NORMAL";
  }
}

inline std::string action_to_state_string(ActionType action)
{
  switch (action) {
    case ActionType::SUPERVISOR:
      return "NODEMANAGER";
    case ActionType::SAFETY_SYSTEM:
      return "SAFETY_SYSTEM";
    case ActionType::NONE:
    default:
      return "NONE";
  }
}

inline std::string safety_command_to_state_string(SafetyCommandType command)
{
  switch (command) {
    case SafetyCommandType::SLOW_DOWN:
      return "SLOW_DOWN";
    case SafetyCommandType::SOFT_STOP:
      return "SOFT_STOP";
    case SafetyCommandType::EMERGENCY_STOP:
      return "EMERGENCY_STOP";
    case SafetyCommandType::NONE:
    default:
      return "NONE";
  }
}

inline std::string faults_to_json_array(const std::vector<FaultInfo> & faults)
{
  std::ostringstream oss;
  oss << '[';
  for (size_t i = 0; i < faults.size(); ++i) {
    if (i > 0) {
      oss << ',';
    }
    const auto & fault = faults[i];
    oss << '{'
        << "\"fault_key\":\"" << monitor_json_escape(fault.fault_key) << "\","
        << "\"module_name\":\"" << monitor_json_escape(fault.module_name) << "\","
        << "\"level\":\"" << fault_level_to_state_string(fault.level) << "\","
        << "\"type\":\"" << monitor_json_escape(fault.fault_type) << "\","
        << "\"fault_type\":\"" << monitor_json_escape(fault.fault_type) << "\","
        << "\"fault_model\":\"" << monitor_json_escape(fault.fault_model) << "\","
        << "\"fault_name\":\"" << monitor_json_escape(fault.fault_name) << "\","
        << "\"action\":\"" << action_to_state_string(fault.action) << "\","
        << "\"safety_command\":\"" << safety_command_to_state_string(fault.safety_command) << "\","
        << "\"safety_slow_down_percentage\":" << fault.safety_slow_down_percentage << ','
        << "\"reason\":\"" << monitor_json_escape(fault.reason) << "\""
        << '}';
  }
  oss << ']';
  return oss.str();
}

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__MONITOR_STATE_JSON_HPP_
