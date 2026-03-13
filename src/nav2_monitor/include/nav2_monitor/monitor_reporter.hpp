#ifndef NAV2_MONITOR__MONITOR_REPORTER_HPP_
#define NAV2_MONITOR__MONITOR_REPORTER_HPP_

#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "nav2_monitor/msg/monitor_status.hpp"
#include "nav2_monitor/msg/fault_event.hpp"
#include "nav2_monitor/msg/safety_cmd.hpp"

namespace nav2_monitor
{

class MonitorReporter
{
public:
  MonitorReporter() = default;

  void configure(rclcpp::Node * node);
  void publish_heartbeat(const msg::MonitorStatus & status, const rclcpp::Time & now) const;
  void cache_supervisor_json(const std::string & json_payload, const rclcpp::Time & now);
  void cache_safety_cmd(const msg::SafetyCmd & msg, const rclcpp::Time & now);
  void publish_fault_event_json(const msg::FaultEvent & event, const rclcpp::Time & now) const;

private:
  struct SupervisorCache
  {
    bool valid{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    std::string module_name;
    size_t nodes_to_restart_count{0};
    std::string raw_json;
  };

  struct SafetyCache
  {
    bool valid{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    uint8_t action{0};
    float slow_down_percentage{0.0F};
    std::string reason;
  };

  static std::string json_escape(const std::string & input);
  static std::string fault_level_to_string(uint8_t level);
  static std::string action_to_string(uint8_t action);
  static std::string edge_to_string(uint8_t edge);
  static std::string safety_action_to_string(uint8_t action);
  static std::string classify_fault_type(const msg::FaultEvent & event);
  static std::string to_stamp_string(const builtin_interfaces::msg::Time & stamp);
  static std::string extract_json_string_field(const std::string & json, const std::string & field);
  static size_t extract_json_array_count(const std::string & json, const std::string & field);

  rclcpp::Node * node_{nullptr};
  double cmd_correlation_window_s_{2.0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  SupervisorCache last_supervisor_cmd_;
  SafetyCache last_safety_cmd_;
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__MONITOR_REPORTER_HPP_
