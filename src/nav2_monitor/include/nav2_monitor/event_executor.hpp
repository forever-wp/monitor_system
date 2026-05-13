#ifndef NAV2_MONITOR__EVENT_EXECUTOR_HPP_
#define NAV2_MONITOR__EVENT_EXECUTOR_HPP_

#include <map>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "nav2_monitor/event_codex_arbiter.hpp"
#include "nav2_monitor/msg/safety_cmd.hpp"

namespace nav2_monitor
{

struct EventExecutionResult
{
  std::string plan_id;
  std::string plan_signature;
  std::optional<msg::SafetyCmd> safety_cmd;
  std::vector<std::string> nodemanager_json_payloads;
  std::vector<EventNodeManagerDecision> nodemanager_decisions;
};

class EventExecutor
{
public:
  EventExecutor() = default;

  void configure(
    rclcpp::Node * node,
    const std::string & safety_cmd_topic,
    const std::string & nodemanager_cmd_topic,
    double safety_cmd_republish_period_s,
    double nodemanager_cooldown_s);

  void update_timing(double safety_cmd_republish_period_s, double nodemanager_cooldown_s);
  EventExecutionResult execute(const EventExecutionPlan & plan, const rclcpp::Time & now);

private:
  struct NodeManagerPublishState
  {
    rclcpp::Time last_publish_time{0, 0, RCL_ROS_TIME};
    std::string last_signature;
  };

  static msg::SafetyCmd to_safety_cmd(const SafetyCommandUpdate & update);
  static std::string json_escape(const std::string & input);
  static std::string build_nodemanager_payload(const EventNodeManagerDecision & decision);
  static std::string nodemanager_signature(const EventNodeManagerDecision & decision);
  bool should_publish_safety(const SafetyCommandUpdate & update, const rclcpp::Time & now) const;
  bool should_publish_nodemanager(
    const EventNodeManagerDecision & decision,
    const std::string & signature,
    const rclcpp::Time & now) const;

  rclcpp::Node * node_{nullptr};
  rclcpp::Publisher<msg::SafetyCmd>::SharedPtr safety_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nodemanager_pub_;
  double safety_cmd_republish_period_s_{0.2};
  double nodemanager_cooldown_s_{5.0};

  bool safety_state_known_{false};
  SafetyCommandUpdate last_safety_update_;
  rclcpp::Time last_safety_publish_time_{0, 0, RCL_ROS_TIME};
  std::map<std::string, NodeManagerPublishState> nodemanager_publish_state_;
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__EVENT_EXECUTOR_HPP_
