#ifndef SAFETY_EMERGENCY_EXECUTOR__SAFETY_POLICY_EXECUTOR_HPP_
#define SAFETY_EMERGENCY_EXECUTOR__SAFETY_POLICY_EXECUTOR_HPP_

#include <vector>

#include <nav2_monitor/msg/safety_cmd.hpp>
#include <rclcpp/rclcpp.hpp>

#include "safety_emergency_executor/command_frame.hpp"

namespace safety_emergency_executor
{

class SafetyPolicyExecutor
{
public:
  void configure(rclcpp::Node & node);
  void reset();

  bool allow_forward() const;
  bool apply(CommandFrame & frame) const;
  std::vector<CommandFrame> on_safety_cmd(
    const nav2_monitor::msg::SafetyCmd & msg,
    const CommandFrame & template_frame);
  int brake_interval_ms() const;

private:
  bool forwarding_enabled_{true};
  bool speed_limit_active_{false};
  bool speed_limit_percentage_{true};
  double speed_limit_value_{0.0};
  double slow_down_percentage_{50.0};
  double brake_strong_speed_{-1.0};
  double brake_medium_speed_{-0.5};
  int brake_strong_repeat_{2};
  int brake_medium_repeat_{2};
  int brake_zero_repeat_{4};
  int brake_interval_ms_{20};
};

}  // namespace safety_emergency_executor

#endif  // SAFETY_EMERGENCY_EXECUTOR__SAFETY_POLICY_EXECUTOR_HPP_
