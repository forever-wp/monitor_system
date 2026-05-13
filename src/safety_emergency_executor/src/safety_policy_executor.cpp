#include "safety_emergency_executor/safety_policy_executor.hpp"

#include <algorithm>

namespace safety_emergency_executor
{

void SafetyPolicyExecutor::configure(rclcpp::Node & node)
{
  slow_down_percentage_ = node.declare_parameter<double>(
    "slow_down_percentage", slow_down_percentage_);
  brake_strong_speed_ = node.declare_parameter<double>("brake_strong_speed", brake_strong_speed_);
  brake_medium_speed_ = node.declare_parameter<double>("brake_medium_speed", brake_medium_speed_);
  brake_strong_repeat_ = node.declare_parameter<int>("brake_strong_repeat", brake_strong_repeat_);
  brake_medium_repeat_ = node.declare_parameter<int>("brake_medium_repeat", brake_medium_repeat_);
  brake_zero_repeat_ = node.declare_parameter<int>("brake_zero_repeat", brake_zero_repeat_);
  brake_interval_ms_ = node.declare_parameter<int>("brake_interval_ms", brake_interval_ms_);
}

void SafetyPolicyExecutor::reset()
{
  forwarding_enabled_ = true;
  speed_limit_active_ = false;
  speed_limit_percentage_ = true;
  speed_limit_value_ = 0.0;
}

bool SafetyPolicyExecutor::allow_forward() const
{
  return forwarding_enabled_;
}

bool SafetyPolicyExecutor::apply(CommandFrame & frame) const
{
  if (!forwarding_enabled_) {
    return false;
  }

  if (!speed_limit_active_) {
    return true;
  }

  if (speed_limit_percentage_) {
    const double factor = speed_limit_value_ / 100.0;
    frame.speed *= factor;
    frame.angle *= factor;
  } else {
    frame.speed = std::clamp(frame.speed, -speed_limit_value_, speed_limit_value_);
    frame.angle = std::clamp(frame.angle, -speed_limit_value_, speed_limit_value_);
  }

  return true;
}

std::vector<CommandFrame> SafetyPolicyExecutor::on_safety_cmd(
  const nav2_monitor::msg::SafetyCmd & msg,
  const CommandFrame & template_frame)
{
  std::vector<CommandFrame> emergency_sequence;
  const auto action = msg.action;

  if (action == nav2_monitor::msg::SafetyCmd::RESUME) {
    forwarding_enabled_ = true;
    speed_limit_percentage_ = true;
    speed_limit_value_ = 0.0;
    speed_limit_active_ = false;
    return emergency_sequence;
  }

  if (action == nav2_monitor::msg::SafetyCmd::SLOW_DOWN) {
    const double requested_percentage =
      msg.slow_down_percentage > 0.0F ? msg.slow_down_percentage : slow_down_percentage_;
    forwarding_enabled_ = true;
    speed_limit_percentage_ = true;
    speed_limit_value_ = std::clamp(requested_percentage, 0.0, 100.0);
    speed_limit_active_ = true;
    return emergency_sequence;
  }

  if (action == nav2_monitor::msg::SafetyCmd::SOFT_STOP) {
    forwarding_enabled_ = true;
    speed_limit_percentage_ = true;
    speed_limit_value_ = 0.0;
    speed_limit_active_ = true;
    CommandFrame frame = template_frame;
    frame.speed = 0.0;
    frame.angle = 0.0;
    emergency_sequence.push_back(frame);
    return emergency_sequence;
  }

  if (action == nav2_monitor::msg::SafetyCmd::EMERGENCY_STOP) {
    forwarding_enabled_ = false;
    speed_limit_percentage_ = true;
    speed_limit_value_ = 0.0;
    speed_limit_active_ = true;

    auto append_speed = [&](double speed, int repeat) {
        for (int idx = 0; idx < std::max(0, repeat); ++idx) {
          CommandFrame frame = template_frame;
          frame.speed = speed;
          frame.angle = 0.0;
          frame.press = 1000;
          frame.acc = 1000;
          frame.place = -1;
          frame.ulock = -1;
          emergency_sequence.push_back(frame);
        }
      };

    append_speed(brake_strong_speed_, brake_strong_repeat_);
    append_speed(brake_medium_speed_, brake_medium_repeat_);
    append_speed(0.0, brake_zero_repeat_);
    return emergency_sequence;
  }

  return emergency_sequence;
}

int SafetyPolicyExecutor::brake_interval_ms() const
{
  return std::max(1, brake_interval_ms_);
}

}  // namespace safety_emergency_executor
