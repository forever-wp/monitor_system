#ifndef SAFETY_EMERGENCY_EXECUTOR__VELOCITY_CONVERTER_HPP_
#define SAFETY_EMERGENCY_EXECUTOR__VELOCITY_CONVERTER_HPP_

#include <optional>
#include <string>
#include <unordered_map>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "safety_emergency_executor/command_frame.hpp"

namespace safety_emergency_executor
{

class VelocityConverter
{
public:
  struct Params
  {
    int acc{2000};
    int press{1400};
    int place{-1};
    int ulock{-1};
  };

  VelocityConverter() = default;

  void configure(rclcpp::Node & node);
  CommandFrame convert(const std::string & source, const geometry_msgs::msg::Twist & msg) const;
  CommandFrame convert(const geometry_msgs::msg::Twist & msg) const;
  void update_press_from_topic(int press_value);
  void update_acc_from_topic(int acc_value);
  std::string to_json(const CommandFrame & frame) const;
  CommandFrame template_frame() const;

private:
  int effective_acc() const;
  bool source_uses_extended_fields(const std::string & source) const;
  bool has_embedded_command_fields(const geometry_msgs::msg::Twist & msg) const;

  Params params_{};
  std::optional<int> acc_override_{};
  std::unordered_map<std::string, bool> extended_fields_enabled_{
    {"navigation", false},
    {"miniapp", true},
    {"remote", true},
    {"other", true},
  };
};

}  // namespace safety_emergency_executor

#endif  // SAFETY_EMERGENCY_EXECUTOR__VELOCITY_CONVERTER_HPP_
