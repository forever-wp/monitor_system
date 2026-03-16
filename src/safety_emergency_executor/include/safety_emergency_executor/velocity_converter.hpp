#ifndef SAFETY_EMERGENCY_EXECUTOR__VELOCITY_CONVERTER_HPP_
#define SAFETY_EMERGENCY_EXECUTOR__VELOCITY_CONVERTER_HPP_

#include <optional>
#include <string>

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
  CommandFrame convert(const geometry_msgs::msg::Twist & msg) const;
  bool update_params_from_json(const std::string & payload, std::string * error = nullptr);
  void update_acc_from_topic(int acc_value);
  std::string to_json(const CommandFrame & frame) const;
  CommandFrame template_frame() const;

private:
  int effective_acc() const;

  Params params_{};
  std::optional<int> acc_override_{};
};

}  // namespace safety_emergency_executor

#endif  // SAFETY_EMERGENCY_EXECUTOR__VELOCITY_CONVERTER_HPP_
