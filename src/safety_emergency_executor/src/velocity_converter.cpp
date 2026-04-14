#include "safety_emergency_executor/velocity_converter.hpp"

#include <cmath>
#include <json/json.h>

namespace
{

constexpr double kEmbeddedFieldEpsilon = 1e-6;

}  // namespace

namespace safety_emergency_executor
{

void VelocityConverter::configure(rclcpp::Node & node)
{
  params_.acc = node.declare_parameter<int>("acc_", params_.acc);
  params_.press = node.declare_parameter<int>("press_", params_.press);
  params_.place = node.declare_parameter<int>("place_", params_.place);
  params_.ulock = node.declare_parameter<int>("ulock_", params_.ulock);
  extended_fields_enabled_["navigation"] =
    node.declare_parameter<bool>("cmd_vel_navigation_extended_fields_enabled", false);
  extended_fields_enabled_["miniapp"] =
    node.declare_parameter<bool>("cmd_vel_miniapp_extended_fields_enabled", true);
  extended_fields_enabled_["remote"] =
    node.declare_parameter<bool>("cmd_vel_remote_extended_fields_enabled", true);
  extended_fields_enabled_["other"] =
    node.declare_parameter<bool>("cmd_vel_other_extended_fields_enabled", true);
}

CommandFrame VelocityConverter::convert(
  const std::string & source,
  const geometry_msgs::msg::Twist & msg) const
{
  CommandFrame frame;
  frame.speed = std::round(msg.linear.x * 100.0) / 100.0;
  frame.angle = std::round(msg.angular.z * 100.0) / 100.0;
  frame.acc = effective_acc();
  frame.press = params_.press;
  frame.place = params_.place;
  frame.ulock = params_.ulock;

  if (source_uses_extended_fields(source) && has_embedded_command_fields(msg)) {
    frame.press = static_cast<int>(std::lround(msg.linear.y));
    frame.acc = static_cast<int>(std::lround(msg.linear.z));
    frame.place = static_cast<int>(std::lround(msg.angular.x));
    frame.ulock = static_cast<int>(std::lround(msg.angular.y));
    frame.press_from_embedded_fields = true;
  }

  return frame;
}

CommandFrame VelocityConverter::convert(const geometry_msgs::msg::Twist & msg) const
{
  return convert("navigation", msg);
}

void VelocityConverter::update_press_from_topic(int press_value)
{
  params_.press = press_value;
}

void VelocityConverter::update_acc_from_topic(int acc_value)
{
  acc_override_ = acc_value;
}

std::string VelocityConverter::to_json(const CommandFrame & frame) const
{
  Json::Value command_json;
  command_json["speed"] = frame.speed;
  command_json["angle"] = frame.angle;
  command_json["acc"] = frame.acc;
  command_json["press"] = frame.press;
  command_json["place"] = frame.place;
  command_json["ulock"] = frame.ulock;

  Json::StreamWriterBuilder writer_builder;
  writer_builder["indentation"] = "";
  return Json::writeString(writer_builder, command_json);
}

CommandFrame VelocityConverter::template_frame() const
{
  CommandFrame frame;
  frame.acc = effective_acc();
  frame.press = params_.press;
  frame.place = params_.place;
  frame.ulock = params_.ulock;
  return frame;
}

int VelocityConverter::effective_acc() const
{
  return acc_override_.value_or(params_.acc);
}

bool VelocityConverter::source_uses_extended_fields(const std::string & source) const
{
  const auto it = extended_fields_enabled_.find(source);
  return it != extended_fields_enabled_.end() && it->second;
}

bool VelocityConverter::has_embedded_command_fields(const geometry_msgs::msg::Twist & msg) const
{
  return std::abs(msg.linear.y) > kEmbeddedFieldEpsilon ||
         std::abs(msg.linear.z) > kEmbeddedFieldEpsilon ||
         std::abs(msg.angular.x) > kEmbeddedFieldEpsilon ||
         std::abs(msg.angular.y) > kEmbeddedFieldEpsilon;
}

}  // namespace safety_emergency_executor
