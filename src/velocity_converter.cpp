#include "safety_emergency_executor/velocity_converter.hpp"

#include <json/json.h>
#include <cmath>

namespace safety_emergency_executor
{

void VelocityConverter::configure(rclcpp::Node & node)
{
  params_.acc = node.declare_parameter<int>("acc_", params_.acc);
  params_.press = node.declare_parameter<int>("press_", params_.press);
  params_.place = node.declare_parameter<int>("place_", params_.place);
  params_.ulock = node.declare_parameter<int>("ulock_", params_.ulock);
}

CommandFrame VelocityConverter::convert(const geometry_msgs::msg::Twist & msg) const
{
  CommandFrame frame;
  frame.speed = std::round(msg.linear.x * 100.0) / 100.0;
  frame.angle = std::round(msg.angular.z * 100.0) / 100.0;
  frame.acc = effective_acc();
  frame.press = params_.press;
  frame.place = params_.place;
  frame.ulock = params_.ulock;
  return frame;
}

bool VelocityConverter::update_params_from_json(const std::string & payload, std::string * error)
{
  Json::CharReaderBuilder builder;
  Json::Value root;
  std::string parse_error;
  std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
  if (!reader->parse(payload.c_str(), payload.c_str() + payload.size(), &root, &parse_error)) {
    if (error != nullptr) {
      *error = parse_error;
    }
    return false;
  }

  if (root.isMember("acc")) {
    params_.acc = root["acc"].asInt();
  }
  if (root.isMember("press")) {
    params_.press = root["press"].asInt();
  }
  if (root.isMember("place")) {
    params_.place = root["place"].asInt();
  }
  if (root.isMember("ulock")) {
    params_.ulock = root["ulock"].asInt();
  }

  return true;
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

}  // namespace safety_emergency_executor
