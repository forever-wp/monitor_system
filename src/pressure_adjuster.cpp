#include "safety_emergency_executor/pressure_adjuster.hpp"

namespace safety_emergency_executor
{

void PressureAdjuster::configure(rclcpp::Node & node)
{
  adjuster_.configure(node);
}

void PressureAdjuster::on_wheel_odom(const nav_msgs::msg::Odometry::SharedPtr & msg)
{
  wheel_odom_ = msg;
}

void PressureAdjuster::on_loc_odom(const nav_msgs::msg::Odometry::SharedPtr & msg)
{
  loc_odom_ = msg;
}

void PressureAdjuster::on_imu(const sensor_msgs::msg::Imu::SharedPtr & msg)
{
  double wheel_speed = 0.0;
  if (wheel_odom_) {
    wheel_speed = wheel_odom_->twist.twist.linear.x;
  }
  adjuster_.updateImu(*msg, wheel_speed);
}

void PressureAdjuster::apply(CommandFrame & frame)
{
  const std::string fallback_mode = adjuster_.getFallbackMode();
  if (fallback_mode == "disabled") {
    return;
  }

  int adjusted_press = frame.press;
  if (adjuster_.isLocalizationEnabled() && wheel_odom_ && loc_odom_) {
    adjuster_.update(*wheel_odom_, *loc_odom_, frame.press, adjusted_press);
    frame.press = adjusted_press;
    return;
  }

  if (adjuster_.isImuEnabled() && wheel_odom_) {
    adjuster_.updateImuOnly(*wheel_odom_, frame.press, adjusted_press);
    frame.press = adjusted_press;
  }
}

}  // namespace safety_emergency_executor
