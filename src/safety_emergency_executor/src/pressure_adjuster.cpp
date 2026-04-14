#include "safety_emergency_executor/pressure_adjuster.hpp"

namespace safety_emergency_executor
{

void PressureAdjuster::configure(rclcpp::Node & node)
{
  clock_ = node.get_clock();
  external_pressure_hold_s_ = node.declare_parameter<double>(
    "external_pressure_hold_s", 30.0);
  if (external_pressure_hold_s_ < 0.0) {
    external_pressure_hold_s_ = 0.0;
  }
  adjuster_.configure(node);
}

void PressureAdjuster::note_external_pressure_override(const rclcpp::Time & stamp)
{
  external_pressure_hold_initialized_ = true;
  if (external_pressure_hold_s_ <= 0.0) {
    external_pressure_hold_until_ = stamp;
    return;
  }

  external_pressure_hold_until_ =
    stamp + rclcpp::Duration::from_seconds(external_pressure_hold_s_);
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

bool PressureAdjuster::external_pressure_hold_active(const rclcpp::Time & now) const
{
  return external_pressure_hold_initialized_ && now < external_pressure_hold_until_;
}

void PressureAdjuster::apply(CommandFrame & frame)
{
  if (frame.press_from_embedded_fields) {
    return;
  }

  if (clock_ && external_pressure_hold_active(clock_->now())) {
    return;
  }

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
