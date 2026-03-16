#ifndef SAFETY_EMERGENCY_EXECUTOR__PRESSURE_ADJUSTER_HPP_
#define SAFETY_EMERGENCY_EXECUTOR__PRESSURE_ADJUSTER_HPP_

#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>

#include "safety_emergency_executor/command_frame.hpp"
#include "safety_emergency_executor/linear_pressure_adjuster.hpp"

namespace safety_emergency_executor
{

class PressureAdjuster
{
public:
  void configure(rclcpp::Node & node);

  void on_wheel_odom(const nav_msgs::msg::Odometry::SharedPtr & msg);
  void on_loc_odom(const nav_msgs::msg::Odometry::SharedPtr & msg);
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr & msg);

  void apply(CommandFrame & frame);

private:
  LinearPressureAdjuster adjuster_{};
  nav_msgs::msg::Odometry::SharedPtr wheel_odom_{};
  nav_msgs::msg::Odometry::SharedPtr loc_odom_{};
};

}  // namespace safety_emergency_executor

#endif  // SAFETY_EMERGENCY_EXECUTOR__PRESSURE_ADJUSTER_HPP_
