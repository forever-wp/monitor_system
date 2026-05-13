#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_monitor/msg/safety_cmd.hpp>
#include <rclcpp/rclcpp.hpp>

#include "safety_emergency_executor/pressure_adjuster.hpp"
#include "safety_emergency_executor/safety_policy_executor.hpp"
#include "safety_emergency_executor/velocity_converter.hpp"

namespace
{

class SafetyExecutorComponentTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    std::filesystem::create_directories("/tmp/ros_logs");
    std::filesystem::create_directories("/tmp/.ros/log");
    ::setenv("HOME", "/tmp", 1);
    ::setenv("ROS_HOME", "/tmp/.ros", 1);
    ::setenv("ROS_LOG_DIR", "/tmp/ros_logs", 1);
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

nav_msgs::msg::Odometry::SharedPtr make_odom(double linear_x, double angular_z = 0.0)
{
  auto msg = std::make_shared<nav_msgs::msg::Odometry>();
  msg->twist.twist.linear.x = linear_x;
  msg->twist.twist.angular.z = angular_z;
  return msg;
}

TEST_F(SafetyExecutorComponentTest, VelocityConverterPressureTopicUpdatesBaselinePress)
{
  auto node = std::make_shared<rclcpp::Node>("velocity_converter_pressure_override_test");
  safety_emergency_executor::VelocityConverter converter;
  converter.configure(*node);

  converter.update_press_from_topic(1100);
  const auto frame = converter.template_frame();

  EXPECT_EQ(frame.press, 1100);
  EXPECT_EQ(frame.acc, 2000);
  EXPECT_EQ(frame.place, -1);
  EXPECT_EQ(frame.ulock, -1);
}

TEST_F(SafetyExecutorComponentTest, VelocityConverterAccTopicOverrideKeepsPressureBaseline)
{
  auto node = std::make_shared<rclcpp::Node>("velocity_converter_acc_override_test");
  safety_emergency_executor::VelocityConverter converter;
  converter.configure(*node);

  converter.update_press_from_topic(1100);
  converter.update_acc_from_topic(3200);
  const auto frame = converter.template_frame();

  EXPECT_EQ(frame.press, 1100);
  EXPECT_EQ(frame.acc, 3200);
}

TEST_F(SafetyExecutorComponentTest, VelocityConverterUsesUpdatedBaselineWhenAuxFieldsAreUnused)
{
  auto node = std::make_shared<rclcpp::Node>("velocity_converter_aux_field_guard_test");
  safety_emergency_executor::VelocityConverter converter;
  converter.configure(*node);

  converter.update_press_from_topic(950);
  converter.update_acc_from_topic(1500);

  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0.42;
  msg.angular.z = 0.11;
  const auto frame = converter.convert("remote", msg);

  EXPECT_DOUBLE_EQ(frame.speed, 0.42);
  EXPECT_DOUBLE_EQ(frame.angle, 0.11);
  EXPECT_EQ(frame.acc, 1500);
  EXPECT_EQ(frame.press, 950);
  EXPECT_EQ(frame.place, -1);
  EXPECT_EQ(frame.ulock, -1);
}

TEST_F(SafetyExecutorComponentTest, PressureAdjusterDisabledModeLeavesPressureUnchanged)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("auto_pressure.fallback_mode", "disabled")
    });
  auto node = std::make_shared<rclcpp::Node>("pressure_adjuster_test", options);

  safety_emergency_executor::PressureAdjuster adjuster;
  adjuster.configure(*node);

  safety_emergency_executor::CommandFrame frame;
  frame.press = 1234;
  adjuster.apply(frame);

  EXPECT_EQ(frame.press, 1234);
}

TEST_F(SafetyExecutorComponentTest, ExternalPressureHoldWindowBypassesAutomaticAdjustment)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("external_pressure_hold_s", 30.0)
    });
  auto node = std::make_shared<rclcpp::Node>("pressure_adjuster_hold_window_test", options);

  safety_emergency_executor::PressureAdjuster adjuster;
  adjuster.configure(*node);
  adjuster.on_wheel_odom(make_odom(1.0));
  adjuster.on_loc_odom(make_odom(0.0));
  adjuster.note_external_pressure_override(node->get_clock()->now());

  safety_emergency_executor::CommandFrame frame;
  frame.press = 1100;
  adjuster.apply(frame);

  EXPECT_EQ(frame.press, 1100);
}

TEST_F(SafetyExecutorComponentTest, AutomaticAdjustmentResumesAfterHoldWindowExpires)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("external_pressure_hold_s", 0.01)
    });
  auto node = std::make_shared<rclcpp::Node>("pressure_adjuster_resume_test", options);

  safety_emergency_executor::PressureAdjuster adjuster;
  adjuster.configure(*node);
  adjuster.on_wheel_odom(make_odom(1.0));
  adjuster.on_loc_odom(make_odom(0.0));
  adjuster.note_external_pressure_override(node->get_clock()->now());

  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  safety_emergency_executor::CommandFrame frame;
  frame.press = 1100;
  adjuster.apply(frame);

  EXPECT_NE(frame.press, 1100);
}

TEST_F(SafetyExecutorComponentTest, SafetyPolicyTransitionsPreserveExpectedSemantics)
{
  auto node = std::make_shared<rclcpp::Node>("safety_policy_test");
  safety_emergency_executor::SafetyPolicyExecutor policy;
  policy.configure(*node);

  safety_emergency_executor::CommandFrame template_frame;
  template_frame.acc = 2000;
  template_frame.press = 1400;
  template_frame.place = -1;
  template_frame.ulock = -1;

  nav2_monitor::msg::SafetyCmd slow_msg;
  slow_msg.action = nav2_monitor::msg::SafetyCmd::SLOW_DOWN;
  slow_msg.slow_down_percentage = 50.0F;
  EXPECT_TRUE(policy.on_safety_cmd(slow_msg, template_frame).empty());

  safety_emergency_executor::CommandFrame slow_frame;
  slow_frame.speed = 1.0;
  slow_frame.angle = 0.4;
  ASSERT_TRUE(policy.apply(slow_frame));
  EXPECT_DOUBLE_EQ(slow_frame.speed, 0.5);
  EXPECT_DOUBLE_EQ(slow_frame.angle, 0.2);

  nav2_monitor::msg::SafetyCmd stop_msg;
  stop_msg.action = nav2_monitor::msg::SafetyCmd::SOFT_STOP;
  const auto stop_sequence = policy.on_safety_cmd(stop_msg, template_frame);
  ASSERT_EQ(stop_sequence.size(), 1u);
  EXPECT_DOUBLE_EQ(stop_sequence.front().speed, 0.0);
  EXPECT_DOUBLE_EQ(stop_sequence.front().angle, 0.0);
  EXPECT_EQ(stop_sequence.front().acc, template_frame.acc);
  EXPECT_EQ(stop_sequence.front().press, template_frame.press);

  safety_emergency_executor::CommandFrame stop_frame;
  stop_frame.speed = 0.8;
  stop_frame.angle = 0.3;
  ASSERT_TRUE(policy.apply(stop_frame));
  EXPECT_DOUBLE_EQ(stop_frame.speed, 0.0);
  EXPECT_DOUBLE_EQ(stop_frame.angle, 0.0);

  nav2_monitor::msg::SafetyCmd estop_msg;
  estop_msg.action = nav2_monitor::msg::SafetyCmd::EMERGENCY_STOP;
  const auto brake_sequence = policy.on_safety_cmd(estop_msg, template_frame);
  EXPECT_FALSE(policy.allow_forward());
  EXPECT_EQ(brake_sequence.size(), 8u);

  safety_emergency_executor::CommandFrame blocked_frame;
  blocked_frame.speed = 0.8;
  blocked_frame.angle = 0.3;
  EXPECT_FALSE(policy.apply(blocked_frame));

  nav2_monitor::msg::SafetyCmd resume_msg;
  resume_msg.action = nav2_monitor::msg::SafetyCmd::RESUME;
  EXPECT_TRUE(policy.on_safety_cmd(resume_msg, template_frame).empty());
  EXPECT_TRUE(policy.allow_forward());

  safety_emergency_executor::CommandFrame resume_frame;
  resume_frame.speed = 0.7;
  resume_frame.angle = 0.2;
  ASSERT_TRUE(policy.apply(resume_frame));
  EXPECT_DOUBLE_EQ(resume_frame.speed, 0.7);
  EXPECT_DOUBLE_EQ(resume_frame.angle, 0.2);
}

TEST_F(SafetyExecutorComponentTest, SafetyPolicyResetClearsPreviousSafetyState)
{
  auto node = std::make_shared<rclcpp::Node>("safety_policy_reset_test");
  safety_emergency_executor::SafetyPolicyExecutor policy;
  policy.configure(*node);

  safety_emergency_executor::CommandFrame template_frame;
  nav2_monitor::msg::SafetyCmd estop_msg;
  estop_msg.action = nav2_monitor::msg::SafetyCmd::EMERGENCY_STOP;
  (void)policy.on_safety_cmd(estop_msg, template_frame);
  EXPECT_FALSE(policy.allow_forward());

  policy.reset();
  EXPECT_TRUE(policy.allow_forward());

  safety_emergency_executor::CommandFrame frame;
  frame.speed = 0.6;
  frame.angle = 0.2;
  ASSERT_TRUE(policy.apply(frame));
  EXPECT_DOUBLE_EQ(frame.speed, 0.6);
  EXPECT_DOUBLE_EQ(frame.angle, 0.2);
}

}  // namespace
