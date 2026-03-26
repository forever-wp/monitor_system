#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <json/json.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav2_monitor/msg/safety_cmd.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "safety_emergency_executor/external_override_controller.hpp"
#include "safety_emergency_executor/safety_emergency_executor_node.hpp"
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

TEST_F(SafetyExecutorComponentTest, VelocityConverterMapsFieldsAndUpdatesParams)
{
  auto node = std::make_shared<rclcpp::Node>("velocity_converter_test");
  safety_emergency_executor::VelocityConverter converter;
  converter.configure(*node);

  std::string error;
  ASSERT_TRUE(
    converter.update_params_from_json(
      "{\"acc\":1500,\"press\":950,\"place\":2,\"ulock\":3}", &error));

  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0.456;
  msg.angular.z = -0.234;
  const auto frame = converter.convert(msg);

  EXPECT_DOUBLE_EQ(frame.speed, 0.46);
  EXPECT_DOUBLE_EQ(frame.angle, -0.23);
  EXPECT_EQ(frame.acc, 1500);
  EXPECT_EQ(frame.press, 950);
  EXPECT_EQ(frame.place, 2);
  EXPECT_EQ(frame.ulock, 3);

  const auto json = converter.to_json(frame);
  EXPECT_NE(json.find("\"speed\""), std::string::npos);
  EXPECT_NE(json.find("\"press\":950"), std::string::npos);
}

TEST_F(SafetyExecutorComponentTest, VelocityConverterAccTopicOverrideKeepsDefaultFallback)
{
  auto node = std::make_shared<rclcpp::Node>("velocity_converter_acc_override_test");
  safety_emergency_executor::VelocityConverter converter;
  converter.configure(*node);

  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0.3;
  msg.angular.z = 0.1;

  const auto default_frame = converter.convert(msg);
  EXPECT_EQ(default_frame.acc, 2000);

  converter.update_acc_from_topic(3200);
  const auto overridden_frame = converter.convert(msg);
  EXPECT_EQ(overridden_frame.acc, 3200);

  std::string error;
  ASSERT_TRUE(converter.update_params_from_json("{\"acc\":1500}", &error));
  const auto still_overridden_frame = converter.convert(msg);
  EXPECT_EQ(still_overridden_frame.acc, 3200);
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
  EXPECT_TRUE(policy.on_safety_cmd(stop_msg, template_frame).empty());

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


class ManualOverrideTestHarness : public rclcpp::Node
{
public:
  ManualOverrideTestHarness()
  : Node("manual_override_test_harness")
  {
    command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/test/manual_override/command", rclcpp::QoS(20),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        command_payloads_.push_back(msg->data);
      });
    state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/test/manual_override_active", rclcpp::QoS(1).transient_local().reliable(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        manual_override_state_ = msg->data;
      });
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/test/manual_override/cmd_vel", rclcpp::QoS(20));
    safety_pub_ = this->create_publisher<nav2_monitor::msg::SafetyCmd>(
      "/test/manual_override/safety_cmd", rclcpp::QoS(20));
    manual_override_client_ = this->create_client<std_srvs::srv::SetBool>(
      "/test/set_manual_override");
  }

  bool wait_for_graph_ready(std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (
        manual_override_client_->wait_for_service(std::chrono::milliseconds(0)) &&
        cmd_vel_pub_->get_subscription_count() >= 1 &&
        safety_pub_->get_subscription_count() >= 1 &&
        command_sub_->get_publisher_count() >= 1 &&
        state_sub_->get_publisher_count() >= 1)
      {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  bool wait_for_manual_override_state(bool expected, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (manual_override_state_.has_value() && manual_override_state_.value() == expected) {
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  bool wait_for_command_count_at_least(size_t expected, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (command_count() >= expected) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  std_srvs::srv::SetBool::Response call_manual_override(
    bool active,
    std::chrono::milliseconds timeout)
  {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = active;
    auto future = manual_override_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      throw std::runtime_error("manual override service call timed out");
    }
    return *future.get();
  }

  void publish_cmd_vel(double linear_x, double angular_z)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    cmd_vel_pub_->publish(msg);
  }

  void publish_safety_cmd(uint8_t action, float slow_down_percentage = 0.0F)
  {
    nav2_monitor::msg::SafetyCmd msg;
    msg.action = action;
    msg.slow_down_percentage = slow_down_percentage;
    msg.reason = "test";
    safety_pub_->publish(msg);
  }

  size_t command_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return command_payloads_.size();
  }

  Json::Value latest_command_json() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string errors;
    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
    const auto & payload = command_payloads_.back();
    const bool ok = reader->parse(payload.data(), payload.data() + payload.size(), &root, &errors);
    if (!ok) {
      throw std::runtime_error("failed to parse command json: " + errors);
    }
    return root;
  }

private:
  mutable std::mutex mutex_;
  std::vector<std::string> command_payloads_;
  std::optional<bool> manual_override_state_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav2_monitor::msg::SafetyCmd>::SharedPtr safety_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr manual_override_client_;
};

TEST_F(SafetyExecutorComponentTest, ExternalOverrideControllerTransitionsAreStable)
{
  safety_emergency_executor::ExternalOverrideController controller;

  EXPECT_FALSE(controller.manual_override_active());

  const auto first_manual = controller.set_manual_override(true);
  EXPECT_TRUE(first_manual.state_changed);
  EXPECT_TRUE(first_manual.publish_zero_command);
  EXPECT_TRUE(controller.manual_override_active());

  const auto second_manual = controller.set_manual_override(true);
  EXPECT_FALSE(second_manual.state_changed);
  EXPECT_FALSE(second_manual.publish_zero_command);
  EXPECT_TRUE(controller.manual_override_active());

  const auto back_to_auto = controller.set_manual_override(false);
  EXPECT_TRUE(back_to_auto.state_changed);
  EXPECT_FALSE(back_to_auto.publish_zero_command);
  EXPECT_FALSE(controller.manual_override_active());
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

TEST_F(SafetyExecutorComponentTest, ManualOverrideServicePublishesStateAndBlocksAutoChain)
{
  auto harness = std::make_shared<ManualOverrideTestHarness>();

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("command_output_topic", "/test/manual_override/command"),
      rclcpp::Parameter("cmd_vel_topic", "/test/manual_override/cmd_vel"),
      rclcpp::Parameter("safety_cmd_topic", "/test/manual_override/safety_cmd"),
      rclcpp::Parameter("manual_override_service", "/test/set_manual_override"),
      rclcpp::Parameter("manual_override_state_topic", "/test/manual_override_active")
    });
  auto executor_node = std::make_shared<safety_emergency_executor::SafetyEmergencyExecutorNode>(
    options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness);
  executor.add_node(executor_node);
  std::thread spin_thread([&executor]() {executor.spin();});
  struct ExecutorCleanup
  {
    rclcpp::executors::SingleThreadedExecutor & executor;
    std::thread & spin_thread;
    ~ExecutorCleanup()
    {
      executor.cancel();
      if (spin_thread.joinable()) {
        spin_thread.join();
      }
    }
  } cleanup{executor, spin_thread};

  ASSERT_TRUE(harness->wait_for_graph_ready(std::chrono::milliseconds(2000)));
  ASSERT_TRUE(harness->wait_for_manual_override_state(false, std::chrono::milliseconds(2000)));

  harness->publish_cmd_vel(0.5, 0.2);
  ASSERT_TRUE(harness->wait_for_command_count_at_least(1, std::chrono::milliseconds(2000)));
  auto auto_json = harness->latest_command_json();
  EXPECT_DOUBLE_EQ(auto_json["speed"].asDouble(), 0.5);
  EXPECT_DOUBLE_EQ(auto_json["angle"].asDouble(), 0.2);

  const size_t auto_count = harness->command_count();
  const auto manual_response = harness->call_manual_override(true, std::chrono::milliseconds(2000));
  EXPECT_TRUE(manual_response.success);
  ASSERT_TRUE(harness->wait_for_manual_override_state(true, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(
    harness->wait_for_command_count_at_least(
      auto_count + 1,
      std::chrono::milliseconds(2000)));
  auto stop_json = harness->latest_command_json();
  EXPECT_DOUBLE_EQ(stop_json["speed"].asDouble(), 0.0);
  EXPECT_DOUBLE_EQ(stop_json["angle"].asDouble(), 0.0);

  const size_t manual_stop_count = harness->command_count();
  harness->publish_safety_cmd(nav2_monitor::msg::SafetyCmd::EMERGENCY_STOP);
  harness->publish_cmd_vel(0.9, 0.3);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  EXPECT_EQ(harness->command_count(), manual_stop_count);

  const auto auto_response = harness->call_manual_override(false, std::chrono::milliseconds(2000));
  EXPECT_TRUE(auto_response.success);
  ASSERT_TRUE(harness->wait_for_manual_override_state(false, std::chrono::milliseconds(2000)));

  harness->publish_cmd_vel(0.5, 0.2);
  ASSERT_TRUE(
    harness->wait_for_command_count_at_least(
      manual_stop_count + 1,
      std::chrono::milliseconds(2000)));
  auto resumed_json = harness->latest_command_json();
  EXPECT_DOUBLE_EQ(resumed_json["speed"].asDouble(), 0.5);
  EXPECT_DOUBLE_EQ(resumed_json["angle"].asDouble(), 0.2);

}

}  // namespace
