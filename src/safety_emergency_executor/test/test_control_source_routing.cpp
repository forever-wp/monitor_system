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
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "safety_emergency_executor/safety_emergency_executor_node.hpp"

namespace
{

class SafetyExecutorRoutingTest : public ::testing::Test
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

class ControlSourceHarness : public rclcpp::Node
{
public:
  ControlSourceHarness()
  : Node("control_source_test_harness")
  {
    command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/test/control_source/command", rclcpp::QoS(20),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        command_payloads_.push_back(msg->data);
      });
    state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/test/control_source/state", rclcpp::QoS(1).transient_local().reliable(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_state_ = msg->data;
      });
    navigation_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/test/control_source/cmd_vel/navigation", rclcpp::QoS(20));
    miniapp_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/test/control_source/cmd_vel/miniapp", rclcpp::QoS(20));
    remote_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/test/control_source/cmd_vel/remote", rclcpp::QoS(20));
    other_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/test/control_source/cmd_vel/other", rclcpp::QoS(20));
    safety_pub_ = this->create_publisher<nav2_monitor::msg::SafetyCmd>(
      "/test/control_source/safety_cmd", rclcpp::QoS(20));
    set_parameters_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(
      "/safety_emergency_executor/set_parameters");
    get_parameters_client_ = this->create_client<rcl_interfaces::srv::GetParameters>(
      "/safety_emergency_executor/get_parameters");
  }

  bool wait_for_graph_ready(std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (
        set_parameters_client_->wait_for_service(std::chrono::milliseconds(0)) &&
        get_parameters_client_->wait_for_service(std::chrono::milliseconds(0)) &&
        navigation_pub_->get_subscription_count() >= 1 &&
        remote_pub_->get_subscription_count() >= 1 &&
        miniapp_pub_->get_subscription_count() >= 1 &&
        other_pub_->get_subscription_count() >= 1 &&
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

  bool wait_for_state(const std::string & expected, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_state_.has_value() && latest_state_.value() == expected) {
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

  bool command_count_stays(size_t expected, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (command_count() != expected) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return true;
  }

  void publish_navigation(double linear_x, double angular_z)
  {
    publish_twist(navigation_pub_, linear_x, angular_z);
  }

  void publish_miniapp(double linear_x, double angular_z)
  {
    publish_twist(miniapp_pub_, linear_x, angular_z);
  }

  void publish_remote(double linear_x, double angular_z)
  {
    publish_twist(remote_pub_, linear_x, angular_z);
  }

  void publish_other(double linear_x, double angular_z)
  {
    publish_twist(other_pub_, linear_x, angular_z);
  }

  void publish_safety_cmd(uint8_t action, float slow_down_percentage = 0.0F)
  {
    nav2_monitor::msg::SafetyCmd msg;
    msg.action = action;
    msg.slow_down_percentage = slow_down_percentage;
    msg.reason = "test";
    safety_pub_->publish(msg);
  }

  std::vector<rcl_interfaces::msg::SetParametersResult> set_active_source(
    const std::string & source,
    std::chrono::milliseconds timeout)
  {
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(
      rclcpp::Parameter("active_control_source", source).to_parameter_msg());
    auto future = set_parameters_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      throw std::runtime_error("set_parameters call timed out");
    }
    return future.get()->results;
  }

  std::string query_active_source(std::chrono::milliseconds timeout)
  {
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back("active_control_source");
    auto future = get_parameters_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      throw std::runtime_error("get_parameters call timed out");
    }
    const auto response = future.get();
    if (response->values.empty()) {
      throw std::runtime_error("get_parameters returned no values");
    }
    return response->values.front().string_value;
  }

  size_t command_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return command_payloads_.size();
  }

  Json::Value latest_command_json() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (command_payloads_.empty()) {
      throw std::runtime_error("no command payloads recorded");
    }
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
  static void publish_twist(
    const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr & publisher,
    double linear_x,
    double angular_z)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    publisher->publish(msg);
  }

  mutable std::mutex mutex_;
  std::vector<std::string> command_payloads_;
  std::optional<std::string> latest_state_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr navigation_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr miniapp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr remote_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr other_pub_;
  rclcpp::Publisher<nav2_monitor::msg::SafetyCmd>::SharedPtr safety_pub_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_client_;
};

TEST_F(SafetyExecutorRoutingTest, DefaultsToNavigationAndPublishesState)
{
  auto harness = std::make_shared<ControlSourceHarness>();

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("command_output_topic", "/test/control_source/command"),
      rclcpp::Parameter("cmd_vel_navigation_topic", "/test/control_source/cmd_vel/navigation"),
      rclcpp::Parameter("cmd_vel_miniapp_topic", "/test/control_source/cmd_vel/miniapp"),
      rclcpp::Parameter("cmd_vel_remote_topic", "/test/control_source/cmd_vel/remote"),
      rclcpp::Parameter("cmd_vel_other_topic", "/test/control_source/cmd_vel/other"),
      rclcpp::Parameter("safety_cmd_topic", "/test/control_source/safety_cmd"),
      rclcpp::Parameter("control_source_state_topic", "/test/control_source/state"),
      rclcpp::Parameter("active_control_source", "navigation")
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
  EXPECT_TRUE(harness->wait_for_state("navigation", std::chrono::milliseconds(2000)));
  EXPECT_EQ(harness->query_active_source(std::chrono::milliseconds(2000)), "navigation");
}

TEST_F(SafetyExecutorRoutingTest, OnlyActiveSourceCommandsReachCommandTopic)
{
  auto harness = std::make_shared<ControlSourceHarness>();

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("command_output_topic", "/test/control_source/command"),
      rclcpp::Parameter("cmd_vel_navigation_topic", "/test/control_source/cmd_vel/navigation"),
      rclcpp::Parameter("cmd_vel_miniapp_topic", "/test/control_source/cmd_vel/miniapp"),
      rclcpp::Parameter("cmd_vel_remote_topic", "/test/control_source/cmd_vel/remote"),
      rclcpp::Parameter("cmd_vel_other_topic", "/test/control_source/cmd_vel/other"),
      rclcpp::Parameter("safety_cmd_topic", "/test/control_source/safety_cmd"),
      rclcpp::Parameter("control_source_state_topic", "/test/control_source/state"),
      rclcpp::Parameter("active_control_source", "navigation")
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

  harness->publish_remote(0.8, 0.3);
  EXPECT_TRUE(harness->command_count_stays(0, std::chrono::milliseconds(250)));

  harness->publish_navigation(0.5, 0.2);
  ASSERT_TRUE(harness->wait_for_command_count_at_least(1, std::chrono::milliseconds(2000)));
  const auto command = harness->latest_command_json();
  EXPECT_DOUBLE_EQ(command["speed"].asDouble(), 0.5);
  EXPECT_DOUBLE_EQ(command["angle"].asDouble(), 0.2);
}

TEST_F(SafetyExecutorRoutingTest, ParameterUpdateSwitchesActiveSource)
{
  auto harness = std::make_shared<ControlSourceHarness>();

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("command_output_topic", "/test/control_source/command"),
      rclcpp::Parameter("cmd_vel_navigation_topic", "/test/control_source/cmd_vel/navigation"),
      rclcpp::Parameter("cmd_vel_miniapp_topic", "/test/control_source/cmd_vel/miniapp"),
      rclcpp::Parameter("cmd_vel_remote_topic", "/test/control_source/cmd_vel/remote"),
      rclcpp::Parameter("cmd_vel_other_topic", "/test/control_source/cmd_vel/other"),
      rclcpp::Parameter("safety_cmd_topic", "/test/control_source/safety_cmd"),
      rclcpp::Parameter("control_source_state_topic", "/test/control_source/state"),
      rclcpp::Parameter("active_control_source", "navigation")
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
  const auto result = harness->set_active_source("remote", std::chrono::milliseconds(2000));
  ASSERT_EQ(result.size(), 1u);
  EXPECT_TRUE(result.front().successful);
  EXPECT_TRUE(harness->wait_for_state("remote", std::chrono::milliseconds(2000)));

  harness->publish_navigation(0.6, 0.2);
  EXPECT_TRUE(harness->command_count_stays(0, std::chrono::milliseconds(250)));

  harness->publish_remote(0.7, 0.1);
  ASSERT_TRUE(harness->wait_for_command_count_at_least(1, std::chrono::milliseconds(2000)));
  EXPECT_EQ(harness->query_active_source(std::chrono::milliseconds(2000)), "remote");
}

TEST_F(SafetyExecutorRoutingTest, InvalidParameterUpdateIsRejected)
{
  auto harness = std::make_shared<ControlSourceHarness>();

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("command_output_topic", "/test/control_source/command"),
      rclcpp::Parameter("cmd_vel_navigation_topic", "/test/control_source/cmd_vel/navigation"),
      rclcpp::Parameter("cmd_vel_miniapp_topic", "/test/control_source/cmd_vel/miniapp"),
      rclcpp::Parameter("cmd_vel_remote_topic", "/test/control_source/cmd_vel/remote"),
      rclcpp::Parameter("cmd_vel_other_topic", "/test/control_source/cmd_vel/other"),
      rclcpp::Parameter("safety_cmd_topic", "/test/control_source/safety_cmd"),
      rclcpp::Parameter("control_source_state_topic", "/test/control_source/state"),
      rclcpp::Parameter("active_control_source", "navigation")
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
  const auto result = harness->set_active_source("invalid", std::chrono::milliseconds(2000));
  ASSERT_EQ(result.size(), 1u);
  EXPECT_FALSE(result.front().successful);
  EXPECT_TRUE(harness->wait_for_state("navigation", std::chrono::milliseconds(2000)));
  EXPECT_EQ(harness->query_active_source(std::chrono::milliseconds(2000)), "navigation");
}

TEST_F(SafetyExecutorRoutingTest, SwitchingSourceDoesNotPublishSyntheticStop)
{
  auto harness = std::make_shared<ControlSourceHarness>();

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("command_output_topic", "/test/control_source/command"),
      rclcpp::Parameter("cmd_vel_navigation_topic", "/test/control_source/cmd_vel/navigation"),
      rclcpp::Parameter("cmd_vel_miniapp_topic", "/test/control_source/cmd_vel/miniapp"),
      rclcpp::Parameter("cmd_vel_remote_topic", "/test/control_source/cmd_vel/remote"),
      rclcpp::Parameter("cmd_vel_other_topic", "/test/control_source/cmd_vel/other"),
      rclcpp::Parameter("safety_cmd_topic", "/test/control_source/safety_cmd"),
      rclcpp::Parameter("control_source_state_topic", "/test/control_source/state"),
      rclcpp::Parameter("active_control_source", "navigation")
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
  harness->publish_navigation(0.5, 0.2);
  ASSERT_TRUE(harness->wait_for_command_count_at_least(1, std::chrono::milliseconds(2000)));
  const size_t before_switch = harness->command_count();

  const auto result = harness->set_active_source("remote", std::chrono::milliseconds(2000));
  ASSERT_EQ(result.size(), 1u);
  EXPECT_TRUE(result.front().successful);
  EXPECT_TRUE(harness->command_count_stays(before_switch, std::chrono::milliseconds(300)));
}

TEST_F(SafetyExecutorRoutingTest, SafetyCommandsStillApplyAfterSwitchingToRemote)
{
  auto harness = std::make_shared<ControlSourceHarness>();

  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("command_output_topic", "/test/control_source/command"),
      rclcpp::Parameter("cmd_vel_navigation_topic", "/test/control_source/cmd_vel/navigation"),
      rclcpp::Parameter("cmd_vel_miniapp_topic", "/test/control_source/cmd_vel/miniapp"),
      rclcpp::Parameter("cmd_vel_remote_topic", "/test/control_source/cmd_vel/remote"),
      rclcpp::Parameter("cmd_vel_other_topic", "/test/control_source/cmd_vel/other"),
      rclcpp::Parameter("safety_cmd_topic", "/test/control_source/safety_cmd"),
      rclcpp::Parameter("control_source_state_topic", "/test/control_source/state"),
      rclcpp::Parameter("active_control_source", "navigation")
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
  const auto result = harness->set_active_source("remote", std::chrono::milliseconds(2000));
  ASSERT_EQ(result.size(), 1u);
  ASSERT_TRUE(result.front().successful);

  harness->publish_safety_cmd(nav2_monitor::msg::SafetyCmd::SLOW_DOWN, 50.0F);
  // Safety commands arrive on a separate subscription, so give the executor one cycle to
  // apply the new policy before asserting against the next remote command.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  harness->publish_remote(1.0, 0.4);
  ASSERT_TRUE(harness->wait_for_command_count_at_least(1, std::chrono::milliseconds(2000)));
  const auto slowed_command = harness->latest_command_json();
  EXPECT_DOUBLE_EQ(slowed_command["speed"].asDouble(), 0.5);
  EXPECT_DOUBLE_EQ(slowed_command["angle"].asDouble(), 0.2);
}

}  // namespace
