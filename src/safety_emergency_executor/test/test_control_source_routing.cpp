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
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

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
        ++state_message_count_;
      });
    navigation_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/test/control_source/cmd_vel/navigation", rclcpp::QoS(20));
    miniapp_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/test/control_source/cmd_vel/miniapp", rclcpp::QoS(20));
    remote_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/test/control_source/cmd_vel/remote", rclcpp::QoS(20));
    other_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/test/control_source/cmd_vel/other", rclcpp::QoS(20));
    wheel_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/test/control_source/odom_base", rclcpp::QoS(20));
    loc_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/test/control_source/odom", rclcpp::QoS(20));
    safety_pub_ = this->create_publisher<nav2_monitor::msg::SafetyCmd>(
      "/test/control_source/safety_cmd", rclcpp::QoS(1).reliable().transient_local());
    pressure_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      "/test/control_source/pressure", rclcpp::QoS(20));
    control_source_cmd_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/control_source_cmd", rclcpp::QoS(10));
    set_parameters_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(
      "/safety_emergency_executor/set_parameters");
    query_control_source_client_ = this->create_client<std_srvs::srv::Trigger>(
      "/safety_emergency_executor/query_control_source");
  }

  bool wait_for_graph_ready(std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (
        set_parameters_client_->wait_for_service(std::chrono::milliseconds(0)) &&
        query_control_source_client_->wait_for_service(std::chrono::milliseconds(0)) &&
        navigation_pub_->get_subscription_count() >= 1 &&
        remote_pub_->get_subscription_count() >= 1 &&
        miniapp_pub_->get_subscription_count() >= 1 &&
        other_pub_->get_subscription_count() >= 1 &&
        wheel_odom_pub_->get_subscription_count() >= 1 &&
        loc_odom_pub_->get_subscription_count() >= 1 &&
        safety_pub_->get_subscription_count() >= 1 &&
        control_source_cmd_pub_->get_subscription_count() >= 1 &&
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

  bool wait_for_state_message_count_at_least(
    size_t expected,
    std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (state_message_count() >= expected) {
        return true;
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

  void publish_remote(const geometry_msgs::msg::Twist & msg)
  {
    remote_pub_->publish(msg);
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

  void publish_wheel_odom(double linear_x, double angular_z = 0.0)
  {
    publish_odom(wheel_odom_pub_, linear_x, angular_z);
  }

  void publish_pressure(int pressure)
  {
    std_msgs::msg::Int32 msg;
    msg.data = pressure;
    pressure_pub_->publish(msg);
  }

  void publish_control_source_command(const std::string & source)
  {
    std_msgs::msg::String msg;
    msg.data = source;
    control_source_cmd_pub_->publish(msg);
  }

  void publish_loc_odom(double linear_x, double angular_z = 0.0)
  {
    publish_odom(loc_odom_pub_, linear_x, angular_z);
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
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = query_control_source_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      throw std::runtime_error("query_control_source call timed out");
    }
    const auto response = future.get();
    if (!response->success) {
      throw std::runtime_error("query_control_source failed: " + response->message);
    }
    return response->message;
  }

  size_t command_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return command_payloads_.size();
  }

  size_t state_message_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_message_count_;
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

  static void publish_odom(
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & publisher,
    double linear_x,
    double angular_z)
  {
    nav_msgs::msg::Odometry msg;
    msg.twist.twist.linear.x = linear_x;
    msg.twist.twist.angular.z = angular_z;
    publisher->publish(msg);
  }

  mutable std::mutex mutex_;
  std::vector<std::string> command_payloads_;
  std::optional<std::string> latest_state_;
  size_t state_message_count_{0};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr navigation_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr miniapp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr remote_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr other_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr loc_odom_pub_;
  rclcpp::Publisher<nav2_monitor::msg::SafetyCmd>::SharedPtr safety_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pressure_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_source_cmd_pub_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr query_control_source_client_;
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
      rclcpp::Parameter("wheel_odom_topic", "/test/control_source/odom_base"),
      rclcpp::Parameter("loc_odom_topic", "/test/control_source/odom"),
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

TEST_F(SafetyExecutorRoutingTest, ControlSourceStateTopicPublishesContinuously)
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
      rclcpp::Parameter("control_source_state_publish_period_ms", 50),
      rclcpp::Parameter("wheel_odom_topic", "/test/control_source/odom_base"),
      rclcpp::Parameter("loc_odom_topic", "/test/control_source/odom"),
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
  ASSERT_TRUE(harness->wait_for_state("navigation", std::chrono::milliseconds(2000)));

  const size_t initial_state_message_count = harness->state_message_count();
  EXPECT_TRUE(
    harness->wait_for_state_message_count_at_least(
      initial_state_message_count + 2,
      std::chrono::milliseconds(400)));
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
      rclcpp::Parameter("wheel_odom_topic", "/test/control_source/odom_base"),
      rclcpp::Parameter("loc_odom_topic", "/test/control_source/odom"),
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
      rclcpp::Parameter("wheel_odom_topic", "/test/control_source/odom_base"),
      rclcpp::Parameter("loc_odom_topic", "/test/control_source/odom"),
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

TEST_F(SafetyExecutorRoutingTest, ParameterUpdateLogsWhenSourceIsAlreadyActive)
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
      rclcpp::Parameter("wheel_odom_topic", "/test/control_source/odom_base"),
      rclcpp::Parameter("loc_odom_topic", "/test/control_source/odom"),
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

  testing::internal::CaptureStderr();
  const auto result = harness->set_active_source("navigation", std::chrono::milliseconds(2000));
  const std::string logs = testing::internal::GetCapturedStderr();

  ASSERT_EQ(result.size(), 1u);
  EXPECT_TRUE(result.front().successful);
  EXPECT_NE(logs.find("control source already set to navigation"), std::string::npos);
}

TEST_F(SafetyExecutorRoutingTest, ParameterUpdateLogsSafetyStateForSwitchedSource)
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
      rclcpp::Parameter("wheel_odom_topic", "/test/control_source/odom_base"),
      rclcpp::Parameter("loc_odom_topic", "/test/control_source/odom"),
      rclcpp::Parameter("control_source_remote_safety_enabled", false),
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

  testing::internal::CaptureStderr();
  const auto result = harness->set_active_source("remote", std::chrono::milliseconds(2000));
  const std::string logs = testing::internal::GetCapturedStderr();

  ASSERT_EQ(result.size(), 1u);
  EXPECT_TRUE(result.front().successful);
  EXPECT_NE(logs.find("control source switched: navigation -> remote"), std::string::npos);
  EXPECT_NE(logs.find("safety_enabled=false"), std::string::npos);
}

TEST_F(SafetyExecutorRoutingTest, TopicCommandSwitchesActiveSourceAndUpdatesParameter)
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
      rclcpp::Parameter("wheel_odom_topic", "/test/control_source/odom_base"),
      rclcpp::Parameter("loc_odom_topic", "/test/control_source/odom"),
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

  harness->publish_control_source_command("remote");
  EXPECT_TRUE(harness->wait_for_state("remote", std::chrono::milliseconds(2000)));
  EXPECT_EQ(executor_node->get_parameter("active_control_source").as_string(), "remote");

  harness->publish_navigation(0.6, 0.2);
  EXPECT_TRUE(harness->command_count_stays(0, std::chrono::milliseconds(250)));

  harness->publish_remote(0.7, 0.1);
  ASSERT_TRUE(harness->wait_for_command_count_at_least(1, std::chrono::milliseconds(2000)));
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
      rclcpp::Parameter("wheel_odom_topic", "/test/control_source/odom_base"),
      rclcpp::Parameter("loc_odom_topic", "/test/control_source/odom"),
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

TEST_F(SafetyExecutorRoutingTest, ParameterServicesRemainResponsiveDuringEmergencyBrakeSequence)
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
      rclcpp::Parameter("wheel_odom_topic", "/test/control_source/odom_base"),
      rclcpp::Parameter("loc_odom_topic", "/test/control_source/odom"),
      rclcpp::Parameter("active_control_source", "navigation"),
      rclcpp::Parameter("brake_strong_repeat", 3),
      rclcpp::Parameter("brake_medium_repeat", 3),
      rclcpp::Parameter("brake_zero_repeat", 3),
      rclcpp::Parameter("brake_interval_ms", 120)
    });
  auto executor_node = std::make_shared<safety_emergency_executor::SafetyEmergencyExecutorNode>(
    options);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(harness);
  executor.add_node(executor_node);
  std::thread spin_thread([&executor]() {executor.spin();});
  struct ExecutorCleanup
  {
    rclcpp::executors::MultiThreadedExecutor & executor;
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

  harness->publish_safety_cmd(nav2_monitor::msg::SafetyCmd::EMERGENCY_STOP);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  EXPECT_NO_THROW(
    {
      const auto set_result = harness->set_active_source("remote", std::chrono::milliseconds(200));
      ASSERT_EQ(set_result.size(), 1u);
      EXPECT_TRUE(set_result.front().successful);
    });
  EXPECT_NO_THROW(
    {
      EXPECT_EQ(harness->query_active_source(std::chrono::milliseconds(200)), "remote");
    });
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
      rclcpp::Parameter("wheel_odom_topic", "/test/control_source/odom_base"),
      rclcpp::Parameter("loc_odom_topic", "/test/control_source/odom"),
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
      rclcpp::Parameter("wheel_odom_topic", "/test/control_source/odom_base"),
      rclcpp::Parameter("loc_odom_topic", "/test/control_source/odom"),
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

TEST_F(SafetyExecutorRoutingTest, SoftStopImmediatelyPublishesZeroCommand)
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
      rclcpp::Parameter("wheel_odom_topic", "/test/control_source/odom_base"),
      rclcpp::Parameter("loc_odom_topic", "/test/control_source/odom"),
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
  harness->publish_safety_cmd(nav2_monitor::msg::SafetyCmd::SOFT_STOP);

  ASSERT_TRUE(harness->wait_for_command_count_at_least(1, std::chrono::milliseconds(2000)));
  const auto stop_command = harness->latest_command_json();
  EXPECT_DOUBLE_EQ(stop_command["speed"].asDouble(), 0.0);
  EXPECT_DOUBLE_EQ(stop_command["angle"].asDouble(), 0.0);
}

TEST_F(SafetyExecutorRoutingTest, SafetyDisabledRemoteBypassesSlowDownPolicy)
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
      rclcpp::Parameter("wheel_odom_topic", "/test/control_source/odom_base"),
      rclcpp::Parameter("loc_odom_topic", "/test/control_source/odom"),
      rclcpp::Parameter("control_source_remote_safety_enabled", false),
      rclcpp::Parameter("active_control_source", "remote")
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

  harness->publish_safety_cmd(nav2_monitor::msg::SafetyCmd::SLOW_DOWN, 50.0F);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  harness->publish_remote(1.0, 0.4);
  ASSERT_TRUE(harness->wait_for_command_count_at_least(1, std::chrono::milliseconds(2000)));
  const auto command = harness->latest_command_json();
  EXPECT_DOUBLE_EQ(command["speed"].asDouble(), 1.0);
  EXPECT_DOUBLE_EQ(command["angle"].asDouble(), 0.4);
}

TEST_F(SafetyExecutorRoutingTest, SafetyDisabledRemoteIgnoresSoftStopImmediateCommand)
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
      rclcpp::Parameter("wheel_odom_topic", "/test/control_source/odom_base"),
      rclcpp::Parameter("loc_odom_topic", "/test/control_source/odom"),
      rclcpp::Parameter("control_source_remote_safety_enabled", false),
      rclcpp::Parameter("active_control_source", "remote")
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

  harness->publish_safety_cmd(nav2_monitor::msg::SafetyCmd::SOFT_STOP);
  EXPECT_TRUE(harness->command_count_stays(0, std::chrono::milliseconds(300)));
}

TEST_F(SafetyExecutorRoutingTest, SafetyDisabledRemoteIgnoresEmergencyStopSequence)
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
      rclcpp::Parameter("wheel_odom_topic", "/test/control_source/odom_base"),
      rclcpp::Parameter("loc_odom_topic", "/test/control_source/odom"),
      rclcpp::Parameter("control_source_remote_safety_enabled", false),
      rclcpp::Parameter("active_control_source", "remote"),
      rclcpp::Parameter("brake_strong_repeat", 1),
      rclcpp::Parameter("brake_medium_repeat", 1),
      rclcpp::Parameter("brake_zero_repeat", 1),
      rclcpp::Parameter("brake_interval_ms", 20)
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

  harness->publish_remote(0.6, 0.2);
  ASSERT_TRUE(harness->wait_for_command_count_at_least(1, std::chrono::milliseconds(2000)));
  const size_t before_emergency_stop = harness->command_count();

  harness->publish_safety_cmd(nav2_monitor::msg::SafetyCmd::EMERGENCY_STOP);
  EXPECT_TRUE(harness->command_count_stays(before_emergency_stop, std::chrono::milliseconds(300)));

  harness->publish_remote(0.7, 0.1);
  ASSERT_TRUE(
    harness->wait_for_command_count_at_least(
      before_emergency_stop + 1,
      std::chrono::milliseconds(2000)));
  const auto command = harness->latest_command_json();
  EXPECT_DOUBLE_EQ(command["speed"].asDouble(), 0.7);
  EXPECT_DOUBLE_EQ(command["angle"].asDouble(), 0.1);
}

TEST_F(SafetyExecutorRoutingTest, RemoteSourceCanEmbedCommandFieldsInTwistPayload)
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
      rclcpp::Parameter("wheel_odom_topic", "/test/control_source/odom_base"),
      rclcpp::Parameter("loc_odom_topic", "/test/control_source/odom"),
      rclcpp::Parameter("active_control_source", "navigation"),
      rclcpp::Parameter("cmd_vel_navigation_extended_fields_enabled", false),
      rclcpp::Parameter("cmd_vel_miniapp_extended_fields_enabled", true),
      rclcpp::Parameter("cmd_vel_remote_extended_fields_enabled", true),
      rclcpp::Parameter("cmd_vel_other_extended_fields_enabled", true)
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

  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0.53;
  msg.linear.y = 950.0;
  msg.linear.z = 1500.0;
  msg.angular.x = 2.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.21;
  harness->publish_remote(msg);
  ASSERT_TRUE(harness->wait_for_command_count_at_least(1, std::chrono::milliseconds(2000)));

  const auto command = harness->latest_command_json();
  EXPECT_DOUBLE_EQ(command["speed"].asDouble(), 0.53);
  EXPECT_DOUBLE_EQ(command["angle"].asDouble(), 0.21);
  EXPECT_EQ(command["press"].asInt(), 950);
  EXPECT_EQ(command["acc"].asInt(), 1500);
  EXPECT_EQ(command["place"].asInt(), 2);
  EXPECT_EQ(command["ulock"].asInt(), 0);
}

TEST_F(SafetyExecutorRoutingTest, EmbeddedTwistFieldsBypassPressureAdjuster)
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
      rclcpp::Parameter("wheel_odom_topic", "/test/control_source/odom_base"),
      rclcpp::Parameter("loc_odom_topic", "/test/control_source/odom"),
      rclcpp::Parameter("active_control_source", "navigation"),
      rclcpp::Parameter("cmd_vel_navigation_extended_fields_enabled", false),
      rclcpp::Parameter("cmd_vel_miniapp_extended_fields_enabled", true),
      rclcpp::Parameter("cmd_vel_remote_extended_fields_enabled", true),
      rclcpp::Parameter("cmd_vel_other_extended_fields_enabled", true)
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

  harness->publish_wheel_odom(1.0);
  harness->publish_loc_odom(0.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0.53;
  msg.linear.y = 950.0;
  msg.linear.z = 1500.0;
  msg.angular.x = 2.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.21;
  harness->publish_remote(msg);
  ASSERT_TRUE(harness->wait_for_command_count_at_least(1, std::chrono::milliseconds(2000)));

  const auto command = harness->latest_command_json();
  EXPECT_EQ(command["press"].asInt(), 950);
}

TEST_F(SafetyExecutorRoutingTest, PressureTopicOverrideUpdatesOutgoingCommandPressure)
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
      rclcpp::Parameter("wheel_odom_topic", "/test/control_source/odom_base"),
      rclcpp::Parameter("loc_odom_topic", "/test/control_source/odom"),
      rclcpp::Parameter("pressure_update_topic", "/test/control_source/pressure"),
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
  harness->publish_pressure(1100);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  harness->publish_navigation(0.5, 0.2);
  ASSERT_TRUE(harness->wait_for_command_count_at_least(1, std::chrono::milliseconds(2000)));

  const auto command = harness->latest_command_json();
  EXPECT_EQ(command["press"].asInt(), 1100);
}

TEST_F(SafetyExecutorRoutingTest, PressureTopicOverrideBypassesAutoPressureDuringHoldWindow)
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
      rclcpp::Parameter("wheel_odom_topic", "/test/control_source/odom_base"),
      rclcpp::Parameter("loc_odom_topic", "/test/control_source/odom"),
      rclcpp::Parameter("pressure_update_topic", "/test/control_source/pressure"),
      rclcpp::Parameter("external_pressure_hold_s", 30.0),
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
  harness->publish_wheel_odom(1.0);
  harness->publish_loc_odom(0.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  harness->publish_pressure(1100);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  harness->publish_navigation(0.5, 0.2);
  ASSERT_TRUE(harness->wait_for_command_count_at_least(1, std::chrono::milliseconds(2000)));

  const auto command = harness->latest_command_json();
  EXPECT_EQ(command["press"].asInt(), 1100);
}

TEST_F(SafetyExecutorRoutingTest, AutoPressureResumesAfterPressureHoldWindowExpires)
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
      rclcpp::Parameter("wheel_odom_topic", "/test/control_source/odom_base"),
      rclcpp::Parameter("loc_odom_topic", "/test/control_source/odom"),
      rclcpp::Parameter("pressure_update_topic", "/test/control_source/pressure"),
      rclcpp::Parameter("external_pressure_hold_s", 0.01),
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
  harness->publish_wheel_odom(1.0);
  harness->publish_loc_odom(0.0);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  harness->publish_pressure(1100);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  harness->publish_navigation(0.5, 0.2);
  ASSERT_TRUE(harness->wait_for_command_count_at_least(1, std::chrono::milliseconds(2000)));

  const auto command = harness->latest_command_json();
  EXPECT_GT(command["press"].asInt(), 1100);
}

}  // namespace
