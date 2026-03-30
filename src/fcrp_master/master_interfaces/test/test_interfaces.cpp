

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "master_interfaces/msg/error_code.hpp"
#include "master_interfaces/msg/robot_state.hpp"
#include "master_interfaces/msg/standard_response.hpp"
#include "master_interfaces/msg/navigation_status.hpp"
#include "master_interfaces/msg/system_state.hpp"
#include "master_interfaces/msg/state_transition.hpp"

#include "master_interfaces/srv/ping.hpp"
#include "master_interfaces/srv/navigate_to_pose.hpp"
#include "master_interfaces/srv/cancel_navigation.hpp"
#include "master_interfaces/srv/get_navigation_status.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"

class BasicMessageTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
    }

    void TearDown() override {
        rclcpp::shutdown();
    }
};

TEST_F(BasicMessageTest, ErrorCodeMessage) {
    auto error_msg = master_interfaces::msg::ErrorCode();

    error_msg.code = master_interfaces::msg::ErrorCode::SUCCESS;
    error_msg.description = "操作成功";
    
    EXPECT_EQ(error_msg.code, 0);
    EXPECT_EQ(error_msg.description, "操作成功");

    EXPECT_EQ(master_interfaces::msg::ErrorCode::SUCCESS, 0);
    EXPECT_EQ(master_interfaces::msg::ErrorCode::UNKNOWN_ERROR, 1000);
    EXPECT_EQ(master_interfaces::msg::ErrorCode::NETWORK_ERROR, 1001);
    EXPECT_EQ(master_interfaces::msg::ErrorCode::INVALID_PARAMETER, 1002);
}

TEST_F(BasicMessageTest, StandardResponseMessage) {
    auto response_msg = master_interfaces::msg::StandardResponse();

    response_msg.error_code.code = master_interfaces::msg::ErrorCode::SUCCESS;
    response_msg.error_code.description = "测试成功";

    response_msg.timestamp = rclcpp::Clock().now();
    
    EXPECT_EQ(response_msg.error_code.code, 0);
    EXPECT_EQ(response_msg.error_code.description, "测试成功");
    EXPECT_GT(response_msg.timestamp.sec, 0);
}

TEST_F(BasicMessageTest, SystemStateMessage) {
    auto state_msg = master_interfaces::msg::SystemState();

    state_msg.current_state = master_interfaces::msg::SystemState::ACTIVE;
    state_msg.previous_state = master_interfaces::msg::SystemState::IDLE;
    state_msg.state_description = "系统激活状态";
    state_msg.state_entered_time = rclcpp::Clock().now();
    state_msg.state_duration = 10.5;
    state_msg.state_data = R"({"battery_level": 85})";
    
    EXPECT_EQ(state_msg.current_state, master_interfaces::msg::SystemState::ACTIVE);
    EXPECT_EQ(state_msg.previous_state, master_interfaces::msg::SystemState::IDLE);
    EXPECT_EQ(state_msg.state_description, "系统激活状态");
    EXPECT_DOUBLE_EQ(state_msg.state_duration, 10.5);

    EXPECT_EQ(master_interfaces::msg::SystemState::UNKNOWN, 0);
    EXPECT_EQ(master_interfaces::msg::SystemState::IDLE, 2);
    EXPECT_EQ(master_interfaces::msg::SystemState::ACTIVE, 3);
    EXPECT_EQ(master_interfaces::msg::SystemState::NAVIGATING, 4);
    EXPECT_EQ(master_interfaces::msg::SystemState::ERROR, 8);
}

TEST_F(BasicMessageTest, NavigationStatusMessage) {
    auto nav_msg = master_interfaces::msg::NavigationStatus();

    nav_msg.task_id = "nav_task_001";
    nav_msg.status = master_interfaces::msg::NavigationStatus::RUNNING;
    nav_msg.remaining_distance = 5.2;
    nav_msg.estimated_time_remaining = 30.0;
    nav_msg.progress = 0.75;
    nav_msg.current_planner_id = "NavfnPlanner";
    nav_msg.current_controller_id = "FollowPath";
    nav_msg.created_time = rclcpp::Clock().now();
    nav_msg.error_message = "";
    
    EXPECT_EQ(nav_msg.task_id, "nav_task_001");
    EXPECT_EQ(nav_msg.status, master_interfaces::msg::NavigationStatus::RUNNING);
    EXPECT_DOUBLE_EQ(nav_msg.remaining_distance, 5.2);
    EXPECT_DOUBLE_EQ(nav_msg.progress, 0.75);

    EXPECT_EQ(master_interfaces::msg::NavigationStatus::PENDING, 0);
    EXPECT_EQ(master_interfaces::msg::NavigationStatus::RUNNING, 2);
    EXPECT_EQ(master_interfaces::msg::NavigationStatus::SUCCEEDED, 3);
    EXPECT_EQ(master_interfaces::msg::NavigationStatus::FAILED, 4);
}

TEST_F(BasicMessageTest, StateTransitionMessage) {
    auto transition_msg = master_interfaces::msg::StateTransition();

    transition_msg.transition_id = "trans_001";
    transition_msg.from_state = master_interfaces::msg::SystemState::IDLE;
    transition_msg.to_state = master_interfaces::msg::SystemState::ACTIVE;
    transition_msg.transition_type = master_interfaces::msg::StateTransition::AUTOMATIC;
    transition_msg.transition_result = master_interfaces::msg::StateTransition::SUCCESS;
    transition_msg.trigger_event = "system_activation";
    transition_msg.reason = "用户激活系统";
    transition_msg.transition_duration_ms = 150;
    
    EXPECT_EQ(transition_msg.transition_id, "trans_001");
    EXPECT_EQ(transition_msg.from_state, master_interfaces::msg::SystemState::IDLE);
    EXPECT_EQ(transition_msg.to_state, master_interfaces::msg::SystemState::ACTIVE);
    EXPECT_EQ(transition_msg.transition_type, master_interfaces::msg::StateTransition::AUTOMATIC);
    EXPECT_EQ(transition_msg.transition_duration_ms, 150);
}

class ServiceInterfaceTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
    }

    void TearDown() override {
        rclcpp::shutdown();
    }
};

TEST_F(ServiceInterfaceTest, PingService) {
    
    auto request = std::make_shared<master_interfaces::srv::Ping::Request>();
    request->message = "ping测试";
    
    EXPECT_EQ(request->message, "ping测试");

    auto response = std::make_shared<master_interfaces::srv::Ping::Response>();
    response->response.error_code.code = master_interfaces::msg::ErrorCode::SUCCESS;
    response->response.error_code.description = "pong";
    
    EXPECT_EQ(response->response.error_code.code, 0);
}

TEST_F(ServiceInterfaceTest, NavigateToPoseService) {
    
    auto request = std::make_shared<master_interfaces::srv::NavigateToPose::Request>();

    request->target_pose.header.frame_id = "map";
    request->target_pose.header.stamp = rclcpp::Clock().now();
    request->target_pose.pose.position.x = 2.0;
    request->target_pose.pose.position.y = 1.0;
    request->target_pose.pose.position.z = 0.0;
    request->target_pose.pose.orientation.w = 1.0;

    request->planner_id = "NavfnPlanner";
    request->controller_id = "FollowPath";
    request->timeout = 60.0;
    
    EXPECT_EQ(request->target_pose.header.frame_id, "map");
    EXPECT_DOUBLE_EQ(request->target_pose.pose.position.x, 2.0);
    EXPECT_EQ(request->planner_id, "NavfnPlanner");
    EXPECT_DOUBLE_EQ(request->timeout, 60.0);

    auto response = std::make_shared<master_interfaces::srv::NavigateToPose::Response>();
    response->response.error_code.code = master_interfaces::msg::ErrorCode::SUCCESS;
    response->task_id = "nav_task_001";
    
    EXPECT_EQ(response->response.error_code.code, 0);
    EXPECT_EQ(response->task_id, "nav_task_001");
}

TEST_F(ServiceInterfaceTest, CancelNavigationService) {
    
    auto request = std::make_shared<master_interfaces::srv::CancelNavigation::Request>();
    request->task_id = "nav_task_001";
    request->reason = "用户取消";
    
    EXPECT_EQ(request->task_id, "nav_task_001");
    EXPECT_EQ(request->reason, "用户取消");

    auto response = std::make_shared<master_interfaces::srv::CancelNavigation::Response>();
    response->response.error_code.code = master_interfaces::msg::ErrorCode::SUCCESS;
    response->cancelled_tasks_count = 1;
    response->cancelled_task_ids.push_back("nav_task_001");
    
    EXPECT_EQ(response->cancelled_tasks_count, 1);
    EXPECT_EQ(response->cancelled_task_ids.size(), 1);
    EXPECT_EQ(response->cancelled_task_ids[0], "nav_task_001");
}

TEST_F(ServiceInterfaceTest, GetNavigationStatusService) {
    
    auto request = std::make_shared<master_interfaces::srv::GetNavigationStatus::Request>();
    request->task_id = "nav_task_001";
    
    EXPECT_EQ(request->task_id, "nav_task_001");

    auto response = std::make_shared<master_interfaces::srv::GetNavigationStatus::Response>();
    response->response.error_code.code = master_interfaces::msg::ErrorCode::SUCCESS;

    master_interfaces::msg::NavigationStatus nav_status;
    nav_status.task_id = "nav_task_001";
    nav_status.status = master_interfaces::msg::NavigationStatus::RUNNING;
    nav_status.progress = 0.5;
    nav_status.remaining_distance = 3.0;
    
    response->navigation_status.push_back(nav_status);
    
    EXPECT_EQ(response->navigation_status.size(), 1);
    EXPECT_EQ(response->navigation_status[0].task_id, "nav_task_001");
    EXPECT_EQ(response->navigation_status[0].status, master_interfaces::msg::NavigationStatus::RUNNING);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
