#ifndef NAV2_MONITOR__VEHICLE_STATE_JUDGE_NODE_HPP_
#define NAV2_MONITOR__VEHICLE_STATE_JUDGE_NODE_HPP_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

#include "nav2_monitor/chassis_evaluator.hpp"
#include "nav2_monitor/fault_detector.hpp"
#include "nav2_monitor/monitor_data_store.hpp"

namespace nav2_monitor
{

class VehicleStateJudgeNode : public rclcpp::Node
{
public:
  VehicleStateJudgeNode();

private:
  void load_configuration();
  void configure_subscriptions();
  void reset_subscriptions();
  void configure_profile_subscription();
  void on_config_profile(const std_msgs::msg::String::SharedPtr msg);
  void try_subscribe_moto_topic();
  void on_command(const std_msgs::msg::String::SharedPtr msg);
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void on_moto_serialized(const rclcpp::SerializedMessage & msg);
  void evaluate_and_publish();
  void publish_state(
    const std::vector<FaultInfo> & faults,
    const ChassisRuntimeState & chassis_state,
    const rclcpp::Time & now);
  void publish_human_intervention_request(const FaultInfo & fault, const rclcpp::Time & now);

  double parse_command_speed(const std::string & payload) const;
  bool decode_moto_info(
    const rclcpp::SerializedMessage & msg,
    double & left_speed,
    double & right_speed) const;
  void update_imu_motion(const sensor_msgs::msg::Imu::SharedPtr msg);
  rclcpp::Time stamp_or_now(const builtin_interfaces::msg::Time & stamp) const;
  static std::string json_escape(const std::string & input);
  static std::string fault_level_to_string(FaultLevel level);
  static std::string action_to_string(ActionType action);
  static std::string safety_command_to_string(SafetyCommandType command);

  std::string fault_config_path_{"/opt/ry/config/Monitor/nav2_monitor/fault_detector_config.yaml"};
  std::string resolved_fault_config_path_;
  std::string config_profile_topic_{"/monitor/config_profile"};
  std::string publish_topic_{"/monitor/vehicle_state"};
  std::string human_intervention_topic_{"/nav2_monitor/human_intervention"};
  double check_rate_hz_{10.0};

  FaultDetector fault_detector_;
  ChassisEvaluator evaluator_;
  MonitorDataStore data_store_;
  ChassisStationaryConfig cfg_{};
  MultiValueJudgeConfig multi_value_cfg_{2, 2};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr human_intervention_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_profile_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::GenericSubscription::SharedPtr moto_sub_;
  rclcpp::TimerBase::SharedPtr evaluate_timer_;
  rclcpp::TimerBase::SharedPtr moto_retry_timer_;

  std::string moto_topic_type_;
  std::set<std::string> active_human_fault_keys_;

  rclcpp::Time imu_last_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time imu_last_process_time_{0, 0, RCL_ROS_TIME};
  bool imu_time_initialized_{false};
  double imu_speed_estimate_{0.0};
  double imu_acc_bias_{0.0};
  bool imu_bias_calibrated_{false};
  std::vector<double> imu_bias_samples_;
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__VEHICLE_STATE_JUDGE_NODE_HPP_
