#ifndef NAV2_MONITOR__NAV2_MONITOR_NODE_HPP_
#define NAV2_MONITOR__NAV2_MONITOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav2_monitor/msg/monitor_status.hpp>
#include <nav2_monitor/msg/fault_event.hpp>
#include <nav2_monitor/msg/algorithm_feedback.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <map>
#include <string>
#include <vector>
#include <mutex>
#include <deque>
#include "nav2_monitor/system_monitor.hpp"
#include "nav2_monitor/monitor_data_store.hpp"
#include "nav2_monitor/monitor_reporter.hpp"
#include "nav2_monitor/fault_detector.hpp"
#include "nav2_monitor/fault_config_watcher.hpp"
#include "nav2_monitor/fault_state_coordinator.hpp"
#include "nav2_monitor/task_fault_config_selector.hpp"
#include "nav2_monitor/task_status_mapper.hpp"
#include "nav2_monitor/vehicle_status_monitor.hpp"
#include <nav2_monitor/msg/safety_cmd.hpp>

namespace nav2_monitor
{

struct TopicInfo
{
  std::string type;
};

struct TransformInfo
{
  rclcpp::Time last_update;
  double latency_ms;
};

class Nav2MonitorNode : public rclcpp::Node
{
public:
  Nav2MonitorNode();

private:
  void scan_topology();
  void check_health();
  void on_algorithm_feedback(const msg::AlgorithmFeedback::SharedPtr msg);
  void on_task_status(const std_msgs::msg::String::SharedPtr msg);
  void on_command(const std_msgs::msg::String::SharedPtr msg);
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_chassis_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void on_battery_state(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void on_collision_prediction_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_collision_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void on_collision_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void on_collision_ultrasonic(const std_msgs::msg::String::SharedPtr msg);
  void try_subscribe_moto_topic();
  double parse_command_speed(const std::string & payload) const;
  bool decode_moto_info(const rclcpp::SerializedMessage & msg, double & left_speed, double & right_speed) const;
  bool should_publish_action(const std::string & module_name, ActionType action, const rclcpp::Time & now);
  rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter>& params);
  void subscribe_watch_topics();
  void publish_collision_zones();
  bool reload_fault_config_if_needed(bool force = false);
  void apply_loaded_fault_config();
  void configure_chassis_monitoring();
  void configure_collision_monitoring();
  void clear_watch_topic_subscriptions();
  void load_task_fault_config_mappings();
  void load_task_status_code_mappings();
  void configure_task_status_subscription();
  void update_task_selected_fault_config(bool force_reload);
  bool update_current_nav_task_locked(const std::string & task_name, const std::string & change_source);
  rclcpp::QoS build_watch_topic_qos(const std::string & topic, const std::string & type) const;
  rclcpp::QoS build_topic_subscription_qos(const std::string & topic, const rclcpp::QoS & fallback, size_t max_depth) const;
  rclcpp::Time stamp_or_now(const builtin_interfaces::msg::Time & stamp) const;

  rclcpp::TimerBase::SharedPtr scan_timer_;
  rclcpp::TimerBase::SharedPtr check_timer_;
  rclcpp::Publisher<msg::MonitorStatus>::SharedPtr pub_;
  rclcpp::Publisher<msg::FaultEvent>::SharedPtr fault_event_pub_;
  rclcpp::Subscription<msg::AlgorithmFeedback>::SharedPtr algorithm_feedback_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr chassis_imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr collision_prediction_cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr collision_scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr collision_pointcloud_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr collision_ultrasonic_sub_;
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr> collision_zone_pubs_;
  rclcpp::GenericSubscription::SharedPtr moto_sub_;

  std::mutex mtx_;
  std::vector<std::string> target_nodes_;
  std::vector<std::string> watch_topics_;
  std::vector<std::string> fallback_target_nodes_;
  std::vector<std::string> fallback_watch_topics_;
  bool monitor_targets_from_fault_config_{false};
  std::map<std::string, TopicInfo> topic_info_;
  std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> topic_subs_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<std::pair<std::string, std::string>> target_transforms_;
  std::map<std::pair<std::string, std::string>, TransformInfo> tf_info_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;

  double timeout_;
  double safety_cooldown_s_;
  double supervisor_cooldown_s_;
  std::string algorithm_feedback_topic_;
  std::string task_status_topic_{"/task_status"};
  std::string battery_state_topic_;
  std::string base_frame_id_{"base_link"};
  std::string command_topic_;
  std::string moto_topic_;
  std::string odom_topic_;
  std::string chassis_imu_topic_;
  std::string moto_topic_type_;
  rclcpp::Time chassis_imu_last_stamp_{0, 0, RCL_ROS_TIME};
  bool chassis_imu_time_initialized_{false};
  double chassis_imu_speed_estimate_{0.0};
  double chassis_imu_acc_bias_{0.0};
  bool chassis_imu_bias_calibrated_{false};
  std::vector<double> chassis_imu_bias_samples_;
  double chassis_imu_speed_threshold_{0.03};
  double chassis_imu_yaw_rate_threshold_{0.08};
  double chassis_imu_static_command_threshold_{0.02};
  double chassis_imu_decay_rate_{0.98};
  int chassis_imu_bias_calibration_samples_{50};
  double battery_state_timeout_s_{5.0};
  bool fault_config_reload_enabled_{true};
  std::string base_fault_config_path_;
  std::string resolved_base_fault_config_path_;
  std::string fault_config_path_;
  std::string resolved_fault_config_path_;
  std::string current_nav_task_{"default"};
  std::map<std::string, std::string> task_fault_config_mappings_;
  std::map<std::string, std::string> task_status_code_mappings_;
  FaultConfigWatcher fault_config_watcher_;
  std::string pending_task_switch_source_;
  TaskFaultConfigSelector task_fault_config_selector_;
  TaskStatusMapper task_status_mapper_;
  SystemMonitor sys_monitor_;
  MonitorDataStore data_store_;
  FaultDetector fault_detector_;
  MonitorReporter monitor_reporter_;
  FaultStateCoordinator fault_state_coordinator_;
  std::unique_ptr<VehicleStatusMonitor> vehicle_monitor_;
  std::map<std::string, rclcpp::Time> last_action_publish_time_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr supervisor_pub_;
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__NAV2_MONITOR_NODE_HPP_
