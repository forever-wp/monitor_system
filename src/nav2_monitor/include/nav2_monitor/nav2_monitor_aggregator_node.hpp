#ifndef NAV2_MONITOR__NAV2_MONITOR_AGGREGATOR_NODE_HPP_
#define NAV2_MONITOR__NAV2_MONITOR_AGGREGATOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <nav2_monitor/msg/monitor_status.hpp>
#include <nav2_monitor/msg/fault_event.hpp>
#include <std_msgs/msg/string.hpp>
#include <master_interfaces/msg/task_status.hpp>
#include <map>
#include <optional>
#include <string>
#include <vector>
#include <mutex>
#include "nav2_monitor/system_monitor.hpp"
#include "nav2_monitor/monitor_data_store.hpp"
#include "nav2_monitor/monitor_reporter.hpp"
#include "nav2_monitor/event_codex_arbiter.hpp"
#include "nav2_monitor/event_executor.hpp"
#include "nav2_monitor/fault_detector.hpp"
#include "nav2_monitor/fault_config_watcher.hpp"
#include "nav2_monitor/task_fault_config_selector.hpp"
#include "nav2_monitor/task_status_mapper.hpp"
#include "nav2_monitor/task_status_message_adapter.hpp"
#include "nav2_monitor/vehicle_status_monitor.hpp"
#include <nav2_monitor/msg/safety_cmd.hpp>

namespace nav2_monitor
{

struct TransformInfo
{
  rclcpp::Time last_update;
  double latency_ms;
};

class Nav2MonitorAggregatorNode : public rclcpp::Node
{
public:
  Nav2MonitorAggregatorNode();

private:
  struct TopicQosOverride
  {
    bool has_reliability{false};
    rclcpp::ReliabilityPolicy reliability{rclcpp::ReliabilityPolicy::BestEffort};
    bool has_durability{false};
    rclcpp::DurabilityPolicy durability{rclcpp::DurabilityPolicy::Volatile};
    bool has_depth{false};
    size_t depth{10};
  };

  void scan_topology();
  void check_health();
  void on_task_status(const master_interfaces::msg::TaskStatus::SharedPtr msg);
  void on_topic_states(const std_msgs::msg::String::SharedPtr msg);
  void on_vehicle_state(const std_msgs::msg::String::SharedPtr msg);
  void on_node_tf_state(const std_msgs::msg::String::SharedPtr msg);
  void on_monitor_battery_state(const std_msgs::msg::String::SharedPtr msg);
  void on_feedback_state(const std_msgs::msg::String::SharedPtr msg);
  void on_collision_state(const std_msgs::msg::String::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter>& params);
  bool reload_fault_config_if_needed(bool force = false);
  void apply_loaded_fault_config();
  void publish_config_profile();
  void configure_topic_state_subscription();
  void configure_vehicle_state_subscription();
  void configure_node_tf_state_subscription();
  void configure_battery_state_subscription();
  void configure_feedback_state_subscription();
  void configure_collision_state_subscription();
  void load_task_fault_config_mappings();
  void load_task_status_code_mappings();
  void load_topic_qos_overrides();
  void configure_task_status_subscription();
  void update_task_selected_fault_config(bool force_reload);
  rclcpp::SubscriptionOptions make_subscription_options(
    const rclcpp::CallbackGroup::SharedPtr & callback_group) const;
  bool update_current_nav_task_locked(const std::string & task_name, const std::string & change_source);
  std::optional<TopicQosOverride> find_topic_qos_override(const std::string & topic) const;
  rclcpp::QoS apply_topic_qos_override(
    const std::string & topic, const rclcpp::QoS & qos, size_t max_depth) const;
  rclcpp::QoS build_topic_subscription_qos(const std::string & topic, const rclcpp::QoS & fallback, size_t max_depth) const;

  rclcpp::TimerBase::SharedPtr scan_timer_;
  rclcpp::TimerBase::SharedPtr check_timer_;
  rclcpp::Publisher<msg::MonitorStatus>::SharedPtr pub_;
  rclcpp::Publisher<msg::FaultEvent>::SharedPtr fault_event_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr config_profile_pub_;
  rclcpp::Subscription<master_interfaces::msg::TaskStatus>::SharedPtr task_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr topic_states_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vehicle_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr node_tf_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr monitor_battery_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr feedback_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr collision_state_sub_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::CallbackGroup::SharedPtr default_callback_group_;

  std::mutex mtx_;
  std::vector<std::string> target_nodes_;
  std::vector<std::string> watch_topics_;
  std::vector<std::string> fallback_target_nodes_;
  std::vector<std::string> fallback_watch_topics_;
  bool monitor_targets_from_fault_config_{false};

  std::vector<std::pair<std::string, std::string>> target_transforms_;
  std::map<std::pair<std::string, std::string>, TransformInfo> tf_info_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;

  double timeout_;
  double nodemanager_cooldown_s_;
  double safety_cmd_republish_period_s_{0.2};
  std::string topic_states_topic_{"/monitor/topic_states"};
  std::string vehicle_state_topic_{"/monitor/vehicle_state"};
  std::string node_tf_state_topic_{"/monitor/node_tf_state"};
  std::string monitor_battery_state_topic_{"/monitor/battery_state"};
  std::string feedback_state_topic_{"/monitor/feedback_state"};
  std::string collision_state_topic_{"/monitor/collision_state"};
  std::string config_profile_topic_{"/monitor/config_profile"};
  double vehicle_state_timeout_s_{1.0};
  double node_tf_state_timeout_s_{2.0};
  double monitor_battery_state_timeout_s_{3.0};
  double feedback_state_timeout_s_{2.0};
  double collision_state_timeout_s_{1.0};
  std::string task_status_topic_{"/task_status_code"};
  bool fault_config_reload_enabled_{true};
  std::string base_fault_config_path_;
  std::string resolved_base_fault_config_path_;
  std::string fault_config_path_;
  std::string resolved_fault_config_path_;
  std::string current_nav_task_{"default"};
  std::map<std::string, std::string> task_fault_config_mappings_;
  std::map<std::string, std::string> task_status_code_mappings_;
  std::map<std::string, TopicQosOverride> topic_qos_overrides_;
  FaultConfigWatcher fault_config_watcher_;
  std::string pending_task_switch_source_;
  TaskFaultConfigSelector task_fault_config_selector_;
  TaskStatusMapper task_status_mapper_;
  SystemMonitor sys_monitor_;
  MonitorDataStore data_store_;
  FaultDetector fault_detector_;
  MonitorReporter monitor_reporter_;
  EventCodexArbiter event_codex_arbiter_;
  EventExecutor event_executor_;
  std::unique_ptr<VehicleStatusMonitor> vehicle_monitor_;
  std::vector<FaultInfo> external_vehicle_faults_;
  rclcpp::Time last_vehicle_state_time_{0, 0, RCL_ROS_TIME};
  std::map<std::string, bool> external_node_active_;
  std::map<std::pair<std::string, std::string>, TransformInfo> external_tf_info_;
  rclcpp::Time last_node_tf_state_time_{0, 0, RCL_ROS_TIME};
  std::vector<FaultInfo> external_feedback_faults_;
  std::vector<FaultInfo> external_collision_faults_;
  bool external_battery_state_seen_{false};
  bool external_battery_source_has_data_{false};
  bool external_battery_source_stale_{true};
  rclcpp::Time last_monitor_battery_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_feedback_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_collision_state_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__NAV2_MONITOR_AGGREGATOR_NODE_HPP_
