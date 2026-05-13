#ifndef NAV2_MONITOR__NODE_TF_MONITOR_NODE_HPP_
#define NAV2_MONITOR__NODE_TF_MONITOR_NODE_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "nav2_monitor/fault_detector.hpp"

namespace nav2_monitor
{

class NodeTfMonitorNode : public rclcpp::Node
{
public:
  NodeTfMonitorNode();

private:
  void load_configuration();
  void configure_profile_subscription();
  void on_config_profile(const std_msgs::msg::String::SharedPtr msg);
  void publish_state();
  static std::string json_escape(const std::string & input);
  static std::string normalize_graph_name(const std::string & name);
  static std::string basename_graph_name(const std::string & name);

  std::string fault_config_path_{"/opt/ry/config/Monitor/nav2_monitor/fault_detector_config.yaml"};
  std::string resolved_fault_config_path_;
  std::string config_profile_topic_{"/monitor/config_profile"};
  std::string publish_topic_{"/monitor/node_tf_state"};
  double scan_rate_hz_{2.0};
  double timeout_s_{5.0};

  FaultDetector fault_detector_;
  std::vector<std::string> monitored_nodes_;
  std::vector<std::pair<std::string, std::string>> monitored_transforms_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_profile_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__NODE_TF_MONITOR_NODE_HPP_
