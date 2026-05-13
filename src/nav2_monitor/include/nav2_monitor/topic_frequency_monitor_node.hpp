#ifndef NAV2_MONITOR__TOPIC_FREQUENCY_MONITOR_NODE_HPP_
#define NAV2_MONITOR__TOPIC_FREQUENCY_MONITOR_NODE_HPP_

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace nav2_monitor
{

class TopicFrequencyMonitorNode : public rclcpp::Node
{
public:
  TopicFrequencyMonitorNode();

private:
  struct WatchedTopic
  {
    std::string topic;
    std::string type;
    double min_hz{0.0};
    double idle_timeout_s{0.0};
    bool type_discovered{false};
    rclcpp::Time last_received{0, 0, RCL_ROS_TIME};
    std::deque<rclcpp::Time> receive_times;
    rclcpp::GenericSubscription::SharedPtr subscription;
  };

  void load_parameters();
  void load_watched_topics_from_fault_config();
  void load_watched_topics_from_indexed_parameters();
  void configure_profile_subscription();
  void on_config_profile(const std_msgs::msg::String::SharedPtr msg);
  void refresh_topic_types_and_subscriptions();
  void subscribe_topics();
  void on_topic_message(const std::string & topic);
  void publish_states();
  std::string resolve_topic_type(const std::string & topic) const;
  std::string resolve_config_path(const std::string & config_file) const;
  double compute_frequency(WatchedTopic & watched, const rclcpp::Time & now) const;
  double compute_idle_timeout(double min_hz) const;
  rclcpp::QoS build_subscription_qos() const;
  static bool requires_frequency(const WatchedTopic & watched);
  static std::string json_escape(const std::string & input);

  double publish_rate_hz_{10.0};
  double frequency_window_s_{1.0};
  double default_idle_timeout_s_{0.5};
  double min_idle_timeout_s_{0.3};
  double max_idle_timeout_s_{2.0};
  std::string publish_topic_{"/monitor/topic_states"};
  std::string fault_config_path_{"/opt/ry/config/Monitor/nav2_monitor/fault_detector_config.yaml"};
  std::string resolved_fault_config_path_;
  std::string config_profile_topic_{"/monitor/config_profile"};
  std::vector<WatchedTopic> watched_topics_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_profile_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__TOPIC_FREQUENCY_MONITOR_NODE_HPP_
