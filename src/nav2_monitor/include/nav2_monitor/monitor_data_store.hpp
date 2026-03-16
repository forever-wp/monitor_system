#ifndef NAV2_MONITOR__MONITOR_DATA_STORE_HPP_
#define NAV2_MONITOR__MONITOR_DATA_STORE_HPP_

#include <deque>
#include <map>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace nav2_monitor
{

struct CollisionPoint
{
  double x{0.0};
  double y{0.0};
  double weight{1.0};
};

struct TopicRuntimeState
{
  bool has_publisher{false};
  bool has_valid_data{false};
  rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
  std::deque<rclcpp::Time> msg_times;
  double frequency{0.0};
  size_t empty_msg_count{0};
};

struct FeedbackRuntimeState
{
  double last_value{0.0};
  bool last_valid{false};
  bool received{false};
  rclcpp::Time first_seen{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
  std::deque<rclcpp::Time> msg_times;
};

struct ChassisRuntimeState
{
  bool command_received{false};
  double command_speed{0.0};
  rclcpp::Time command_stamp{0, 0, RCL_ROS_TIME};
  bool moto_received{false};
  bool moto_valid{false};
  double left_speed_rad{0.0};
  double right_speed_rad{0.0};
  rclcpp::Time moto_stamp{0, 0, RCL_ROS_TIME};
  bool odom_received{false};
  double odom_speed{0.0};
  rclcpp::Time odom_stamp{0, 0, RCL_ROS_TIME};
};


struct CollisionRuntimeState
{
  bool has_data{false};
  rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
  std::vector<CollisionPoint> points;
};

struct BatteryRuntimeState
{
  bool has_data{false};
  rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
  float temperature{0.0F};
  float percentage{0.0F};
};

class MonitorDataStore
{
public:
  void mark_node_seen(const std::string & node_name, const rclcpp::Time & now);
  void set_node_active(const std::string & node_name, bool active, const rclcpp::Time & now);
  void set_watch_topic_publisher(const std::string & topic, bool has_publisher);
  void set_watch_topic_frequency(const std::string & topic, double frequency);
  void add_watch_topic_sample(const std::string & topic, const rclcpp::Time & now, bool valid_data);
  void add_feedback_sample(
    const std::string & module_name, const std::string & source_topic,
    const std::string & metric_name, double value, bool valid, const rclcpp::Time & stamp);
  void set_command_speed(double speed, const rclcpp::Time & stamp);
  void set_moto_speed(double left_speed_rad, double right_speed_rad, bool valid, const rclcpp::Time & stamp);
  void set_odom_speed(double linear_speed, const rclcpp::Time & stamp);
  void set_battery_state(float temperature, float percentage, const rclcpp::Time & stamp);
  void set_collision_points(const std::vector<CollisionPoint> & points, const rclcpp::Time & stamp);
  void set_collision_points(const std::string & source_name, const std::vector<CollisionPoint> & points, const rclcpp::Time & stamp);

  bool is_node_active(const std::string & node_name, const rclcpp::Time & now, double timeout_s) const;
  double get_watch_topic_frequency(const std::string & topic) const;
  const TopicRuntimeState * get_watch_topic_state(const std::string & topic) const;
  const FeedbackRuntimeState * get_feedback_state(const std::string & feedback_key) const;
  const ChassisRuntimeState & get_chassis_state() const;
  const BatteryRuntimeState & get_battery_state() const;
  const CollisionRuntimeState & get_collision_state() const;
  std::vector<CollisionPoint> get_collision_points(const rclcpp::Time & now, double timeout_s) const;

private:
  mutable std::mutex mtx_;
  static std::string make_feedback_key(
    const std::string & module_name, const std::string & source_topic,
    const std::string & metric_name);
  static void trim_time_window(std::deque<rclcpp::Time> & msg_times, size_t max_size);
  static double calc_frequency(const std::deque<rclcpp::Time> & msg_times);
  std::vector<CollisionPoint> collect_collision_points_locked(const rclcpp::Time & now, double timeout_s) const;

  std::map<std::string, rclcpp::Time> node_last_seen_;
  std::map<std::string, TopicRuntimeState> watch_topics_;
  std::map<std::string, FeedbackRuntimeState> feedback_states_;
  ChassisRuntimeState chassis_state_;
  BatteryRuntimeState battery_state_;
  CollisionRuntimeState collision_state_;
  std::map<std::string, CollisionRuntimeState> collision_sources_;
};

}  // namespace nav2_monitor

#endif  // NAV2_MONITOR__MONITOR_DATA_STORE_HPP_
