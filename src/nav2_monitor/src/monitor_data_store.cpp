#include "nav2_monitor/monitor_data_store.hpp"

#include <limits>

namespace nav2_monitor
{

std::string MonitorDataStore::make_feedback_key(
  const std::string & module_name, const std::string & source_topic,
  const std::string & metric_name)
{
  return module_name + "|feedback:" + source_topic + ":" + metric_name;
}

void MonitorDataStore::trim_time_window(std::deque<rclcpp::Time> & msg_times, size_t max_size)
{
  while (msg_times.size() > max_size) {
    msg_times.pop_front();
  }
}

double MonitorDataStore::calc_frequency(const std::deque<rclcpp::Time> & msg_times)
{
  if (msg_times.size() < 2) {
    return 0.0;
  }

  const double span = (msg_times.back() - msg_times.front()).seconds();
  if (span <= 0.0) {
    return 0.0;
  }

  return static_cast<double>(msg_times.size() - 1) / span;
}

void MonitorDataStore::mark_node_seen(const std::string & node_name, const rclcpp::Time & now)
{
  std::lock_guard<std::mutex> lock(mtx_);
  node_last_seen_[node_name] = now;
}

void MonitorDataStore::set_node_active(const std::string & node_name, bool active, const rclcpp::Time & now)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (active) {
    node_last_seen_[node_name] = now;
  } else {
    node_last_seen_.erase(node_name);
  }
}

void MonitorDataStore::set_watch_topic_publisher(const std::string & topic, bool has_publisher)
{
  std::lock_guard<std::mutex> lock(mtx_);
  watch_topics_[topic].has_publisher = has_publisher;
}

void MonitorDataStore::set_watch_topic_frequency(const std::string & topic, double frequency)
{
  std::lock_guard<std::mutex> lock(mtx_);
  watch_topics_[topic].frequency = frequency;
  if (frequency > 0.0) {
    watch_topics_[topic].has_valid_data = true;
  }
}

void MonitorDataStore::add_watch_topic_sample(
  const std::string & topic, const rclcpp::Time & now, bool valid_data)
{
  std::lock_guard<std::mutex> lock(mtx_);
  auto & state = watch_topics_[topic];
  if (!valid_data) {
    state.empty_msg_count++;
    return;
  }

  state.has_valid_data = true;
  state.last_seen = now;
  state.msg_times.push_back(now);
  trim_time_window(state.msg_times, 10);
  state.frequency = calc_frequency(state.msg_times);
}

void MonitorDataStore::add_feedback_sample(
  const std::string & module_name, const std::string & source_topic,
  const std::string & metric_name, double value, bool valid, const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mtx_);
  auto & state = feedback_states_[make_feedback_key(module_name, source_topic, metric_name)];
  if (!state.received) {
    state.first_seen = stamp;
  }
  state.received = true;
  state.last_value = value;
  state.last_valid = valid;
  state.last_seen = stamp;
  state.msg_times.push_back(stamp);
  trim_time_window(state.msg_times, 10);
}

void MonitorDataStore::set_command_speed(double speed, const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mtx_);
  chassis_state_.command_received = true;
  chassis_state_.command_speed = speed;
  chassis_state_.command_stamp = stamp;
}

void MonitorDataStore::set_moto_speed(
  double left_speed_rad, double right_speed_rad, bool valid, const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mtx_);
  chassis_state_.moto_received = true;
  chassis_state_.moto_valid = valid;
  chassis_state_.left_speed_rad = left_speed_rad;
  chassis_state_.right_speed_rad = right_speed_rad;
  chassis_state_.moto_stamp = stamp;
}

void MonitorDataStore::set_odom_speed(double linear_speed, const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mtx_);
  chassis_state_.odom_received = true;
  chassis_state_.odom_speed = linear_speed;
  chassis_state_.odom_stamp = stamp;
}

void MonitorDataStore::set_battery_state(float temperature, float percentage, const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mtx_);
  battery_state_.has_data = true;
  battery_state_.last_seen = stamp;
  battery_state_.temperature = temperature;
  battery_state_.percentage = percentage;
}

void MonitorDataStore::set_collision_points(const std::vector<CollisionPoint> & points, const rclcpp::Time & stamp)
{
  set_collision_points("default", points, stamp);
}

void MonitorDataStore::set_collision_points(
  const std::string & source_name, const std::vector<CollisionPoint> & points, const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mtx_);
  auto & source_state = collision_sources_[source_name];
  source_state.has_data = true;
  source_state.last_seen = stamp;
  source_state.points = points;

  collision_state_.has_data = true;
  collision_state_.last_seen = stamp;
  collision_state_.points = collect_collision_points_locked(stamp, std::numeric_limits<double>::max());
}

bool MonitorDataStore::is_node_active(
  const std::string & node_name, const rclcpp::Time & now, double timeout_s) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  const auto it = node_last_seen_.find(node_name);
  return it != node_last_seen_.end() && (now - it->second).seconds() <= timeout_s;
}

double MonitorDataStore::get_watch_topic_frequency(const std::string & topic) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  const auto it = watch_topics_.find(topic);
  return it == watch_topics_.end() ? 0.0 : it->second.frequency;
}

const TopicRuntimeState * MonitorDataStore::get_watch_topic_state(const std::string & topic) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  const auto it = watch_topics_.find(topic);
  return it == watch_topics_.end() ? nullptr : &it->second;
}

const FeedbackRuntimeState * MonitorDataStore::get_feedback_state(const std::string & feedback_key) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  const auto it = feedback_states_.find(feedback_key);
  return it == feedback_states_.end() ? nullptr : &it->second;
}

const ChassisRuntimeState & MonitorDataStore::get_chassis_state() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return chassis_state_;
}

const BatteryRuntimeState & MonitorDataStore::get_battery_state() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return battery_state_;
}

const CollisionRuntimeState & MonitorDataStore::get_collision_state() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return collision_state_;
}

std::vector<CollisionPoint> MonitorDataStore::get_collision_points(const rclcpp::Time & now, double timeout_s) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return collect_collision_points_locked(now, timeout_s);
}

std::vector<CollisionPoint> MonitorDataStore::collect_collision_points_locked(
  const rclcpp::Time & now, double timeout_s) const
{
  std::vector<CollisionPoint> points;
  for (const auto & [source_name, state] : collision_sources_) {
    (void)source_name;
    if (!state.has_data) {
      continue;
    }
    if ((now - state.last_seen).seconds() > timeout_s) {
      continue;
    }
    points.insert(points.end(), state.points.begin(), state.points.end());
  }
  return points;
}

}  // namespace nav2_monitor
