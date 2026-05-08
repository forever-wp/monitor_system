#include "nav2_monitor/monitor_data_store.hpp"

#include <algorithm>
#include <cmath>
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

void MonitorDataStore::trim_receive_window(
  std::deque<rclcpp::Time> & receive_times,
  const rclcpp::Time & now,
  double min_hz)
{
  const double window_s = receive_window_s(min_hz);
  while (!receive_times.empty() && (now - receive_times.front()).seconds() > window_s) {
    receive_times.pop_front();
  }
  while (receive_times.size() > 100U) {
    receive_times.pop_front();
  }
}

double MonitorDataStore::receive_window_s(double min_hz)
{
  const double base_window_s = 2.0;
  if (min_hz <= 0.0) {
    return 30.0;
  }

  const double low_rate_window_s = min_hz < 1.0 ? 2.0 / min_hz : base_window_s;
  return std::clamp(low_rate_window_s, base_window_s, 30.0);
}

double MonitorDataStore::receive_gap_timeout_s(double min_hz)
{
  if (min_hz <= 0.0) {
    return 0.0;
  }

  // Give high-rate topics room for scheduler/DDS jitter while still detecting stop quickly.
  return std::clamp(3.0 / min_hz, 0.3, 2.0);
}

double MonitorDataStore::calc_receive_frequency(
  const std::deque<rclcpp::Time> & receive_times,
  const rclcpp::Time & now,
  double min_hz)
{
  if (receive_times.size() < 2) {
    return 0.0;
  }

  const double max_gap_s = receive_gap_timeout_s(min_hz);
  if (max_gap_s > 0.0 && (now - receive_times.back()).seconds() > max_gap_s) {
    return 0.0;
  }

  const double window_s = receive_window_s(min_hz);
  size_t first_idx = 0;
  while (
    first_idx + 1 < receive_times.size() &&
    (now - receive_times[first_idx]).seconds() > window_s)
  {
    ++first_idx;
  }

  if (receive_times.size() - first_idx < 2) {
    return 0.0;
  }

  const double span = (receive_times.back() - receive_times[first_idx]).seconds();
  if (span <= 0.0) {
    return 0.0;
  }

  return static_cast<double>(receive_times.size() - first_idx - 1) / span;
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
  auto & state = watch_topics_[topic];
  state.frequency = frequency;
  state.has_frequency_override = true;
  if (frequency > 0.0) {
    state.has_valid_data = true;
  }
}

void MonitorDataStore::add_watch_topic_sample(
  const std::string & topic,
  const rclcpp::Time & sample_stamp,
  const rclcpp::Time & receive_time,
  bool valid_data)
{
  std::lock_guard<std::mutex> lock(mtx_);
  auto & state = watch_topics_[topic];
  if (!valid_data) {
    state.empty_msg_count++;
    return;
  }

  state.has_valid_data = true;
  state.has_frequency_override = false;
  state.last_seen = sample_stamp;
  state.last_received = receive_time;
  state.msg_times.push_back(sample_stamp);
  state.receive_times.push_back(receive_time);
  trim_time_window(state.msg_times, 10);
  trim_receive_window(state.receive_times, receive_time, 0.0);
  state.frequency = calc_receive_frequency(state.receive_times, receive_time, 0.0);
}

void MonitorDataStore::add_feedback_sample(
  const std::string & module_name, const std::string & source_topic,
  const std::string & metric_name,
  double value,
  bool valid,
  const rclcpp::Time & stamp,
  const rclcpp::Time & receive_time)
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
  state.last_received = receive_time;
  state.msg_times.push_back(stamp);
  state.receive_times.push_back(receive_time);
  trim_time_window(state.msg_times, 10);
  trim_receive_window(state.receive_times, receive_time, 0.0);
}

void MonitorDataStore::set_command_speed(double speed, const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mtx_);
  chassis_state_.command_received = true;
  chassis_state_.command_speed = speed;
  chassis_state_.command_stamp = stamp;
}

void MonitorDataStore::set_prediction_speed(double speed, const rclcpp::Time & stamp)
{
  set_prediction_motion(speed, 0.0, 0.0, stamp);
}

void MonitorDataStore::set_prediction_motion(
  double linear_x, double linear_y, double angular_z, const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mtx_);
  chassis_state_.prediction_speed_received = true;
  chassis_state_.prediction_linear_x = linear_x;
  chassis_state_.prediction_linear_y = linear_y;
  chassis_state_.prediction_angular_z = angular_z;
  chassis_state_.prediction_speed = std::sqrt(linear_x * linear_x + linear_y * linear_y);
  chassis_state_.prediction_speed_stamp = stamp;
}

void MonitorDataStore::set_imu_motion(double speed_estimate, double yaw_rate, const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mtx_);
  chassis_state_.imu_received = true;
  chassis_state_.imu_speed_estimate = speed_estimate;
  chassis_state_.imu_yaw_rate = yaw_rate;
  chassis_state_.imu_stamp = stamp;
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

void MonitorDataStore::set_collision_voxels(
  const std::vector<CollisionVoxel> & cells, const rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mtx_);
  collision_voxel_state_.has_data = true;
  collision_voxel_state_.last_seen = stamp;
  collision_voxel_state_.cells = cells;
}

bool MonitorDataStore::is_node_active(
  const std::string & node_name, const rclcpp::Time & now, double timeout_s) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  const auto it = node_last_seen_.find(node_name);
  return it != node_last_seen_.end() && (now - it->second).seconds() <= timeout_s;
}

double MonitorDataStore::get_watch_topic_frequency(
  const std::string & topic,
  const rclcpp::Time & now,
  double min_hz) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  const auto it = watch_topics_.find(topic);
  if (it == watch_topics_.end()) {
    return 0.0;
  }
  if (it->second.has_frequency_override) {
    return it->second.frequency;
  }
  return calc_receive_frequency(it->second.receive_times, now, min_hz);
}

std::optional<TopicRuntimeState> MonitorDataStore::get_watch_topic_state(const std::string & topic) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  const auto it = watch_topics_.find(topic);
  return it == watch_topics_.end() ? std::nullopt : std::optional<TopicRuntimeState>{it->second};
}

std::optional<FeedbackRuntimeState> MonitorDataStore::get_feedback_state(const std::string & feedback_key) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  const auto it = feedback_states_.find(feedback_key);
  return it == feedback_states_.end() ? std::nullopt : std::optional<FeedbackRuntimeState>{it->second};
}

ChassisRuntimeState MonitorDataStore::get_chassis_state() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return chassis_state_;
}

BatteryRuntimeState MonitorDataStore::get_battery_state() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return battery_state_;
}

CollisionRuntimeState MonitorDataStore::get_collision_state() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return collision_state_;
}

CollisionVoxelRuntimeState MonitorDataStore::get_collision_voxel_state() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return collision_voxel_state_;
}

std::vector<CollisionPoint> MonitorDataStore::get_collision_points(const rclcpp::Time & now, double timeout_s) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return collect_collision_points_locked(now, timeout_s);
}

std::vector<CollisionVoxel> MonitorDataStore::get_collision_voxels(
  const rclcpp::Time & now, double timeout_s) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!collision_voxel_state_.has_data) {
    return {};
  }
  if ((now - collision_voxel_state_.last_seen).seconds() > timeout_s) {
    return {};
  }
  return collision_voxel_state_.cells;
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
