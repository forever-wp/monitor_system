#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <collision_voxel_layer/msg/voxel_grid.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

#include "nav2_monitor/collision_prediction_router.hpp"
#include "nav2_monitor/config_profile_sync.hpp"
#include "nav2_monitor/fault_detector.hpp"
#include "nav2_monitor/monitor_data_store.hpp"
#include "nav2_monitor/monitor_state_json.hpp"

namespace
{
std::string resolve_config_path(const std::string & config_file)
{
  if (config_file.empty()) {
    return config_file;
  }

  namespace fs = std::filesystem;
  const fs::path input(config_file);
  if (input.is_absolute() && fs::exists(input)) {
    return input.string();
  }
  if (fs::exists(input)) {
    return fs::absolute(input).lexically_normal().string();
  }

  try {
    const fs::path package_share =
      ament_index_cpp::get_package_share_directory("nav2_monitor");
    const std::array<fs::path, 2> candidates = {
      package_share / input,
      package_share / "config" / input
    };
    for (const auto & candidate : candidates) {
      if (fs::exists(candidate)) {
        return candidate.lexically_normal().string();
      }
    }
  } catch (const std::exception &) {
  }

  return config_file;
}

bool extract_ultrasonic_distances(const YAML::Node & node, std::vector<double> & distances)
{
  distances.clear();
  if (!node || !node.IsSequence()) {
    return false;
  }

  size_t numeric_count = 0;
  for (const auto & item : node) {
    try {
      distances.push_back(item.as<double>());
      ++numeric_count;
    } catch (...) {
      if (distances.empty() && item.IsScalar()) {
        try {
          (void)item.as<bool>();
          continue;
        } catch (...) {
        }
      }
      distances.clear();
      return false;
    }
  }
  return numeric_count >= 8;
}

bool parse_ultrasonic_payload(
  const std::string & payload,
  const std::string & distances_key,
  std::vector<double> & distances)
{
  distances.clear();
  if (payload.empty()) {
    return false;
  }

  try {
    const auto root = YAML::Load(payload);
    if (root.IsMap()) {
      if (!distances_key.empty() && root[distances_key] &&
        extract_ultrasonic_distances(root[distances_key], distances))
      {
        return true;
      }
      for (const auto & entry : root) {
        if (extract_ultrasonic_distances(entry.second, distances)) {
          return true;
        }
      }
    } else if (extract_ultrasonic_distances(root, distances)) {
      return true;
    }
  } catch (...) {
  }

  std::stringstream ss(payload);
  std::string token;
  std::vector<std::string> tokens;
  while (std::getline(ss, token, ',')) {
    const auto begin = token.find_first_not_of(" \t\r\n");
    const auto end = token.find_last_not_of(" \t\r\n");
    if (begin == std::string::npos) {
      continue;
    }
    tokens.push_back(token.substr(begin, end - begin + 1));
  }

  size_t start_idx = 0;
  if (!tokens.empty()) {
    std::string lowered = tokens.front();
    std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if ((lowered == "true" || lowered == "false") && tokens.size() >= 9) {
      start_idx = 1;
    }
  }
  if (tokens.size() < start_idx + 8) {
    return false;
  }

  try {
    for (size_t idx = 0; idx < 8; ++idx) {
      distances.push_back(std::stod(tokens[start_idx + idx]));
    }
    return true;
  } catch (...) {
    distances.clear();
    return false;
  }
}
}  // namespace

namespace nav2_monitor
{

class CollisionMonitorNode : public rclcpp::Node
{
public:
  CollisionMonitorNode()
  : Node("collision_monitor"), fault_detector_(this), tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    fault_config_path_ = declare_parameter<std::string>(
      "fault_config", "/opt/ry/config/Monitor/nav2_monitor/fault_detector_config.yaml");
    config_profile_topic_ = declare_parameter<std::string>(
      "config_profile_topic", "/monitor/config_profile");
    publish_topic_ = declare_parameter<std::string>("publish_topic", "/monitor/collision_state");
    check_rate_hz_ = std::max(5.0, declare_parameter<double>("check_rate_hz", 20.0));
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");

    load_configuration();

    state_pub_ = create_publisher<std_msgs::msg::String>(
      publish_topic_, rclcpp::QoS(1).reliable().transient_local());
    config_profile_sub_ = create_subscription<std_msgs::msg::String>(
      config_profile_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        on_config_profile(msg);
      });
    configure_publishers();
    configure_subscriptions();

    const auto period = std::chrono::duration<double>(1.0 / check_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { evaluate_and_publish(); });

    RCLCPP_INFO(
      get_logger(),
      "collision_monitor started: publish=%s voxel=%s scan=%s pointcloud=%s ultrasonic=%s",
      publish_topic_.c_str(),
      cfg_.voxel_topic.empty() ? "<disabled>" : cfg_.voxel_topic.c_str(),
      cfg_.scan_topic.empty() ? "<disabled>" : cfg_.scan_topic.c_str(),
      cfg_.pointcloud_topic.empty() ? "<disabled>" : cfg_.pointcloud_topic.c_str(),
      cfg_.ultrasonic_topic.empty() ? "<disabled>" : cfg_.ultrasonic_topic.c_str());
  }

private:
  void load_configuration()
  {
    resolved_fault_config_path_ = resolve_config_path(fault_config_path_);
    fault_detector_.load_config(resolved_fault_config_path_);
    cfg_ = fault_detector_.get_collision_detection_config();
    router_ = CollisionPredictionRouter(CollisionPredictionRoutingConfig{
        cfg_.prediction_speed_topic,
        cfg_.control_source_state_topic,
        cfg_.prediction_speed_navigation_topic,
        cfg_.prediction_speed_miniapp_topic,
        cfg_.prediction_speed_remote_topic,
        cfg_.prediction_speed_other_topic});
  }

  void on_config_profile(const std_msgs::msg::String::SharedPtr msg)
  {
    ConfigProfileUpdate update;
    if (!parse_config_profile_update(msg->data, update)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignore invalid config profile message: %s", msg->data.c_str());
      return;
    }

    const auto resolved = resolve_config_path(update.fault_config);
    if (update.fault_config == fault_config_path_ && resolved == resolved_fault_config_path_) {
      return;
    }

    fault_config_path_ = update.fault_config;
    reset_runtime();
    load_configuration();
    configure_publishers();
    configure_subscriptions();
    RCLCPP_WARN(
      get_logger(),
      "Reload collision_monitor config profile: task=%s fault_config=%s",
      update.task_name.c_str(), fault_config_path_.c_str());
  }

  void reset_runtime()
  {
    navigation_mode_pub_.reset();
    collision_ttc_markers_pub_.reset();
    collision_zone_pubs_.clear();
    control_source_sub_.reset();
    prediction_subs_.clear();
    voxel_sub_.reset();
    scan_sub_.reset();
    pointcloud_sub_.reset();
    ultrasonic_sub_.reset();
    navigation_safe_mode_active_ = false;
    navigation_ttc_abnormal_ = false;
    navigation_ttc_abnormal_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    navigation_ttc_clear_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    navigation_safe_mode_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_navigation_mode_publish_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    fault_detector_.set_collision_navigation_safe_mode(false);
  }

  rclcpp::Time stamp_or_now(const builtin_interfaces::msg::Time & stamp) const
  {
    return (stamp.sec != 0 || stamp.nanosec != 0) ? rclcpp::Time(stamp) : now();
  }

  void configure_publishers()
  {
    if (!cfg_.enabled) {
      return;
    }
    if (cfg_.navigation_mode_switch_enabled && !cfg_.navigation_mode_topic.empty()) {
      navigation_mode_pub_ = create_publisher<std_msgs::msg::String>(
        cfg_.navigation_mode_topic, rclcpp::QoS(1).reliable().transient_local());
      publish_navigation_mode(true);
    }
    for (const auto & zone : cfg_.zones) {
      if (zone.model == CollisionModelType::TTC || !zone.visualize || zone.points.size() < 3) {
        continue;
      }
      collision_zone_pubs_[zone.name] = create_publisher<geometry_msgs::msg::PolygonStamped>(
        zone.polygon_pub_topic, rclcpp::QoS(1).reliable().transient_local());
    }
    if (cfg_.ttc_visualization_enabled) {
      collision_ttc_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/nav2_monitor/collision_ttc_markers", rclcpp::QoS(1).reliable().transient_local());
    }
  }

  void configure_subscriptions()
  {
    if (!cfg_.enabled) {
      return;
    }
    if (!router_.control_source_state_topic().empty()) {
      control_source_sub_ = create_subscription<std_msgs::msg::String>(
        router_.control_source_state_topic(), rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::String::SharedPtr msg) {
          on_control_source(msg);
        });
    }
    for (const auto & route : router_.subscribed_sources()) {
      prediction_subs_[route.source] = create_subscription<geometry_msgs::msg::Twist>(
        route.topic, rclcpp::QoS(20).reliable().durability_volatile(),
        [this, source = route.source, topic = route.topic](geometry_msgs::msg::Twist::SharedPtr msg) {
          on_prediction_cmd_vel(source, topic, msg);
        });
    }
    if (!cfg_.voxel_topic.empty()) {
      voxel_sub_ = create_subscription<collision_voxel_layer::msg::VoxelGrid>(
        cfg_.voxel_topic, rclcpp::QoS(1).reliable().transient_local(),
        [this](collision_voxel_layer::msg::VoxelGrid::SharedPtr msg) {
          on_voxel_grid(msg);
        });
    }
    if (!cfg_.scan_topic.empty()) {
      scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        cfg_.scan_topic, rclcpp::SensorDataQoS().keep_last(10),
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
          on_scan(msg);
        });
    }
    if (!cfg_.pointcloud_topic.empty()) {
      pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cfg_.pointcloud_topic, rclcpp::SensorDataQoS().keep_last(3),
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          on_pointcloud(msg);
        });
    }
    if (!cfg_.ultrasonic_topic.empty()) {
      ultrasonic_sub_ = create_subscription<std_msgs::msg::String>(
        cfg_.ultrasonic_topic, rclcpp::QoS(10).reliable().durability_volatile(),
        [this](std_msgs::msg::String::SharedPtr msg) {
          on_ultrasonic(msg);
        });
    }
  }

  void on_control_source(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto normalized = CollisionPredictionRouter::normalize_source(msg->data);
    if (!router_.is_known_source(normalized)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignore TTC control source state '%s'", msg->data.c_str());
      return;
    }
    if (router_.update_active_source(normalized)) {
      RCLCPP_INFO(
        get_logger(), "TTC control source switched: source=%s topic=%s",
        router_.active_source().c_str(), router_.active_topic().c_str());
    }
  }

  void on_prediction_cmd_vel(
    const std::string & source,
    const std::string & topic,
    const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!router_.should_accept_source(source)) {
      return;
    }
    const auto motion = CollisionPredictionRouter::extract_prediction_motion(source, *msg);
    data_store_.set_prediction_motion(
      motion.linear_x, motion.linear_y, motion.angular_z, now());
    RCLCPP_DEBUG(
      get_logger(), "TTC prediction input: source=%s topic=%s", source.c_str(), topic.c_str());
  }

  void on_voxel_grid(const collision_voxel_layer::msg::VoxelGrid::SharedPtr msg)
  {
    std::vector<CollisionVoxel> cells;
    cells.reserve(msg->cells.size());
    for (const auto & cell : msg->cells) {
      cells.push_back(CollisionVoxel{
        cell.x,
        cell.y,
        cell.z,
        cell.occupancy,
        cell.source_mask});
    }
    data_store_.set_collision_voxels(cells, stamp_or_now(msg->header.stamp));
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::vector<CollisionPoint> points;
    points.reserve(msg->ranges.size());
    double tx = 0.0;
    double ty = 0.0;
    double yaw = 0.0;
    bool have_tf = msg->header.frame_id.empty() || msg->header.frame_id == base_frame_id_;
    if (!have_tf) {
      try {
        auto tf = tf_buffer_.lookupTransform(base_frame_id_, msg->header.frame_id, tf2::TimePointZero);
        tx = tf.transform.translation.x;
        ty = tf.transform.translation.y;
        tf2::Quaternion q(
          tf.transform.rotation.x,
          tf.transform.rotation.y,
          tf.transform.rotation.z,
          tf.transform.rotation.w);
        double roll = 0.0;
        double pitch = 0.0;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        have_tf = true;
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Collision scan transform failed: %s", e.what());
        return;
      }
    }

    double angle = msg->angle_min;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    for (const auto range : msg->ranges) {
      if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max) {
        angle += msg->angle_increment;
        continue;
      }
      const double sx = range * std::cos(angle);
      const double sy = range * std::sin(angle);
      if (have_tf && msg->header.frame_id != base_frame_id_) {
        points.push_back(CollisionPoint{
          cos_yaw * sx - sin_yaw * sy + tx,
          sin_yaw * sx + cos_yaw * sy + ty,
          1.0});
      } else {
        points.push_back(CollisionPoint{sx, sy, 1.0});
      }
      angle += msg->angle_increment;
    }
    data_store_.set_collision_points("scan", points, stamp_or_now(msg->header.stamp));
  }

  void on_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::vector<CollisionPoint> points;
    points.reserve(msg->width * msg->height);

    tf2::Transform transform;
    bool have_tf = msg->header.frame_id.empty() || msg->header.frame_id == base_frame_id_;
    if (!have_tf) {
      try {
        auto tf = tf_buffer_.lookupTransform(base_frame_id_, msg->header.frame_id, tf2::TimePointZero);
        tf2::Quaternion q(
          tf.transform.rotation.x,
          tf.transform.rotation.y,
          tf.transform.rotation.z,
          tf.transform.rotation.w);
        transform.setOrigin(tf2::Vector3(
          tf.transform.translation.x,
          tf.transform.translation.y,
          tf.transform.translation.z));
        transform.setRotation(q);
        have_tf = true;
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Collision pointcloud transform failed: %s", e.what());
        return;
      }
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
        continue;
      }
      tf2::Vector3 point_xyz(*iter_x, *iter_y, *iter_z);
      if (have_tf && msg->header.frame_id != base_frame_id_) {
        point_xyz = transform * point_xyz;
      }
      if (cfg_.pointcloud_height_filter_enabled &&
        (point_xyz.z() < cfg_.pointcloud_min_height || point_xyz.z() > cfg_.pointcloud_max_height))
      {
        continue;
      }
      points.push_back(CollisionPoint{point_xyz.x(), point_xyz.y(), 1.0});
    }
    data_store_.set_collision_points("pointcloud", points, stamp_or_now(msg->header.stamp));
  }

  void on_ultrasonic(const std_msgs::msg::String::SharedPtr msg)
  {
    if (cfg_.ultrasonic_sensors.empty()) {
      return;
    }
    std::vector<double> distances;
    if (!parse_ultrasonic_payload(msg->data, cfg_.ultrasonic_distances_key, distances)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Collision ultrasonic parse failed: topic=%s", cfg_.ultrasonic_topic.c_str());
      return;
    }

    std::vector<CollisionPoint> points;
    points.reserve(cfg_.ultrasonic_sensors.size());
    for (const auto & sensor : cfg_.ultrasonic_sensors) {
      if (!sensor.enabled || sensor.index >= distances.size()) {
        continue;
      }
      const double raw_distance = distances[sensor.index];
      if (!std::isfinite(raw_distance) || raw_distance <= 0.0) {
        continue;
      }
      if (raw_distance >= cfg_.ultrasonic_out_of_range_value || raw_distance > sensor.max_distance) {
        continue;
      }
      constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
      const double effective_distance = std::max(raw_distance, cfg_.ultrasonic_blind_distance);
      const double yaw = sensor.yaw_deg * kDegToRad;
      points.push_back(CollisionPoint{
        sensor.x + effective_distance * std::cos(yaw),
        sensor.y + effective_distance * std::sin(yaw),
        sensor.weight});
    }
    data_store_.set_collision_points("ultrasonic", points, now());
  }

  void evaluate_and_publish()
  {
    const auto now_time = now();
    auto faults = fault_detector_.detect_collision_faults(data_store_, now_time);
    update_navigation_mode_from_ttc(faults, now_time);
    publish_collision_zones();
    publish_collision_ttc_markers();

    std::string state = cfg_.enabled ? "OK" : "UNKNOWN";
    std::string summary = cfg_.enabled ? "collision monitor normal" : "collision_detection disabled";
    if (!faults.empty()) {
      state = fault_level_to_state_string(faults.front().level);
      summary = faults.front().reason;
    }

    std::ostringstream oss;
    oss << '{'
        << "\"stamp\":" << now_time.seconds() << ','
        << "\"source_module\":\"collision_monitor\","
        << "\"state\":\"" << state << "\","
        << "\"healthy\":" << (faults.empty() ? "true" : "false") << ','
        << "\"summary\":\"" << monitor_json_escape(summary) << "\","
        << "\"active_source\":\"" << monitor_json_escape(router_.active_source()) << "\","
        << "\"active_topic\":\"" << monitor_json_escape(router_.active_topic()) << "\","
        << "\"navigation_mode\":\""
        << (navigation_safe_mode_active_ ? cfg_.navigation_safe_mode : cfg_.navigation_fast_mode) << "\","
        << "\"faults\":" << faults_to_json_array(faults)
        << '}';
    std_msgs::msg::String msg;
    msg.data = oss.str();
    state_pub_->publish(msg);
  }

  void update_navigation_mode_from_ttc(
    const std::vector<FaultInfo> & faults,
    const rclcpp::Time & now_time)
  {
    if (!cfg_.enabled || !navigation_mode_pub_) {
      return;
    }

    bool ttc_abnormal = false;
    for (const auto & fault : faults) {
      if (fault.module_name == cfg_.module_name && fault.fault_type == "collision_detection" &&
        fault.fault_model == "ttc")
      {
        ttc_abnormal = true;
        break;
      }
    }

    if (ttc_abnormal) {
      if (!navigation_ttc_abnormal_) {
        navigation_ttc_abnormal_since_ = now_time;
      }
      navigation_ttc_abnormal_ = true;
      navigation_ttc_clear_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    } else {
      if (navigation_ttc_abnormal_ || navigation_ttc_clear_since_.nanoseconds() == 0) {
        navigation_ttc_clear_since_ = now_time;
      }
      navigation_ttc_abnormal_ = false;
      navigation_ttc_abnormal_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    if (!navigation_safe_mode_active_ && ttc_abnormal) {
      const double abnormal_duration = (now_time - navigation_ttc_abnormal_since_).seconds();
      if (abnormal_duration >= cfg_.navigation_safe_enter_duration_s) {
        navigation_safe_mode_active_ = true;
        navigation_safe_mode_since_ = now_time;
        fault_detector_.set_collision_navigation_safe_mode(true);
        publish_navigation_mode(true);
        RCLCPP_WARN(
          get_logger(), "Navigation mode switched to %s by TTC: duration=%.3fs",
          cfg_.navigation_safe_mode.c_str(), abnormal_duration);
      }
    }

    if (navigation_safe_mode_active_ && !ttc_abnormal) {
      const double clear_duration = navigation_ttc_clear_since_.nanoseconds() == 0 ?
        0.0 : (now_time - navigation_ttc_clear_since_).seconds();
      const double hold_duration = navigation_safe_mode_since_.nanoseconds() == 0 ?
        0.0 : (now_time - navigation_safe_mode_since_).seconds();
      if (clear_duration >= cfg_.navigation_safe_clear_duration_s &&
        hold_duration >= cfg_.navigation_safe_min_hold_s)
      {
        navigation_safe_mode_active_ = false;
        fault_detector_.set_collision_navigation_safe_mode(false);
        publish_navigation_mode(true);
        RCLCPP_WARN(
          get_logger(), "Navigation mode recovered to %s: clear_duration=%.3fs hold_duration=%.3fs",
          cfg_.navigation_fast_mode.c_str(), clear_duration, hold_duration);
      }
    }

    publish_navigation_mode(false);
  }

  void publish_navigation_mode(bool force)
  {
    if (!navigation_mode_pub_) {
      return;
    }
    const auto now_time = now();
    if (!force && last_navigation_mode_publish_time_.nanoseconds() != 0 &&
      (now_time - last_navigation_mode_publish_time_).seconds() < cfg_.navigation_mode_publish_cooldown_s)
    {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = navigation_safe_mode_active_ ? cfg_.navigation_safe_mode : cfg_.navigation_fast_mode;
    navigation_mode_pub_->publish(msg);
    last_navigation_mode_publish_time_ = now_time;
  }

  void publish_collision_zones()
  {
    for (const auto & zone : cfg_.zones) {
      if (zone.model == CollisionModelType::TTC || !zone.visualize || zone.points.size() < 3) {
        continue;
      }
      const auto pub_it = collision_zone_pubs_.find(zone.name);
      if (pub_it == collision_zone_pubs_.end()) {
        continue;
      }
      geometry_msgs::msg::PolygonStamped polygon_msg;
      polygon_msg.header.stamp = now();
      polygon_msg.header.frame_id = base_frame_id_;
      polygon_msg.polygon.points.reserve(zone.points.size());
      for (const auto & point : zone.points) {
        geometry_msgs::msg::Point32 p;
        p.x = static_cast<float>(point.x);
        p.y = static_cast<float>(point.y);
        p.z = 0.0F;
        polygon_msg.polygon.points.push_back(p);
      }
      pub_it->second->publish(polygon_msg);
    }
  }

  void publish_collision_ttc_markers()
  {
    if (!collision_ttc_markers_pub_) {
      return;
    }

    const auto chassis_state = data_store_.get_chassis_state();
    const bool prediction_speed_fresh = chassis_state.prediction_speed_received &&
      (now() - chassis_state.prediction_speed_stamp).seconds() <= cfg_.source_timeout_s;
    const std::string active_source = router_.active_source();
    const std::string active_topic = router_.active_topic();
    const auto & vis = fault_detector_.get_collision_ttc_visualization();
    visualization_msgs::msg::MarkerArray marker_array;
    const auto stamp_now = now();
    const auto stamp_ns = stamp_now.nanoseconds();
    builtin_interfaces::msg::Time marker_stamp;
    marker_stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    marker_stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.stamp = marker_stamp;
    clear_marker.header.frame_id = base_frame_id_;
    clear_marker.ns = "collision_ttc";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    if (!vis.enabled || !vis.active) {
      collision_ttc_markers_pub_->publish(marker_array);
      if (!prediction_speed_fresh) {
        const double age_s = chassis_state.prediction_speed_received ?
          (now() - chassis_state.prediction_speed_stamp).seconds() :
          -1.0;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TTC markers idle: source=%s topic=%s reason=no_fresh_prediction_speed age=%.3fs "
          "timeout=%.3fs",
          active_source.c_str(), active_topic.c_str(), age_s, cfg_.source_timeout_s);
      } else if (chassis_state.prediction_speed <= 1e-3) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TTC markers idle: source=%s topic=%s reason=zero_prediction_speed "
          "linear_x=%.3f linear_y=%.3f angular_z=%.3f",
          active_source.c_str(), active_topic.c_str(),
          chassis_state.prediction_linear_x,
          chassis_state.prediction_linear_y,
          chassis_state.prediction_angular_z);
      } else {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TTC markers idle: source=%s topic=%s reason=visualization_inactive",
          active_source.c_str(), active_topic.c_str());
      }
      return;
    }

    visualization_msgs::msg::Marker trajectory_marker;
    trajectory_marker.header.stamp = marker_stamp;
    trajectory_marker.header.frame_id = base_frame_id_;
    trajectory_marker.ns = "collision_ttc_trajectory";
    trajectory_marker.id = 1;
    trajectory_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajectory_marker.action = visualization_msgs::msg::Marker::ADD;
    trajectory_marker.scale.x = 0.03;
    trajectory_marker.color.r = 0.1F;
    trajectory_marker.color.g = 0.9F;
    trajectory_marker.color.b = 0.2F;
    trajectory_marker.color.a = 0.95F;
    for (const auto & point : vis.trajectory_points) {
      geometry_msgs::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = 0.03;
      trajectory_marker.points.push_back(p);
    }
    marker_array.markers.push_back(trajectory_marker);

    if (!vis.corridor_outline.empty()) {
      visualization_msgs::msg::Marker corridor_marker;
      corridor_marker.header.stamp = marker_stamp;
      corridor_marker.header.frame_id = base_frame_id_;
      corridor_marker.ns = "collision_ttc_corridor";
      corridor_marker.id = 4;
      corridor_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      corridor_marker.action = visualization_msgs::msg::Marker::ADD;
      corridor_marker.scale.x = 0.02;
      corridor_marker.color.r = 1.0F;
      corridor_marker.color.g = 0.65F;
      corridor_marker.color.b = 0.10F;
      corridor_marker.color.a = 0.70F;
      for (const auto & point : vis.corridor_outline) {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.02;
        corridor_marker.points.push_back(p);
      }
      geometry_msgs::msg::Point close_p;
      close_p.x = vis.corridor_outline.front().x;
      close_p.y = vis.corridor_outline.front().y;
      close_p.z = 0.02;
      corridor_marker.points.push_back(close_p);
      marker_array.markers.push_back(corridor_marker);
    }

    int footprint_marker_id = 100;
    for (const auto & polygon : vis.footprint_samples) {
      if (polygon.empty()) {
        continue;
      }
      visualization_msgs::msg::Marker footprint_marker;
      footprint_marker.header.stamp = marker_stamp;
      footprint_marker.header.frame_id = base_frame_id_;
      footprint_marker.ns = "collision_ttc_footprint";
      footprint_marker.id = footprint_marker_id++;
      footprint_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      footprint_marker.action = visualization_msgs::msg::Marker::ADD;
      footprint_marker.scale.x = 0.01;
      footprint_marker.color.r = 0.2F;
      footprint_marker.color.g = 0.7F;
      footprint_marker.color.b = 1.0F;
      footprint_marker.color.a = 0.45F;
      for (const auto & point : polygon) {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.01;
        footprint_marker.points.push_back(p);
      }
      geometry_msgs::msg::Point close_p;
      close_p.x = polygon.front().x;
      close_p.y = polygon.front().y;
      close_p.z = 0.01;
      footprint_marker.points.push_back(close_p);
      marker_array.markers.push_back(footprint_marker);
    }

    if (vis.ttc_s >= 0.0) {
      visualization_msgs::msg::Marker collision_point_marker;
      collision_point_marker.header.stamp = marker_stamp;
      collision_point_marker.header.frame_id = base_frame_id_;
      collision_point_marker.ns = "collision_ttc_point";
      collision_point_marker.id = 2;
      collision_point_marker.type = visualization_msgs::msg::Marker::SPHERE;
      collision_point_marker.action = visualization_msgs::msg::Marker::ADD;
      collision_point_marker.pose.position.x = vis.collision_point.x;
      collision_point_marker.pose.position.y = vis.collision_point.y;
      collision_point_marker.pose.position.z = 0.05;
      collision_point_marker.pose.orientation.w = 1.0;
      collision_point_marker.scale.x = 0.08;
      collision_point_marker.scale.y = 0.08;
      collision_point_marker.scale.z = 0.08;
      collision_point_marker.color.r = 1.0F;
      collision_point_marker.color.g = 0.2F;
      collision_point_marker.color.b = 0.1F;
      collision_point_marker.color.a = 0.95F;
      marker_array.markers.push_back(collision_point_marker);

      visualization_msgs::msg::Marker text_marker;
      text_marker.header.stamp = marker_stamp;
      text_marker.header.frame_id = base_frame_id_;
      text_marker.ns = "collision_ttc_text";
      text_marker.id = 3;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose.position.x = vis.collision_point.x;
      text_marker.pose.position.y = vis.collision_point.y;
      text_marker.pose.position.z = 0.35;
      text_marker.pose.orientation.w = 1.0;
      text_marker.scale.z = 0.12;
      text_marker.color.r = 1.0F;
      text_marker.color.g = 1.0F;
      text_marker.color.b = 1.0F;
      text_marker.color.a = 1.0F;
      std::ostringstream oss;
      if (!vis.zone_name.empty()) {
        oss << vis.zone_name << ' ';
      }
      oss << "ttc=" << std::fixed << std::setprecision(2) << vis.ttc_s;
      if (vis.min_clearance >= 0.0) {
        oss << " clr=" << std::fixed << std::setprecision(2) << vis.min_clearance;
      }
      text_marker.text = oss.str();
      marker_array.markers.push_back(text_marker);
    }

    collision_ttc_markers_pub_->publish(marker_array);
    if (vis.ttc_s >= 0.0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "TTC markers published: source=%s topic=%s zone=%s ttc=%.3fs threshold=%.3fs "
        "clearance=%.3fm markers=%zu",
        active_source.c_str(), active_topic.c_str(), vis.zone_name.c_str(),
        vis.ttc_s, vis.threshold_s, vis.min_clearance, marker_array.markers.size());
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TTC markers published: source=%s topic=%s preview_only=1 trajectory_points=%zu "
        "footprints=%zu markers=%zu",
        active_source.c_str(), active_topic.c_str(), vis.trajectory_points.size(),
        vis.footprint_samples.size(), marker_array.markers.size());
    }
  }

  std::string fault_config_path_;
  std::string resolved_fault_config_path_;
  std::string config_profile_topic_;
  std::string publish_topic_;
  std::string base_frame_id_{"base_link"};
  double check_rate_hz_{20.0};
  FaultDetector fault_detector_;
  MonitorDataStore data_store_;
  CollisionDetectionConfig cfg_;
  CollisionPredictionRouter router_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  bool navigation_safe_mode_active_{false};
  bool navigation_ttc_abnormal_{false};
  rclcpp::Time navigation_ttc_abnormal_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time navigation_ttc_clear_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time navigation_safe_mode_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_navigation_mode_publish_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr config_profile_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_mode_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr collision_ttc_markers_pub_;
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr> collision_zone_pubs_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_source_sub_;
  std::map<std::string, rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> prediction_subs_;
  rclcpp::Subscription<collision_voxel_layer::msg::VoxelGrid>::SharedPtr voxel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ultrasonic_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace nav2_monitor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_monitor::CollisionMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
