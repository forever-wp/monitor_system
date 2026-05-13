#include "collision_voxel_layer/collision_voxel_layer_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_set>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/parameter_map.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker.hpp>

#include "collision_voxel_layer/voxel_types.hpp"

namespace collision_voxel_layer
{

namespace
{

constexpr uint8_t kScanSourceMask = 0x01U;
constexpr uint8_t kDepthSourceMask = 0x02U;
constexpr uint8_t kLidarSourceMask = 0x04U;

int32_t quantize(double value, double resolution)
{
  return static_cast<int32_t>(std::floor(value / resolution));
}

tf2::Transform make_transform(const geometry_msgs::msg::TransformStamped & transform_msg)
{
  const auto & translation = transform_msg.transform.translation;
  const auto & rotation = transform_msg.transform.rotation;

  tf2::Quaternion quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
  return tf2::Transform(
    quaternion,
    tf2::Vector3(translation.x, translation.y, translation.z));
}

std_msgs::msg::ColorRGBA make_color_for_source(uint8_t source_mask, float occupancy, float occupancy_max)
{
  std_msgs::msg::ColorRGBA color;
  const bool scan = (source_mask & 0x01U) != 0U;
  const bool depth = (source_mask & 0x02U) != 0U;
  const bool lidar = (source_mask & 0x04U) != 0U;
  const float normalized = std::clamp(
    occupancy_max > 0.0F ? occupancy / occupancy_max : 0.0F,
    0.0F, 1.0F);

  color.r = lidar ? 0.15F : 0.05F;
  color.g = depth ? 0.85F : 0.10F;
  color.b = scan ? 0.85F : 0.10F;
  if (scan && depth) {
    color.r = 0.15F;
    color.g = 0.75F;
    color.b = 0.95F;
  }
  if (scan && lidar) {
    color.r = 0.85F;
    color.g = 0.55F;
    color.b = 0.20F;
  }
  if (depth && lidar) {
    color.r = 0.80F;
    color.g = 0.20F;
    color.b = 0.85F;
  }
  if (scan && depth && lidar) {
    color.r = 0.95F;
    color.g = 0.95F;
    color.b = 0.25F;
  }
  color.a = std::max(0.25F, normalized);
  return color;
}

std::string normalize_graph_name(const std::string & name)
{
  if (name.empty()) {
    return name;
  }

  size_t pos = 0;
  while (pos < name.size() && name[pos] == '/') {
    ++pos;
  }
  if (pos >= name.size()) {
    return "";
  }
  return name.substr(pos);
}

std::string join_strings(const std::vector<std::string> & values, const std::string & delimiter)
{
  std::ostringstream stream;
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      stream << delimiter;
    }
    stream << values[i];
  }
  return stream.str();
}

std::vector<int64_t> expand_int_vector(
  const std::vector<int64_t> & values,
  std::size_t size,
  int64_t default_value)
{
  std::vector<int64_t> expanded(size, default_value);
  for (std::size_t i = 0; i < size; ++i) {
    if (i < values.size()) {
      expanded[i] = values[i];
    } else if (!values.empty()) {
      expanded[i] = values.back();
    }
  }
  return expanded;
}

std::vector<double> expand_double_vector(
  const std::vector<double> & values,
  std::size_t size,
  double default_value)
{
  std::vector<double> expanded(size, default_value);
  for (std::size_t i = 0; i < size; ++i) {
    if (i < values.size()) {
      expanded[i] = values[i];
    } else if (!values.empty()) {
      expanded[i] = values.back();
    }
  }
  return expanded;
}

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
      ament_index_cpp::get_package_share_directory("collision_voxel_layer");
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

}  // namespace

CollisionVoxelLayerNode::CollisionVoxelLayerNode(const rclcpp::NodeOptions & options)
: Node("collision_voxel_layer", options),
  tf_buffer_(this->get_clock())
{
  RuntimeConfig initial_config;
  initial_config.config_file = this->declare_parameter<std::string>("config_file", "");
  initial_config.config_reload_enabled = this->declare_parameter<bool>(
    "config_reload_enabled", true);
  initial_config.config_reload_period_s = std::max(
    0.1, this->declare_parameter<double>("config_reload_period_s", 1.0));
  initial_config.base_frame = this->declare_parameter<std::string>("base_frame", "base_link");
  initial_config.scan_topic = this->declare_parameter<std::string>("scan_topic", "/scan");
  initial_config.scan_topics = this->declare_parameter<std::vector<std::string>>(
    "scan_topics", std::vector<std::string>{initial_config.scan_topic});
  const bool scan_topic_overridden = std::any_of(
    options.parameter_overrides().begin(), options.parameter_overrides().end(),
    [](const auto & parameter) {
      return parameter.get_name() == "scan_topic";
    });
  const bool scan_topics_overridden = std::any_of(
    options.parameter_overrides().begin(), options.parameter_overrides().end(),
    [](const auto & parameter) {
      return parameter.get_name() == "scan_topics";
    });
  if (!scan_topics_overridden && scan_topic_overridden) {
    initial_config.scan_topics = initial_config.scan_topic.empty() ?
      std::vector<std::string>{} : std::vector<std::string>{initial_config.scan_topic};
  } else if (initial_config.scan_topics.empty() && !initial_config.scan_topic.empty()) {
    initial_config.scan_topics.push_back(initial_config.scan_topic);
  }
  initial_config.scan_point_steps = this->declare_parameter<std::vector<int64_t>>(
    "scan_point_steps", std::vector<int64_t>(initial_config.scan_topics.size(), 1));
  initial_config.scan_max_points = this->declare_parameter<std::vector<int64_t>>(
    "scan_max_points", std::vector<int64_t>(initial_config.scan_topics.size(), 0));
  initial_config.scan_voxel_prefilters = this->declare_parameter<std::vector<double>>(
    "scan_voxel_prefilters", std::vector<double>(initial_config.scan_topics.size(), 0.0));
  initial_config.depth_cloud_topic = this->declare_parameter<std::string>(
    "depth_cloud_topic", "/camera/depth/points");
  initial_config.lidar_cloud_topic = this->declare_parameter<std::string>(
    "lidar_cloud_topic", "/livox/points");
  initial_config.depth_source_frame = this->declare_parameter<std::string>(
    "depth_source_frame", "camera_color_optical_frame");
  initial_config.depth_extrinsic_xyz = this->declare_parameter<std::vector<double>>(
    "depth_extrinsic_xyz", std::vector<double>{0.363, -0.040, -0.284});
  initial_config.depth_extrinsic_qxyzw = this->declare_parameter<std::vector<double>>(
    "depth_extrinsic_qxyzw", std::vector<double>{0.0, 0.216440, 0.0, 0.976296});
  initial_config.depth_use_extrinsic_fallback = this->declare_parameter<bool>(
    "depth_use_extrinsic_fallback", true);
  initial_config.points_topic = this->declare_parameter<std::string>(
    "points_topic", "/collision_voxel_layer/points");
  initial_config.fused_scan_topic = this->declare_parameter<std::string>(
    "fused_scan_topic", "/collision_voxel_layer/scan");
  initial_config.grid_topic = this->declare_parameter<std::string>(
    "grid_topic", "/collision_voxel_layer/grid");
  initial_config.markers_topic = this->declare_parameter<std::string>(
    "markers_topic", "/collision_voxel_layer/markers");
  initial_config.debug_cloud_topic = this->declare_parameter<std::string>(
    "debug_cloud_topic", "/collision_voxel_layer/debug_cloud");
  initial_config.source_status_topic = this->declare_parameter<std::string>(
    "source_status_topic", "/collision_voxel_layer/source_status");
  initial_config.visualization_control_topic = this->declare_parameter<std::string>(
    "visualization_control_topic", "/collision_voxel_layer/visualization_enabled");
  initial_config.visualization_enabled = this->declare_parameter<bool>(
    "visualization_enabled", false);
  initial_config.publish_points = this->declare_parameter<bool>("publish_points", true);
  initial_config.publish_fused_scan = this->declare_parameter<bool>("publish_fused_scan", true);
  initial_config.publish_voxel_grid = this->declare_parameter<bool>("publish_voxel_grid", false);
  initial_config.publish_markers = this->declare_parameter<bool>("publish_markers", false);
  initial_config.publish_debug_cloud = this->declare_parameter<bool>(
    "publish_debug_cloud", false);
  initial_config.publish_rate_hz = this->declare_parameter<double>("publish_rate", 10.0);
  initial_config.tf_timeout_s = this->declare_parameter<double>("tf_timeout_s", 0.05);
  const int sync_queue_size = this->declare_parameter<int>("sync_queue_size", 20);
  initial_config.sync_queue_size =
    sync_queue_size < 0 ? 0U : static_cast<std::size_t>(sync_queue_size);
  // Deprecated: kept so older deployed YAML files do not fail parameter loading.
  initial_config.sync_slop_s = this->declare_parameter<double>("sync_slop_s", 0.15);
  initial_config.source_timeout_s = this->declare_parameter<double>(
    "source_timeout_s", std::max(1.0, initial_config.sync_slop_s * 3.0));
  initial_config.source_health_check_period_s = this->declare_parameter<double>(
    "source_health_check_period_s", 1.0);
  initial_config.voxel_size_xy = this->declare_parameter<double>("voxel_size_xy", 0.10);
  initial_config.voxel_size_z = this->declare_parameter<double>("voxel_size_z", 0.10);
  initial_config.voxel_decay_time_s = this->declare_parameter<double>("voxel_decay_time_s", 1.0);
  initial_config.prune_threshold = this->declare_parameter<double>("prune_threshold", 0.05);
  initial_config.occupancy_max = this->declare_parameter<double>("occupancy_max", 1.0);
  initial_config.voxel_region_xy = this->declare_parameter<std::vector<double>>(
    "voxel_region_xy",
    std::vector<double>{-1.0, -2.5, 4.0, -2.5, 4.0, 2.5, -1.0, 2.5});
  initial_config.scan_min_range = this->declare_parameter<double>("scan_min_range", 0.05);
  initial_config.scan_max_range = this->declare_parameter<double>("scan_max_range", 8.0);
  initial_config.scan_weight = this->declare_parameter<double>("scan_weight", 0.6);
  initial_config.fused_scan_angle_min = this->declare_parameter<double>(
    "fused_scan_angle_min", -M_PI);
  initial_config.fused_scan_angle_max = this->declare_parameter<double>(
    "fused_scan_angle_max", M_PI);
  initial_config.fused_scan_angle_increment = this->declare_parameter<double>(
    "fused_scan_angle_increment", M_PI / 180.0);
  initial_config.fused_scan_range_min = this->declare_parameter<double>(
    "fused_scan_range_min", initial_config.scan_min_range);
  initial_config.fused_scan_range_max = this->declare_parameter<double>(
    "fused_scan_range_max", initial_config.scan_max_range);
  initial_config.depth_min_range = this->declare_parameter<double>("depth_min_range", 0.1);
  initial_config.depth_max_range = this->declare_parameter<double>("depth_max_range", 4.0);
  initial_config.depth_min_height = this->declare_parameter<double>("depth_min_height", -0.1);
  initial_config.depth_max_height = this->declare_parameter<double>("depth_max_height", 1.5);
  initial_config.depth_min_base_height = this->declare_parameter<double>(
    "depth_min_base_height", 0.08);
  initial_config.depth_max_base_height = this->declare_parameter<double>(
    "depth_max_base_height", 1.8);
  initial_config.depth_weight = this->declare_parameter<double>("depth_weight", 0.8);
  initial_config.depth_voxel_prefilter = this->declare_parameter<double>(
    "depth_voxel_prefilter", initial_config.voxel_size_xy);
  initial_config.depth_process_rate_hz = this->declare_parameter<double>(
    "depth_process_rate_hz", 5.0);
  const int depth_point_step = this->declare_parameter<int>("depth_point_step", 4);
  initial_config.depth_point_step = depth_point_step < 1 ? 1U : static_cast<std::size_t>(depth_point_step);
  const int depth_max_points = this->declare_parameter<int>("depth_max_points", 1200);
  initial_config.depth_max_points = depth_max_points < 0 ? 0U : static_cast<std::size_t>(depth_max_points);
  initial_config.lidar_min_range = this->declare_parameter<double>("lidar_min_range", 0.1);
  initial_config.lidar_max_range = this->declare_parameter<double>("lidar_max_range", 8.0);
  initial_config.lidar_min_height = this->declare_parameter<double>("lidar_min_height", 0.05);
  initial_config.lidar_max_height = this->declare_parameter<double>("lidar_max_height", 1.8);
  initial_config.lidar_weight = this->declare_parameter<double>("lidar_weight", 0.8);
  initial_config.lidar_voxel_prefilter = this->declare_parameter<double>(
    "lidar_voxel_prefilter", initial_config.voxel_size_xy);
  initial_config.lidar_process_rate_hz = this->declare_parameter<double>(
    "lidar_process_rate_hz", 5.0);
  const int lidar_point_step = this->declare_parameter<int>("lidar_point_step", 8);
  initial_config.lidar_point_step = lidar_point_step < 1 ? 1U : static_cast<std::size_t>(lidar_point_step);
  const int lidar_max_points = this->declare_parameter<int>("lidar_max_points", 1800);
  initial_config.lidar_max_points = lidar_max_points < 0 ? 0U : static_cast<std::size_t>(lidar_max_points);

  std::string error;
  if (!validate_runtime_config(initial_config, error)) {
    throw std::runtime_error("invalid collision_voxel_layer parameters: " + error);
  }

  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_, this, true);
  apply_runtime_config(initial_config);
  configure_config_file_tracking(true);
  parameter_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&CollisionVoxelLayerNode::on_parameter_change, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "collision_voxel_layer started: scan_topics=%s depth=%s lidar=%s base_frame=%s points=%s grid=%s markers=%s debug_cloud=%s source_status=%s",
    join_strings(scan_topics_, ",").c_str(), depth_cloud_topic_.c_str(),
    lidar_cloud_topic_.c_str(), base_frame_.c_str(), points_topic_.c_str(),
    grid_topic_.c_str(), markers_topic_.c_str(), debug_cloud_topic_.c_str(),
    source_status_topic_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "collision_voxel_layer uses independent inputs; one missing source will warn but will not block voxel output from available sources");
}

bool CollisionVoxelLayerNode::reload_config_if_needed(bool force)
{
  if (resolved_config_file_.empty()) {
    return false;
  }

  if (!force) {
    if (!config_reload_enabled_) {
      return false;
    }
    if (!poll_config_file_changed()) {
      return false;
    }
  }

  try {
    const auto parameters = load_parameters_from_file(resolved_config_file_);
    if (parameters.empty()) {
      configure_config_file_tracking(true);
      RCLCPP_WARN(
        get_logger(),
        "Config file '%s' did not contain any parameters for collision_voxel_layer",
        resolved_config_file_.c_str());
      return false;
    }

    const auto result = this->set_parameters_atomically(parameters);
    if (!result.successful) {
      configure_config_file_tracking(true);
      RCLCPP_ERROR(
        get_logger(),
        "Failed to apply config reload from '%s': %s",
        resolved_config_file_.c_str(),
        result.reason.c_str());
      return false;
    }

    configure_config_file_tracking(true);
    RCLCPP_INFO(get_logger(), "Reloaded collision_voxel_layer config from %s", resolved_config_file_.c_str());
    return true;
  } catch (const std::exception & ex) {
    configure_config_file_tracking(true);
    RCLCPP_ERROR(
      get_logger(),
      "Failed to load collision_voxel_layer config '%s': %s",
      resolved_config_file_.c_str(),
      ex.what());
    return false;
  }
}

bool CollisionVoxelLayerNode::validate_runtime_config(
  const RuntimeConfig & config,
  std::string & error) const
{
  if (config.publish_rate_hz < 0.5) {
    error = "publish_rate must be >= 0.5";
    return false;
  }
  if (config.tf_timeout_s < 0.0) {
    error = "tf_timeout_s must be >= 0.0";
    return false;
  }
  if (config.sync_queue_size < 2U) {
    error = "sync_queue_size must be >= 2";
    return false;
  }
  if (config.sync_slop_s < 0.0) {
    error = "sync_slop_s must be >= 0.0";
    return false;
  }
  if (!(config.source_timeout_s > 0.0)) {
    error = "source_timeout_s must be > 0.0";
    return false;
  }
  if (!(config.source_health_check_period_s > 0.0)) {
    error = "source_health_check_period_s must be > 0.0";
    return false;
  }
  if (!(config.voxel_size_xy > 0.0)) {
    error = "voxel_size_xy must be > 0.0";
    return false;
  }
  if (!(config.voxel_size_z > 0.0)) {
    error = "voxel_size_z must be > 0.0";
    return false;
  }
  if (!(config.voxel_decay_time_s > 0.0)) {
    error = "voxel_decay_time_s must be > 0.0";
    return false;
  }
  if (config.prune_threshold < 0.0) {
    error = "prune_threshold must be >= 0.0";
    return false;
  }
  if (!(config.occupancy_max > 0.0)) {
    error = "occupancy_max must be > 0.0";
    return false;
  }
  if (config.voxel_region_xy.size() < 6U || config.voxel_region_xy.size() % 2U != 0U) {
    error = "voxel_region_xy must contain at least 3 xy pairs";
    return false;
  }
  if (config.scan_min_range < 0.0 || config.scan_max_range < config.scan_min_range) {
    error = "scan range bounds are invalid";
    return false;
  }
  for (const auto & topic : config.scan_topics) {
    if (topic.empty()) {
      error = "scan_topics must not contain empty entries";
      return false;
    }
  }
  for (const auto value : config.scan_point_steps) {
    if (value < 1) {
      error = "scan_point_steps values must be >= 1";
      return false;
    }
  }
  for (const auto value : config.scan_max_points) {
    if (value < 0) {
      error = "scan_max_points values must be >= 0";
      return false;
    }
  }
  for (const auto value : config.scan_voxel_prefilters) {
    if (value < 0.0) {
      error = "scan_voxel_prefilters values must be >= 0.0";
      return false;
    }
  }
  if (config.scan_weight < 0.0) {
    error = "scan_weight must be >= 0.0";
    return false;
  }
  if (config.depth_min_range < 0.0 || config.depth_max_range < config.depth_min_range) {
    error = "depth range bounds are invalid";
    return false;
  }
  if (config.depth_max_height < config.depth_min_height) {
    error = "depth height bounds are invalid";
    return false;
  }
  if (config.depth_max_base_height < config.depth_min_base_height) {
    error = "depth base height bounds are invalid";
    return false;
  }
  if (config.depth_use_extrinsic_fallback && config.depth_source_frame.empty()) {
    error = "depth_source_frame must not be empty when depth_use_extrinsic_fallback is enabled";
    return false;
  }
  if (config.depth_extrinsic_xyz.size() != 3U) {
    error = "depth_extrinsic_xyz must contain exactly 3 values";
    return false;
  }
  if (config.depth_extrinsic_qxyzw.size() != 4U) {
    error = "depth_extrinsic_qxyzw must contain exactly 4 values";
    return false;
  }
  const double depth_q_norm2 =
    config.depth_extrinsic_qxyzw[0] * config.depth_extrinsic_qxyzw[0] +
    config.depth_extrinsic_qxyzw[1] * config.depth_extrinsic_qxyzw[1] +
    config.depth_extrinsic_qxyzw[2] * config.depth_extrinsic_qxyzw[2] +
    config.depth_extrinsic_qxyzw[3] * config.depth_extrinsic_qxyzw[3];
  if (depth_q_norm2 <= 1e-12) {
    error = "depth_extrinsic_qxyzw must not be a zero quaternion";
    return false;
  }
  if (config.depth_weight < 0.0) {
    error = "depth_weight must be >= 0.0";
    return false;
  }
  if (config.depth_voxel_prefilter < 0.0) {
    error = "depth_voxel_prefilter must be >= 0.0";
    return false;
  }
  if (config.depth_process_rate_hz <= 0.0) {
    error = "depth_process_rate_hz must be > 0.0";
    return false;
  }
  if (config.depth_point_step == 0U) {
    error = "depth_point_step must be >= 1";
    return false;
  }
  if (config.lidar_min_range < 0.0 || config.lidar_max_range < config.lidar_min_range) {
    error = "lidar range bounds are invalid";
    return false;
  }
  if (config.lidar_max_height < config.lidar_min_height) {
    error = "lidar height bounds are invalid";
    return false;
  }
  if (config.lidar_weight < 0.0) {
    error = "lidar_weight must be >= 0.0";
    return false;
  }
  if (config.lidar_voxel_prefilter < 0.0) {
    error = "lidar_voxel_prefilter must be >= 0.0";
    return false;
  }
  if (config.lidar_process_rate_hz <= 0.0) {
    error = "lidar_process_rate_hz must be > 0.0";
    return false;
  }
  if (config.lidar_point_step == 0U) {
    error = "lidar_point_step must be >= 1";
    return false;
  }
  if (config.publish_points && config.points_topic.empty()) {
    error = "points_topic must not be empty when publish_points is enabled";
    return false;
  }
  if (config.publish_fused_scan && config.fused_scan_topic.empty()) {
    error = "fused_scan_topic must not be empty when publish_fused_scan is enabled";
    return false;
  }
  if (config.fused_scan_angle_increment <= 0.0) {
    error = "fused_scan_angle_increment must be > 0.0";
    return false;
  }
  if (config.fused_scan_angle_max <= config.fused_scan_angle_min) {
    error = "fused_scan angle bounds are invalid";
    return false;
  }
  if (
    config.fused_scan_range_min < 0.0 ||
    config.fused_scan_range_max <= config.fused_scan_range_min)
  {
    error = "fused_scan range bounds are invalid";
    return false;
  }
  if (config.publish_voxel_grid && config.grid_topic.empty()) {
    error = "grid_topic must not be empty when publish_voxel_grid is enabled";
    return false;
  }
  if (config.config_reload_period_s < 0.1) {
    error = "config_reload_period_s must be >= 0.1";
    return false;
  }
  if (config.source_status_topic.empty()) {
    error = "source_status_topic must not be empty";
    return false;
  }
  if (config.visualization_control_topic.empty()) {
    error = "visualization_control_topic must not be empty";
    return false;
  }
  return true;
}

void CollisionVoxelLayerNode::apply_runtime_config(const RuntimeConfig & config)
{
  current_config_ = config;
  config_file_ = config.config_file;
  resolved_config_file_ = resolve_config_path(config_file_);
  config_reload_enabled_ = config.config_reload_enabled;
  config_reload_period_s_ = config.config_reload_period_s;
  base_frame_ = config.base_frame;
  scan_topic_ = config.scan_topic;
  scan_topics_ = config.scan_topics.empty() && !config.scan_topic.empty() ?
    std::vector<std::string>{config.scan_topic} : config.scan_topics;
  scan_point_steps_ = expand_int_vector(config.scan_point_steps, scan_topics_.size(), 1);
  scan_max_points_ = expand_int_vector(config.scan_max_points, scan_topics_.size(), 0);
  scan_voxel_prefilters_ =
    expand_double_vector(config.scan_voxel_prefilters, scan_topics_.size(), 0.0);
  depth_cloud_topic_ = config.depth_cloud_topic;
  lidar_cloud_topic_ = config.lidar_cloud_topic;
  depth_source_frame_ = config.depth_source_frame;
  depth_extrinsic_xyz_ = config.depth_extrinsic_xyz;
  depth_extrinsic_qxyzw_ = config.depth_extrinsic_qxyzw;
  depth_use_extrinsic_fallback_ = config.depth_use_extrinsic_fallback;
  points_topic_ = config.points_topic;
  fused_scan_topic_ = config.fused_scan_topic;
  grid_topic_ = config.grid_topic;
  markers_topic_ = config.markers_topic;
  debug_cloud_topic_ = config.debug_cloud_topic;
  source_status_topic_ = config.source_status_topic;
  visualization_control_topic_ = config.visualization_control_topic;
  publish_points_ = config.publish_points;
  publish_fused_scan_ = config.publish_fused_scan;
  publish_voxel_grid_ = config.publish_voxel_grid;
  visualization_enabled_ = config.visualization_enabled;
  publish_markers_ = config.publish_markers;
  publish_debug_cloud_ = config.publish_debug_cloud;
  publish_rate_hz_ = config.publish_rate_hz;
  tf_timeout_s_ = config.tf_timeout_s;
  sync_queue_size_ = config.sync_queue_size;
  source_timeout_s_ = config.source_timeout_s;
  source_health_check_period_s_ = config.source_health_check_period_s;
  scan_weight_ = config.scan_weight;
  fused_scan_angle_min_ = config.fused_scan_angle_min;
  fused_scan_angle_max_ = config.fused_scan_angle_max;
  fused_scan_angle_increment_ = config.fused_scan_angle_increment;
  fused_scan_range_min_ = config.fused_scan_range_min;
  fused_scan_range_max_ = config.fused_scan_range_max;
  depth_weight_ = config.depth_weight;
  depth_voxel_prefilter_ = config.depth_voxel_prefilter;
  lidar_weight_ = config.lidar_weight;
  lidar_voxel_prefilter_ = config.lidar_voxel_prefilter;
  depth_process_period_s_ = 1.0 / config.depth_process_rate_hz;
  lidar_process_period_s_ = 1.0 / config.lidar_process_rate_hz;
  depth_min_base_height_ = config.depth_min_base_height;
  depth_max_base_height_ = config.depth_max_base_height;
  voxel_region_xy_ = config.voxel_region_xy;

  scan_params_.min_range = config.scan_min_range;
  scan_params_.max_range = config.scan_max_range;

  depth_params_.min_range = config.depth_min_range;
  depth_params_.max_range = config.depth_max_range;
  depth_params_.min_height = config.depth_min_height;
  depth_params_.max_height = config.depth_max_height;
  depth_params_.point_step = config.depth_point_step;
  depth_params_.max_points = config.depth_max_points;

  lidar_params_.min_range = config.lidar_min_range;
  lidar_params_.max_range = config.lidar_max_range;
  lidar_params_.min_height = config.lidar_min_height;
  lidar_params_.max_height = config.lidar_max_height;
  lidar_params_.point_step = config.lidar_point_step;
  lidar_params_.max_points = config.lidar_max_points;

  depth_extrinsic_params_.enabled = config.depth_use_extrinsic_fallback;
  depth_extrinsic_params_.translation = tf2::Vector3(
    config.depth_extrinsic_xyz[0],
    config.depth_extrinsic_xyz[1],
    config.depth_extrinsic_xyz[2]);
  depth_extrinsic_params_.rotation = tf2::Quaternion(
    config.depth_extrinsic_qxyzw[0],
    config.depth_extrinsic_qxyzw[1],
    config.depth_extrinsic_qxyzw[2],
    config.depth_extrinsic_qxyzw[3]);

  sparse_grid_ = std::make_unique<SparseVoxelGrid>(
    config.voxel_size_xy,
    config.voxel_size_z,
    config.voxel_decay_time_s,
    config.prune_threshold,
    static_cast<float>(config.occupancy_max));

  rebuild_publishers();
  rebuild_input_pipeline();
  rebuild_decay_timer();
  rebuild_config_reload_timer();
}

void CollisionVoxelLayerNode::rebuild_publishers()
{
  grid_pub_.reset();
  points_pub_.reset();
  fused_scan_pub_.reset();
  markers_pub_.reset();
  debug_cloud_pub_.reset();
  source_status_pub_.reset();

  if (publish_points_) {
    points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      points_topic_, rclcpp::QoS(1).transient_local().reliable());
  }
  if (publish_fused_scan_) {
    fused_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      fused_scan_topic_, rclcpp::QoS(1).transient_local().reliable());
  }
  if (voxel_output_enabled()) {
    grid_pub_ = this->create_publisher<msg::VoxelGrid>(
      grid_topic_, rclcpp::QoS(1).transient_local().reliable());
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      markers_topic_, rclcpp::QoS(1).transient_local().reliable());
    debug_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      debug_cloud_topic_, rclcpp::QoS(1).transient_local().reliable());
  }
  source_status_pub_ = this->create_publisher<std_msgs::msg::String>(
    source_status_topic_, rclcpp::QoS(1).transient_local().reliable());
}

void CollisionVoxelLayerNode::rebuild_input_pipeline()
{
  scan_sources_.clear();
  depth_sub_.reset();
  lidar_sub_.reset();
  visualization_control_sub_.reset();
  depth_seen_ = false;
  last_depth_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_depth_receive_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_depth_process_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  depth_points_.clear();
  lidar_seen_ = false;
  last_lidar_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_lidar_receive_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_lidar_process_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  lidar_points_.clear();
  last_graph_health_check_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  if (scan_topics_.empty() && depth_cloud_topic_.empty() && lidar_cloud_topic_.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "collision_voxel_layer inputs are disabled because scan_topics, depth_cloud_topic, and lidar_cloud_topic are empty");
    return;
  }

  if (!scan_topics_.empty()) {
    scan_sources_.resize(scan_topics_.size());
    for (std::size_t i = 0; i < scan_topics_.size(); ++i) {
      scan_sources_[i].topic = scan_topics_[i];
      scan_sources_[i].params = scan_params_;
      scan_sources_[i].params.point_step =
        static_cast<std::size_t>(std::max<int64_t>(1, scan_point_steps_[i]));
      scan_sources_[i].params.max_points =
        static_cast<std::size_t>(std::max<int64_t>(0, scan_max_points_[i]));
      scan_sources_[i].voxel_prefilter = scan_voxel_prefilters_[i];
      scan_sources_[i].subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_sources_[i].topic,
        rclcpp::SensorDataQoS(),
        [this, i](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          on_scan(i, msg);
        });
    }
  } else {
    RCLCPP_WARN(get_logger(), "collision_voxel_layer scan input disabled: scan_topics is empty");
  }

  if (!depth_cloud_topic_.empty()) {
    depth_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      depth_cloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&CollisionVoxelLayerNode::on_depth_cloud, this, std::placeholders::_1));
  } else {
    RCLCPP_WARN(
      get_logger(), "collision_voxel_layer depth input disabled: depth_cloud_topic is empty");
  }

  if (!lidar_cloud_topic_.empty()) {
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_cloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&CollisionVoxelLayerNode::on_lidar_cloud, this, std::placeholders::_1));
  } else {
    RCLCPP_WARN(
      get_logger(), "collision_voxel_layer lidar input disabled: lidar_cloud_topic is empty");
  }

  visualization_control_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    visualization_control_topic_,
    rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&CollisionVoxelLayerNode::on_visualization_control, this, std::placeholders::_1));

  log_graph_health(true);
}

void CollisionVoxelLayerNode::rebuild_decay_timer()
{
  decay_timer_.reset();
  decay_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_hz_),
    std::bind(&CollisionVoxelLayerNode::on_decay_timer, this));
}

void CollisionVoxelLayerNode::rebuild_config_reload_timer()
{
  config_reload_timer_.reset();
  if (!config_reload_enabled_ || resolved_config_file_.empty()) {
    return;
  }

  config_reload_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(config_reload_period_s_),
    std::bind(&CollisionVoxelLayerNode::on_config_reload_timer, this));
}

void CollisionVoxelLayerNode::configure_config_file_tracking(bool sync_state)
{
  if (!sync_state) {
    config_file_state_initialized_ = false;
    last_config_file_state_ = FileState{};
    return;
  }

  last_config_file_state_ = capture_config_file_state();
  config_file_state_initialized_ = true;
}

CollisionVoxelLayerNode::FileState CollisionVoxelLayerNode::capture_config_file_state() const
{
  FileState state;
  if (resolved_config_file_.empty()) {
    return state;
  }

  std::error_code ec;
  state.exists = std::filesystem::exists(resolved_config_file_, ec);
  if (ec || !state.exists) {
    state.exists = false;
    return state;
  }

  state.mtime = std::filesystem::last_write_time(resolved_config_file_, ec);
  if (ec) {
    state.exists = false;
    state.mtime = std::filesystem::file_time_type{};
  }
  return state;
}

bool CollisionVoxelLayerNode::poll_config_file_changed()
{
  const auto current_state = capture_config_file_state();
  if (!config_file_state_initialized_) {
    last_config_file_state_ = current_state;
    config_file_state_initialized_ = true;
    return true;
  }

  const bool changed =
    current_state.exists != last_config_file_state_.exists ||
    (current_state.exists && current_state.mtime != last_config_file_state_.mtime);
  if (changed) {
    last_config_file_state_ = current_state;
  }
  return changed;
}

std::vector<rclcpp::Parameter> CollisionVoxelLayerNode::load_parameters_from_file(
  const std::string & path) const
{
  const auto parameter_map = rclcpp::parameter_map_from_yaml_file(path);
  const std::string normalized_name = normalize_graph_name(this->get_name());
  const std::string normalized_fqn = normalize_graph_name(this->get_fully_qualified_name());

  for (const auto & entry : parameter_map) {
    const std::string normalized_entry = normalize_graph_name(entry.first);
    if (normalized_entry == normalized_name || normalized_entry == normalized_fqn) {
      return entry.second;
    }
  }

  throw std::runtime_error(
          "parameter file does not contain collision_voxel_layer node entry: " + path);
}

rcl_interfaces::msg::SetParametersResult CollisionVoxelLayerNode::on_parameter_change(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto next_config = current_config_;
  bool has_runtime_change = false;
  auto reject = [](const std::string & reason) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = reason;
      return result;
    };

  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (name == "config_file") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        return reject("config_file must be a string");
      }
      next_config.config_file = parameter.as_string();
      has_runtime_change = true;
      continue;
    }
    if (name == "config_reload_enabled") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        return reject("config_reload_enabled must be a bool");
      }
      next_config.config_reload_enabled = parameter.as_bool();
      has_runtime_change = true;
      continue;
    }
    if (name == "config_reload_period_s") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("config_reload_period_s must be a double");
      }
      next_config.config_reload_period_s = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "base_frame") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        return reject("base_frame must be a string");
      }
      next_config.base_frame = parameter.as_string();
      has_runtime_change = true;
      continue;
    }
    if (name == "scan_topic") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        return reject("scan_topic must be a string");
      }
      next_config.scan_topic = parameter.as_string();
      next_config.scan_topics = parameter.as_string().empty() ?
        std::vector<std::string>{} : std::vector<std::string>{parameter.as_string()};
      has_runtime_change = true;
      continue;
    }
    if (name == "scan_topics") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
        return reject("scan_topics must be a string array");
      }
      next_config.scan_topics = parameter.as_string_array();
      next_config.scan_topic = next_config.scan_topics.empty() ? "" : next_config.scan_topics.front();
      has_runtime_change = true;
      continue;
    }
    if (name == "scan_point_steps") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
        return reject("scan_point_steps must be an integer array");
      }
      next_config.scan_point_steps = parameter.as_integer_array();
      has_runtime_change = true;
      continue;
    }
    if (name == "scan_max_points") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
        return reject("scan_max_points must be an integer array");
      }
      next_config.scan_max_points = parameter.as_integer_array();
      has_runtime_change = true;
      continue;
    }
    if (name == "scan_voxel_prefilters") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        return reject("scan_voxel_prefilters must be a double array");
      }
      next_config.scan_voxel_prefilters = parameter.as_double_array();
      has_runtime_change = true;
      continue;
    }
    if (name == "depth_cloud_topic") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        return reject("depth_cloud_topic must be a string");
      }
      next_config.depth_cloud_topic = parameter.as_string();
      has_runtime_change = true;
      continue;
    }
    if (name == "lidar_cloud_topic") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        return reject("lidar_cloud_topic must be a string");
      }
      next_config.lidar_cloud_topic = parameter.as_string();
      has_runtime_change = true;
      continue;
    }
    if (name == "depth_source_frame") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        return reject("depth_source_frame must be a string");
      }
      next_config.depth_source_frame = parameter.as_string();
      has_runtime_change = true;
      continue;
    }
    if (name == "depth_extrinsic_xyz") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        return reject("depth_extrinsic_xyz must be a double array");
      }
      next_config.depth_extrinsic_xyz = parameter.as_double_array();
      has_runtime_change = true;
      continue;
    }
    if (name == "depth_extrinsic_qxyzw") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        return reject("depth_extrinsic_qxyzw must be a double array");
      }
      next_config.depth_extrinsic_qxyzw = parameter.as_double_array();
      has_runtime_change = true;
      continue;
    }
    if (name == "depth_use_extrinsic_fallback") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        return reject("depth_use_extrinsic_fallback must be a bool");
      }
      next_config.depth_use_extrinsic_fallback = parameter.as_bool();
      has_runtime_change = true;
      continue;
    }
    if (name == "points_topic") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        return reject("points_topic must be a string");
      }
      next_config.points_topic = parameter.as_string();
      has_runtime_change = true;
      continue;
    }
    if (name == "fused_scan_topic") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        return reject("fused_scan_topic must be a string");
      }
      next_config.fused_scan_topic = parameter.as_string();
      has_runtime_change = true;
      continue;
    }
    if (name == "grid_topic") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        return reject("grid_topic must be a string");
      }
      next_config.grid_topic = parameter.as_string();
      has_runtime_change = true;
      continue;
    }
    if (name == "markers_topic") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        return reject("markers_topic must be a string");
      }
      next_config.markers_topic = parameter.as_string();
      has_runtime_change = true;
      continue;
    }
    if (name == "debug_cloud_topic") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        return reject("debug_cloud_topic must be a string");
      }
      next_config.debug_cloud_topic = parameter.as_string();
      has_runtime_change = true;
      continue;
    }
    if (name == "source_status_topic") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        return reject("source_status_topic must be a string");
      }
      next_config.source_status_topic = parameter.as_string();
      has_runtime_change = true;
      continue;
    }
    if (name == "visualization_control_topic") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        return reject("visualization_control_topic must be a string");
      }
      next_config.visualization_control_topic = parameter.as_string();
      has_runtime_change = true;
      continue;
    }
    if (name == "visualization_enabled") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        return reject("visualization_enabled must be a bool");
      }
      next_config.visualization_enabled = parameter.as_bool();
      has_runtime_change = true;
      continue;
    }
    if (name == "publish_points") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        return reject("publish_points must be a bool");
      }
      next_config.publish_points = parameter.as_bool();
      has_runtime_change = true;
      continue;
    }
    if (name == "publish_fused_scan") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        return reject("publish_fused_scan must be a bool");
      }
      next_config.publish_fused_scan = parameter.as_bool();
      has_runtime_change = true;
      continue;
    }
    if (name == "publish_voxel_grid") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        return reject("publish_voxel_grid must be a bool");
      }
      next_config.publish_voxel_grid = parameter.as_bool();
      has_runtime_change = true;
      continue;
    }
    if (name == "publish_markers") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        return reject("publish_markers must be a bool");
      }
      next_config.publish_markers = parameter.as_bool();
      has_runtime_change = true;
      continue;
    }
    if (name == "publish_debug_cloud") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        return reject("publish_debug_cloud must be a bool");
      }
      next_config.publish_debug_cloud = parameter.as_bool();
      has_runtime_change = true;
      continue;
    }
    if (name == "publish_rate") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("publish_rate must be a double");
      }
      next_config.publish_rate_hz = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "tf_timeout_s") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("tf_timeout_s must be a double");
      }
      next_config.tf_timeout_s = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "sync_queue_size") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        return reject("sync_queue_size must be an integer");
      }
      const auto value = parameter.as_int();
      next_config.sync_queue_size = value < 0 ? 0U : static_cast<std::size_t>(value);
      has_runtime_change = true;
      continue;
    }
    if (name == "sync_slop_s") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("sync_slop_s must be a double");
      }
      next_config.sync_slop_s = parameter.as_double();
      next_config.source_timeout_s = std::max(1.0, next_config.sync_slop_s * 3.0);
      has_runtime_change = true;
      continue;
    }
    if (name == "source_timeout_s") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("source_timeout_s must be a double");
      }
      next_config.source_timeout_s = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "source_health_check_period_s") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("source_health_check_period_s must be a double");
      }
      next_config.source_health_check_period_s = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "voxel_size_xy") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("voxel_size_xy must be a double");
      }
      next_config.voxel_size_xy = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "voxel_size_z") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("voxel_size_z must be a double");
      }
      next_config.voxel_size_z = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "voxel_decay_time_s") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("voxel_decay_time_s must be a double");
      }
      next_config.voxel_decay_time_s = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "prune_threshold") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("prune_threshold must be a double");
      }
      next_config.prune_threshold = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "occupancy_max") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("occupancy_max must be a double");
      }
      next_config.occupancy_max = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "voxel_region_xy") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        return reject("voxel_region_xy must be a double array");
      }
      next_config.voxel_region_xy = parameter.as_double_array();
      has_runtime_change = true;
      continue;
    }
    if (name == "scan_min_range") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("scan_min_range must be a double");
      }
      next_config.scan_min_range = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "scan_max_range") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("scan_max_range must be a double");
      }
      next_config.scan_max_range = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "scan_weight") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("scan_weight must be a double");
      }
      next_config.scan_weight = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "fused_scan_angle_min") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("fused_scan_angle_min must be a double");
      }
      next_config.fused_scan_angle_min = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "fused_scan_angle_max") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("fused_scan_angle_max must be a double");
      }
      next_config.fused_scan_angle_max = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "fused_scan_angle_increment") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("fused_scan_angle_increment must be a double");
      }
      next_config.fused_scan_angle_increment = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "fused_scan_range_min") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("fused_scan_range_min must be a double");
      }
      next_config.fused_scan_range_min = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "fused_scan_range_max") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("fused_scan_range_max must be a double");
      }
      next_config.fused_scan_range_max = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "depth_min_range") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("depth_min_range must be a double");
      }
      next_config.depth_min_range = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "depth_max_range") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("depth_max_range must be a double");
      }
      next_config.depth_max_range = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "depth_min_height") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("depth_min_height must be a double");
      }
      next_config.depth_min_height = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "depth_max_height") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("depth_max_height must be a double");
      }
      next_config.depth_max_height = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "depth_min_base_height") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("depth_min_base_height must be a double");
      }
      next_config.depth_min_base_height = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "depth_max_base_height") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("depth_max_base_height must be a double");
      }
      next_config.depth_max_base_height = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "depth_weight") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("depth_weight must be a double");
      }
      next_config.depth_weight = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "depth_voxel_prefilter") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("depth_voxel_prefilter must be a double");
      }
      next_config.depth_voxel_prefilter = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "lidar_min_range") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("lidar_min_range must be a double");
      }
      next_config.lidar_min_range = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "lidar_max_range") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("lidar_max_range must be a double");
      }
      next_config.lidar_max_range = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "lidar_min_height") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("lidar_min_height must be a double");
      }
      next_config.lidar_min_height = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "lidar_max_height") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("lidar_max_height must be a double");
      }
      next_config.lidar_max_height = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "lidar_weight") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("lidar_weight must be a double");
      }
      next_config.lidar_weight = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "lidar_voxel_prefilter") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("lidar_voxel_prefilter must be a double");
      }
      next_config.lidar_voxel_prefilter = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "ok";

  if (!has_runtime_change) {
    return result;
  }

  std::string error;
  if (!validate_runtime_config(next_config, error)) {
    result.successful = false;
    result.reason = error;
    return result;
  }

  const bool config_file_changed = next_config.config_file != current_config_.config_file;
  apply_runtime_config(next_config);
  configure_config_file_tracking(!config_file_changed);
  return result;
}

void CollisionVoxelLayerNode::on_scan(
  std::size_t source_index,
  const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (source_index >= scan_sources_.size()) {
    return;
  }

  const auto receive_time = this->get_clock()->now();
  auto & source = scan_sources_[source_index];
  source.seen = true;
  source.last_receive_time = receive_time;

  tf2::Transform transform;
  if (!lookup_transform(msg->header.frame_id, msg->header.stamp, transform)) {
    return;
  }

  const auto stamp = msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0 ?
    rclcpp::Time(msg->header.stamp) : this->get_clock()->now();

  const bool apply_transform = !msg->header.frame_id.empty() && msg->header.frame_id != base_frame_;
  source.points = convert_scan_to_points(*msg, transform, apply_transform, source.params);
  source.points = prefilter_points(source.points, source.voxel_prefilter);
  source.last_stamp = stamp;
  if (voxel_output_enabled()) {
    rebuild_scan_voxels(receive_time);
  }

  log_missing_sources("scan:" + source.topic, receive_time);
  if (publish_voxel_grid_) {
    publish_grid_only(stamp);
  }
}

void CollisionVoxelLayerNode::on_depth_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const auto receive_time = this->get_clock()->now();
  depth_seen_ = true;
  last_depth_receive_time_ = receive_time;
  if (
    last_depth_process_time_.nanoseconds() != 0 &&
    (receive_time - last_depth_process_time_).seconds() < depth_process_period_s_)
  {
    log_missing_sources("depth_skip", receive_time);
    return;
  }
  last_depth_process_time_ = receive_time;

  tf2::Transform transform;
  bool apply_transform = !msg->header.frame_id.empty() && msg->header.frame_id != base_frame_;
  if (apply_transform && !lookup_transform(msg->header.frame_id, msg->header.stamp, transform)) {
    if (
      depth_use_extrinsic_fallback_ &&
      msg->header.frame_id == depth_source_frame_)
    {
      transform = make_extrinsic_transform(depth_extrinsic_params_);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Using configured depth extrinsic fallback for %s -> %s because TF lookup failed",
        msg->header.frame_id.c_str(), base_frame_.c_str());
    } else {
      return;
    }
  }

  const auto stamp = msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0 ?
    rclcpp::Time(msg->header.stamp) : this->get_clock()->now();

  auto depth_points = filter_depth_cloud(*msg, transform, apply_transform, depth_params_);
  depth_points = prefilter_depth_points(depth_points);
  depth_points_.clear();
  depth_points_.reserve(depth_points.size());
  for (const auto & point : depth_points) {
    if (!point_in_voxel_region(point)) {
      continue;
    }
    if (point.z() < depth_min_base_height_ || point.z() > depth_max_base_height_) {
      continue;
    }
    depth_points_.push_back(point);
  }

  if (voxel_output_enabled()) {
    sparse_grid_->clear_source(kDepthSourceMask, stamp);
    for (const auto & point : depth_points_) {
      sparse_grid_->insert_point(
        point.x(), point.y(), point.z(),
        static_cast<float>(depth_weight_),
        kDepthSourceMask,
        stamp);
    }
  }

  last_depth_stamp_ = stamp;
  log_missing_sources("depth", receive_time);
  if (publish_voxel_grid_) {
    publish_grid_only(stamp);
  }
}

void CollisionVoxelLayerNode::on_lidar_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const auto receive_time = this->get_clock()->now();
  lidar_seen_ = true;
  last_lidar_receive_time_ = receive_time;
  if (
    last_lidar_process_time_.nanoseconds() != 0 &&
    (receive_time - last_lidar_process_time_).seconds() < lidar_process_period_s_)
  {
    log_missing_sources("lidar_skip", receive_time);
    return;
  }
  last_lidar_process_time_ = receive_time;

  tf2::Transform transform;
  if (!lookup_transform(msg->header.frame_id, msg->header.stamp, transform)) {
    return;
  }

  const auto stamp = msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0 ?
    rclcpp::Time(msg->header.stamp) : this->get_clock()->now();

  const bool apply_transform = !msg->header.frame_id.empty() && msg->header.frame_id != base_frame_;
  auto lidar_points = filter_depth_cloud(*msg, transform, apply_transform, lidar_params_);
  lidar_points = prefilter_lidar_points(lidar_points);
  lidar_points_.clear();
  lidar_points_.reserve(lidar_points.size());
  for (const auto & point : lidar_points) {
    if (!point_in_voxel_region(point)) {
      continue;
    }
    lidar_points_.push_back(point);
  }

  if (voxel_output_enabled()) {
    sparse_grid_->clear_source(kLidarSourceMask, stamp);
    for (const auto & point : lidar_points_) {
      sparse_grid_->insert_point(
        point.x(), point.y(), point.z(),
        static_cast<float>(lidar_weight_),
        kLidarSourceMask,
        stamp);
    }
  }

  last_lidar_stamp_ = stamp;
  log_missing_sources("lidar", receive_time);
  if (publish_voxel_grid_) {
    publish_grid_only(stamp);
  }
}

void CollisionVoxelLayerNode::on_visualization_control(const std_msgs::msg::Bool::SharedPtr msg)
{
  visualization_enabled_ = msg->data;
  RCLCPP_INFO(
    get_logger(),
    "collision_voxel_layer visualization %s by topic %s",
    visualization_enabled_ ? "enabled" : "disabled",
    visualization_control_topic_.c_str());
}

void CollisionVoxelLayerNode::on_decay_timer()
{
  const auto now = this->get_clock()->now();
  if (voxel_output_enabled()) {
    sparse_grid_->decay_to(now);
    if (!any_scan_source_active(now)) {
      sparse_grid_->clear_source(kScanSourceMask, now);
    }
    if (
      depth_sub_ && depth_seen_ &&
      (now - last_depth_receive_time_).seconds() > source_timeout_s_)
    {
      sparse_grid_->clear_source(kDepthSourceMask, now);
    }
    if (
      lidar_sub_ && lidar_seen_ &&
      (now - last_lidar_receive_time_).seconds() > source_timeout_s_)
    {
      sparse_grid_->clear_source(kLidarSourceMask, now);
    }
  }
  log_missing_sources("timer", now);
  log_graph_health();
  publish_state(now);
}

bool CollisionVoxelLayerNode::any_scan_source_seen() const
{
  return std::any_of(
    scan_sources_.begin(), scan_sources_.end(),
    [](const auto & source) {
      return source.seen;
    });
}

bool CollisionVoxelLayerNode::any_scan_source_active(const rclcpp::Time & now) const
{
  return std::any_of(
    scan_sources_.begin(), scan_sources_.end(),
    [this, &now](const auto & source) {
      return source.seen && (now - source.last_receive_time).seconds() <= source_timeout_s_;
    });
}

bool CollisionVoxelLayerNode::point_in_voxel_region(const tf2::Vector3 & point) const
{
  bool inside = false;
  const double x = point.x();
  const double y = point.y();
  const std::size_t point_count = voxel_region_xy_.size() / 2U;
  for (std::size_t i = 0, j = point_count - 1U; i < point_count; j = i++) {
    const double xi = voxel_region_xy_[2U * i];
    const double yi = voxel_region_xy_[2U * i + 1U];
    const double xj = voxel_region_xy_[2U * j];
    const double yj = voxel_region_xy_[2U * j + 1U];
    const bool intersects = ((yi > y) != (yj > y)) &&
      (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
    if (intersects) {
      inside = !inside;
    }
  }
  return inside;
}

void CollisionVoxelLayerNode::rebuild_scan_voxels(const rclcpp::Time & now)
{
  sparse_grid_->clear_source(kScanSourceMask, now);
  for (const auto & source : scan_sources_) {
    if (!source.seen || (now - source.last_receive_time).seconds() > source_timeout_s_) {
      continue;
    }
    for (const auto & point : source.points) {
      if (!point_in_voxel_region(point)) {
        continue;
      }
      sparse_grid_->insert_point(
        point.x(), point.y(), point.z(),
        static_cast<float>(scan_weight_),
        kScanSourceMask,
        now);
    }
  }
}

void CollisionVoxelLayerNode::on_config_reload_timer()
{
  (void)reload_config_if_needed(false);
}

bool CollisionVoxelLayerNode::lookup_transform(
  const std::string & source_frame,
  const builtin_interfaces::msg::Time & stamp,
  tf2::Transform & transform)
{
  transform.setIdentity();

  if (source_frame.empty() || source_frame == base_frame_) {
    return true;
  }

  try {
    const auto transform_msg = tf_buffer_.lookupTransform(
      base_frame_, source_frame, rclcpp::Time(stamp),
      rclcpp::Duration::from_seconds(tf_timeout_s_));
    transform = make_transform(transform_msg);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Failed to transform %s -> %s: %s",
      source_frame.c_str(), base_frame_.c_str(), ex.what());
    return false;
  }
}

void CollisionVoxelLayerNode::publish_state(const rclcpp::Time & stamp)
{
  publish_fused_outputs(stamp);

  if (publish_voxel_grid_) {
    publish_grid_only(stamp);
  }

  if (!voxel_output_enabled() || !visualization_enabled_ ||
    (!publish_markers_ && !publish_debug_cloud_))
  {
    return;
  }

  std_msgs::msg::Header header;
  header.frame_id = base_frame_;
  header.stamp = static_cast<builtin_interfaces::msg::Time>(stamp);

  auto grid_msg = sparse_grid_->export_grid(header);
  if (publish_markers_ && markers_pub_->get_subscription_count() > 0) {
    markers_pub_->publish(build_markers(grid_msg));
  }
  if (publish_debug_cloud_ && debug_cloud_pub_->get_subscription_count() > 0) {
    debug_cloud_pub_->publish(build_debug_cloud(grid_msg));
  }
}

void CollisionVoxelLayerNode::publish_grid_only(const rclcpp::Time & stamp)
{
  if (!grid_pub_) {
    return;
  }
  std_msgs::msg::Header header;
  header.frame_id = base_frame_;
  header.stamp = static_cast<builtin_interfaces::msg::Time>(stamp);
  grid_pub_->publish(sparse_grid_->export_grid(header));
}

void CollisionVoxelLayerNode::publish_fused_outputs(const rclcpp::Time & stamp)
{
  if (!points_pub_ && !fused_scan_pub_) {
    return;
  }

  std_msgs::msg::Header header;
  header.frame_id = base_frame_;
  header.stamp = static_cast<builtin_interfaces::msg::Time>(stamp);
  const auto points = collect_sparse_points(this->get_clock()->now());
  if (points_pub_) {
    points_pub_->publish(build_sparse_cloud(header, points));
  }
  if (fused_scan_pub_) {
    fused_scan_pub_->publish(build_fused_scan(header, points));
  }
}

void CollisionVoxelLayerNode::publish_source_status(
  const std::string & active_source,
  const rclcpp::Time & now)
{
  std_msgs::msg::String status;
  const bool scan_stale = !scan_sources_.empty() && !any_scan_source_active(now);
  const bool depth_stale =
    depth_sub_ && (!depth_seen_ || (now - last_depth_receive_time_).seconds() > source_timeout_s_);
  const bool lidar_stale =
    lidar_sub_ && (!lidar_seen_ || (now - last_lidar_receive_time_).seconds() > source_timeout_s_);
  const bool any_source_active =
    any_scan_source_active(now) ||
    (depth_sub_ && depth_seen_ && !depth_stale) ||
    (lidar_sub_ && lidar_seen_ && !lidar_stale);

  std::ostringstream stream;
  stream << "{"
         << "\"active_source\":\"" << active_source << "\","
         << "\"any_source_active\":" << (any_source_active ? "true" : "false") << ","
         << "\"source_timeout_s\":" << source_timeout_s_ << ","
         << "\"output\":{"
         << "\"points\":" << (publish_points_ ? "true" : "false") << ","
         << "\"fused_scan\":" << (publish_fused_scan_ ? "true" : "false") << ","
         << "\"voxel_grid\":" << (publish_voxel_grid_ ? "true" : "false") << ","
         << "\"points_topic\":\"" << points_topic_ << "\","
         << "\"fused_scan_topic\":\"" << fused_scan_topic_ << "\","
         << "\"grid_topic\":\"" << grid_topic_ << "\""
         << "},"
         << "\"visualization\":{"
         << "\"enabled\":" << (visualization_enabled_ ? "true" : "false") << ","
         << "\"markers\":" << (publish_markers_ ? "true" : "false") << ","
         << "\"debug_cloud\":" << (publish_debug_cloud_ ? "true" : "false") << ","
         << "\"control_topic\":\"" << visualization_control_topic_ << "\""
         << "},"
         << "\"voxel_region_xy\":[";
  for (std::size_t i = 0; i < voxel_region_xy_.size(); ++i) {
    if (i > 0) {
      stream << ",";
    }
    stream << voxel_region_xy_[i];
  }
  stream << "],"
         << "\"scan\":{"
         << "\"enabled\":" << (!scan_sources_.empty() ? "true" : "false") << ","
         << "\"topics\":\"" << join_strings(scan_topics_, ",") << "\","
         << "\"seen\":" << (any_scan_source_seen() ? "true" : "false") << ","
         << "\"stale\":" << (scan_stale ? "true" : "false")
         << ",\"sources\":[";
  for (std::size_t i = 0; i < scan_sources_.size(); ++i) {
    const auto & source = scan_sources_[i];
    const bool source_stale =
      !source.seen || (now - source.last_receive_time).seconds() > source_timeout_s_;
    if (i > 0) {
      stream << ",";
    }
    stream << "{"
           << "\"topic\":\"" << source.topic << "\","
           << "\"seen\":" << (source.seen ? "true" : "false") << ","
           << "\"age_s\":" << (source.seen ? (now - source.last_receive_time).seconds() : -1.0)
           << ",\"stale\":" << (source_stale ? "true" : "false")
           << "}";
  }
  stream << "]"
         << "},"
         << "\"depth\":{"
         << "\"enabled\":" << (depth_sub_ ? "true" : "false") << ","
         << "\"topic\":\"" << depth_cloud_topic_ << "\","
         << "\"seen\":" << (depth_seen_ ? "true" : "false") << ","
         << "\"age_s\":" << (depth_seen_ ? (now - last_depth_receive_time_).seconds() : -1.0) << ","
         << "\"stale\":" << (depth_stale ? "true" : "false")
         << "},"
         << "\"lidar\":{"
         << "\"enabled\":" << (lidar_sub_ ? "true" : "false") << ","
         << "\"topic\":\"" << lidar_cloud_topic_ << "\","
         << "\"seen\":" << (lidar_seen_ ? "true" : "false") << ","
         << "\"age_s\":" << (lidar_seen_ ? (now - last_lidar_receive_time_).seconds() : -1.0) << ","
         << "\"stale\":" << (lidar_stale ? "true" : "false")
         << "}"
         << "}";
  status.data = stream.str();
  if (source_status_pub_) {
    source_status_pub_->publish(status);
  }
}

void CollisionVoxelLayerNode::log_missing_sources(
  const std::string & active_source,
  const rclcpp::Time & now)
{
  for (const auto & source : scan_sources_) {
    const bool scan_stale =
      !source.seen || (now - source.last_receive_time).seconds() > source_timeout_s_;
    if (scan_stale) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "collision_voxel_layer missing or stale scan input: topic=%s active_source=%s seen=%s timeout_s=%.2f",
        source.topic.c_str(), active_source.c_str(), source.seen ? "true" : "false",
        source_timeout_s_);
    }
  }

  if (depth_sub_) {
    const bool depth_stale =
      !depth_seen_ || (now - last_depth_receive_time_).seconds() > source_timeout_s_;
    if (depth_stale) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "collision_voxel_layer missing or stale depth input: topic=%s active_source=%s seen=%s timeout_s=%.2f",
        depth_cloud_topic_.c_str(), active_source.c_str(), depth_seen_ ? "true" : "false",
        source_timeout_s_);
    }
  }

  if (lidar_sub_) {
    const bool lidar_stale =
      !lidar_seen_ || (now - last_lidar_receive_time_).seconds() > source_timeout_s_;
    if (lidar_stale) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "collision_voxel_layer missing or stale lidar input: topic=%s active_source=%s seen=%s timeout_s=%.2f",
        lidar_cloud_topic_.c_str(), active_source.c_str(), lidar_seen_ ? "true" : "false",
        source_timeout_s_);
    }
  }

  publish_source_status(active_source, now);
}

void CollisionVoxelLayerNode::log_graph_health(bool force)
{
  const auto now = this->get_clock()->now();
  if (
    !force &&
    last_graph_health_check_time_.nanoseconds() != 0 &&
    (now - last_graph_health_check_time_).seconds() < source_health_check_period_s_)
  {
    return;
  }
  last_graph_health_check_time_ = now;

  for (const auto & source : scan_sources_) {
    const auto publishers = this->get_publishers_info_by_topic(source.topic);
    bool type_ok = true;
    for (const auto & publisher : publishers) {
      if (publisher.topic_type() != "sensor_msgs/msg/LaserScan") {
        type_ok = false;
        break;
      }
    }
    if (publishers.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "collision_voxel_layer scan topic has no publishers: topic=%s",
        source.topic.c_str());
    } else if (!type_ok) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "collision_voxel_layer scan topic type mismatch: topic=%s expected=sensor_msgs/msg/LaserScan",
        source.topic.c_str());
    }
  }

  if (depth_sub_) {
    const auto publishers = this->get_publishers_info_by_topic(depth_cloud_topic_);
    bool type_ok = true;
    for (const auto & publisher : publishers) {
      if (publisher.topic_type() != "sensor_msgs/msg/PointCloud2") {
        type_ok = false;
        break;
      }
    }
    if (publishers.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "collision_voxel_layer depth cloud topic has no publishers: topic=%s",
        depth_cloud_topic_.c_str());
    } else if (!type_ok) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "collision_voxel_layer depth cloud topic type mismatch: topic=%s expected=sensor_msgs/msg/PointCloud2",
        depth_cloud_topic_.c_str());
    }
  }

  if (lidar_sub_) {
    const auto publishers = this->get_publishers_info_by_topic(lidar_cloud_topic_);
    bool type_ok = true;
    for (const auto & publisher : publishers) {
      if (publisher.topic_type() != "sensor_msgs/msg/PointCloud2") {
        type_ok = false;
        break;
      }
    }
    if (publishers.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "collision_voxel_layer lidar cloud topic has no publishers: topic=%s",
        lidar_cloud_topic_.c_str());
    } else if (!type_ok) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "collision_voxel_layer lidar cloud topic type mismatch: topic=%s expected=sensor_msgs/msg/PointCloud2",
        lidar_cloud_topic_.c_str());
    }
  }
}

sensor_msgs::msg::PointCloud2 CollisionVoxelLayerNode::build_debug_cloud(
  const msg::VoxelGrid & grid) const
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = grid.header;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(grid.cells.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (const auto & cell : grid.cells) {
    *iter_x = cell.x;
    *iter_y = cell.y;
    *iter_z = cell.z;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return cloud;
}

sensor_msgs::msg::PointCloud2 CollisionVoxelLayerNode::build_sparse_cloud(
  const std_msgs::msg::Header & header,
  const std::vector<tf2::Vector3> & points) const
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  for (const auto & point : points) {
    *iter_x = static_cast<float>(point.x());
    *iter_y = static_cast<float>(point.y());
    *iter_z = static_cast<float>(point.z());
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return cloud;
}

sensor_msgs::msg::LaserScan CollisionVoxelLayerNode::build_fused_scan(
  const std_msgs::msg::Header & header,
  const std::vector<tf2::Vector3> & points) const
{
  sensor_msgs::msg::LaserScan scan;
  scan.header = header;
  scan.angle_min = static_cast<float>(fused_scan_angle_min_);
  scan.angle_max = static_cast<float>(fused_scan_angle_max_);
  scan.angle_increment = static_cast<float>(fused_scan_angle_increment_);
  scan.time_increment = 0.0F;
  scan.scan_time = static_cast<float>(1.0 / std::max(0.001, publish_rate_hz_));
  scan.range_min = static_cast<float>(fused_scan_range_min_);
  scan.range_max = static_cast<float>(fused_scan_range_max_);

  const auto bin_count = static_cast<std::size_t>(
    std::floor((fused_scan_angle_max_ - fused_scan_angle_min_) / fused_scan_angle_increment_)) + 1U;
  scan.ranges.assign(bin_count, std::numeric_limits<float>::infinity());

  for (const auto & point : points) {
    const double range = std::hypot(point.x(), point.y());
    if (range < fused_scan_range_min_ || range > fused_scan_range_max_) {
      continue;
    }
    const double angle = std::atan2(point.y(), point.x());
    if (angle < fused_scan_angle_min_ || angle > fused_scan_angle_max_) {
      continue;
    }
    const auto index = static_cast<std::size_t>(
      std::floor((angle - fused_scan_angle_min_) / fused_scan_angle_increment_));
    if (index >= scan.ranges.size()) {
      continue;
    }
    auto & range_bin = scan.ranges[index];
    range_bin = std::min(range_bin, static_cast<float>(range));
  }

  return scan;
}

visualization_msgs::msg::MarkerArray CollisionVoxelLayerNode::build_markers(
  const msg::VoxelGrid & grid) const
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.header = grid.header;
  clear_marker.ns = "collision_voxel_layer";
  clear_marker.id = 0;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  visualization_msgs::msg::Marker voxel_marker;
  voxel_marker.header = grid.header;
  voxel_marker.ns = "collision_voxel_layer";
  voxel_marker.id = 1;
  voxel_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  voxel_marker.action = visualization_msgs::msg::Marker::ADD;
  voxel_marker.pose.orientation.w = 1.0;
  voxel_marker.scale.x = grid.resolution_xy;
  voxel_marker.scale.y = grid.resolution_xy;
  voxel_marker.scale.z = grid.resolution_z;

  for (const auto & cell : grid.cells) {
    geometry_msgs::msg::Point point;
    point.x = cell.x;
    point.y = cell.y;
    point.z = cell.z;
    voxel_marker.points.push_back(point);
    voxel_marker.colors.push_back(make_color_for_source(cell.source_mask, cell.occupancy, 1.0F));
  }

  marker_array.markers.push_back(voxel_marker);
  return marker_array;
}

std::vector<tf2::Vector3> CollisionVoxelLayerNode::prefilter_depth_points(
  const std::vector<tf2::Vector3> & points) const
{
  return prefilter_points(points, depth_voxel_prefilter_);
}

std::vector<tf2::Vector3> CollisionVoxelLayerNode::prefilter_lidar_points(
  const std::vector<tf2::Vector3> & points) const
{
  return prefilter_points(points, lidar_voxel_prefilter_);
}

std::vector<tf2::Vector3> CollisionVoxelLayerNode::prefilter_points(
  const std::vector<tf2::Vector3> & points,
  double resolution) const
{
  if (points.empty() || resolution <= 0.0) {
    return points;
  }

  const double safe_resolution = std::max(0.001, resolution);
  std::unordered_set<VoxelKey, VoxelKeyHash> seen;
  std::vector<tf2::Vector3> filtered;
  filtered.reserve(points.size());

  for (const auto & point : points) {
    const VoxelKey key{
      quantize(point.x(), safe_resolution),
      quantize(point.y(), safe_resolution),
      quantize(point.z(), safe_resolution)
    };
    if (seen.insert(key).second) {
      filtered.push_back(point);
    }
  }

  return filtered;
}

std::vector<tf2::Vector3> CollisionVoxelLayerNode::collect_sparse_points(
  const rclcpp::Time & now) const
{
  std::vector<tf2::Vector3> points;

  for (const auto & source : scan_sources_) {
    if (!source.seen || (now - source.last_receive_time).seconds() > source_timeout_s_) {
      continue;
    }
    points.insert(points.end(), source.points.begin(), source.points.end());
  }

  if (depth_sub_ && depth_seen_ &&
    (now - last_depth_receive_time_).seconds() <= source_timeout_s_)
  {
    points.insert(points.end(), depth_points_.begin(), depth_points_.end());
  }

  if (lidar_sub_ && lidar_seen_ &&
    (now - last_lidar_receive_time_).seconds() <= source_timeout_s_)
  {
    points.insert(points.end(), lidar_points_.begin(), lidar_points_.end());
  }

  return points;
}

bool CollisionVoxelLayerNode::voxel_output_enabled() const
{
  return publish_voxel_grid_ || visualization_enabled_ || publish_markers_ || publish_debug_cloud_;
}

}  // namespace collision_voxel_layer
