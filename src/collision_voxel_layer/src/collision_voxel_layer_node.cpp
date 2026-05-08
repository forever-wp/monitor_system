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

std_msgs::msg::ColorRGBA make_color(float occupancy, float occupancy_max)
{
  const float normalized = std::clamp(
    occupancy_max > 0.0F ? occupancy / occupancy_max : 0.0F,
    0.0F, 1.0F);

  std_msgs::msg::ColorRGBA color;
  color.r = normalized;
  color.g = 1.0F - normalized;
  color.b = 0.2F;
  color.a = std::max(0.2F, normalized);
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
  initial_config.depth_cloud_topic = this->declare_parameter<std::string>(
    "depth_cloud_topic", "/camera/depth/points");
  initial_config.grid_topic = this->declare_parameter<std::string>(
    "grid_topic", "/collision_voxel_layer/grid");
  initial_config.markers_topic = this->declare_parameter<std::string>(
    "markers_topic", "/collision_voxel_layer/markers");
  initial_config.debug_cloud_topic = this->declare_parameter<std::string>(
    "debug_cloud_topic", "/collision_voxel_layer/debug_cloud");
  initial_config.source_status_topic = this->declare_parameter<std::string>(
    "source_status_topic", "/collision_voxel_layer/source_status");
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
  initial_config.scan_min_range = this->declare_parameter<double>("scan_min_range", 0.05);
  initial_config.scan_max_range = this->declare_parameter<double>("scan_max_range", 8.0);
  initial_config.scan_z_min = this->declare_parameter<double>("scan_z_min", 0.0);
  initial_config.scan_z_max = this->declare_parameter<double>("scan_z_max", 0.4);
  initial_config.scan_weight = this->declare_parameter<double>("scan_weight", 0.6);
  initial_config.depth_min_range = this->declare_parameter<double>("depth_min_range", 0.1);
  initial_config.depth_max_range = this->declare_parameter<double>("depth_max_range", 4.0);
  initial_config.depth_min_height = this->declare_parameter<double>("depth_min_height", -0.1);
  initial_config.depth_max_height = this->declare_parameter<double>("depth_max_height", 1.5);
  initial_config.depth_weight = this->declare_parameter<double>("depth_weight", 0.8);
  initial_config.depth_voxel_prefilter = this->declare_parameter<double>(
    "depth_voxel_prefilter", initial_config.voxel_size_xy);

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
    "collision_voxel_layer started: scan=%s depth=%s base_frame=%s grid=%s markers=%s debug_cloud=%s source_status=%s",
    scan_topic_.c_str(), depth_cloud_topic_.c_str(), base_frame_.c_str(),
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
  if (config.scan_min_range < 0.0 || config.scan_max_range < config.scan_min_range) {
    error = "scan range bounds are invalid";
    return false;
  }
  if (config.scan_z_max < config.scan_z_min) {
    error = "scan z bounds are invalid";
    return false;
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
  if (config.depth_weight < 0.0) {
    error = "depth_weight must be >= 0.0";
    return false;
  }
  if (config.depth_voxel_prefilter < 0.0) {
    error = "depth_voxel_prefilter must be >= 0.0";
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
  depth_cloud_topic_ = config.depth_cloud_topic;
  grid_topic_ = config.grid_topic;
  markers_topic_ = config.markers_topic;
  debug_cloud_topic_ = config.debug_cloud_topic;
  source_status_topic_ = config.source_status_topic;
  publish_rate_hz_ = config.publish_rate_hz;
  tf_timeout_s_ = config.tf_timeout_s;
  sync_queue_size_ = config.sync_queue_size;
  source_timeout_s_ = config.source_timeout_s;
  source_health_check_period_s_ = config.source_health_check_period_s;
  scan_weight_ = config.scan_weight;
  depth_weight_ = config.depth_weight;
  depth_voxel_prefilter_ = config.depth_voxel_prefilter;

  scan_params_.min_range = config.scan_min_range;
  scan_params_.max_range = config.scan_max_range;
  scan_params_.z_min = config.scan_z_min;
  scan_params_.z_max = config.scan_z_max;
  scan_params_.voxel_size_z = config.voxel_size_z;

  depth_params_.min_range = config.depth_min_range;
  depth_params_.max_range = config.depth_max_range;
  depth_params_.min_height = config.depth_min_height;
  depth_params_.max_height = config.depth_max_height;

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
  markers_pub_.reset();
  debug_cloud_pub_.reset();
  source_status_pub_.reset();

  grid_pub_ = this->create_publisher<msg::VoxelGrid>(
    grid_topic_, rclcpp::QoS(1).transient_local().reliable());
  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    markers_topic_, rclcpp::QoS(1).transient_local().reliable());
  debug_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    debug_cloud_topic_, rclcpp::QoS(1).transient_local().reliable());
  source_status_pub_ = this->create_publisher<std_msgs::msg::String>(
    source_status_topic_, rclcpp::QoS(1).transient_local().reliable());
}

void CollisionVoxelLayerNode::rebuild_input_pipeline()
{
  scan_sub_.reset();
  depth_sub_.reset();
  scan_seen_ = false;
  depth_seen_ = false;
  last_scan_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_depth_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_graph_health_check_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  if (scan_topic_.empty() && depth_cloud_topic_.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "collision_voxel_layer inputs are disabled because scan_topic and depth_cloud_topic are empty");
    return;
  }

  if (!scan_topic_.empty()) {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&CollisionVoxelLayerNode::on_scan, this, std::placeholders::_1));
  } else {
    RCLCPP_WARN(get_logger(), "collision_voxel_layer scan input disabled: scan_topic is empty");
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
    if (name == "scan_z_min") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("scan_z_min must be a double");
      }
      next_config.scan_z_min = parameter.as_double();
      has_runtime_change = true;
      continue;
    }
    if (name == "scan_z_max") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        return reject("scan_z_max must be a double");
      }
      next_config.scan_z_max = parameter.as_double();
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

void CollisionVoxelLayerNode::on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  tf2::Transform transform;
  if (!lookup_transform(msg->header.frame_id, msg->header.stamp, transform)) {
    return;
  }

  const auto stamp = msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0 ?
    rclcpp::Time(msg->header.stamp) : this->get_clock()->now();
  sparse_grid_->decay_to(stamp);

  const bool apply_transform = !msg->header.frame_id.empty() && msg->header.frame_id != base_frame_;
  const auto scan_points = convert_scan_to_points(*msg, transform, apply_transform, scan_params_);
  for (const auto & point : scan_points) {
    sparse_grid_->insert_point(
      point.x(), point.y(), point.z(),
      static_cast<float>(scan_weight_),
      kScanSourceMask,
      stamp);
  }

  scan_seen_ = true;
  last_scan_stamp_ = stamp;
  log_missing_sources("scan", stamp);
  publish_state(stamp);
}

void CollisionVoxelLayerNode::on_depth_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  tf2::Transform transform;
  if (!lookup_transform(msg->header.frame_id, msg->header.stamp, transform)) {
    return;
  }

  const auto stamp = msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0 ?
    rclcpp::Time(msg->header.stamp) : this->get_clock()->now();
  sparse_grid_->decay_to(stamp);

  const bool apply_transform = !msg->header.frame_id.empty() && msg->header.frame_id != base_frame_;
  auto depth_points = filter_depth_cloud(*msg, transform, apply_transform, depth_params_);
  depth_points = prefilter_depth_points(depth_points);
  for (const auto & point : depth_points) {
    sparse_grid_->insert_point(
      point.x(), point.y(), point.z(),
      static_cast<float>(depth_weight_),
      kDepthSourceMask,
      stamp);
  }

  depth_seen_ = true;
  last_depth_stamp_ = stamp;
  log_missing_sources("depth", stamp);
  publish_state(stamp);
}

void CollisionVoxelLayerNode::on_decay_timer()
{
  const auto now = this->get_clock()->now();
  sparse_grid_->decay_to(now);
  log_missing_sources("timer", now);
  log_graph_health();
  publish_state(now);
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
  std_msgs::msg::Header header;
  header.frame_id = base_frame_;
  header.stamp = static_cast<builtin_interfaces::msg::Time>(stamp);

  auto grid_msg = sparse_grid_->export_grid(header);
  grid_pub_->publish(grid_msg);
  markers_pub_->publish(build_markers(grid_msg));
  debug_cloud_pub_->publish(build_debug_cloud(grid_msg));
}

void CollisionVoxelLayerNode::publish_source_status(
  const std::string & active_source,
  const rclcpp::Time & now)
{
  std_msgs::msg::String status;
  const bool scan_stale =
    scan_sub_ && (!scan_seen_ || (now - last_scan_stamp_).seconds() > source_timeout_s_);
  const bool depth_stale =
    depth_sub_ && (!depth_seen_ || (now - last_depth_stamp_).seconds() > source_timeout_s_);
  const bool any_source_active =
    (scan_sub_ && scan_seen_ && !scan_stale) || (depth_sub_ && depth_seen_ && !depth_stale);

  std::ostringstream stream;
  stream << "{"
         << "\"active_source\":\"" << active_source << "\","
         << "\"any_source_active\":" << (any_source_active ? "true" : "false") << ","
         << "\"source_timeout_s\":" << source_timeout_s_ << ","
         << "\"scan\":{"
         << "\"enabled\":" << (scan_sub_ ? "true" : "false") << ","
         << "\"topic\":\"" << scan_topic_ << "\","
         << "\"seen\":" << (scan_seen_ ? "true" : "false") << ","
         << "\"stale\":" << (scan_stale ? "true" : "false")
         << "},"
         << "\"depth\":{"
         << "\"enabled\":" << (depth_sub_ ? "true" : "false") << ","
         << "\"topic\":\"" << depth_cloud_topic_ << "\","
         << "\"seen\":" << (depth_seen_ ? "true" : "false") << ","
         << "\"stale\":" << (depth_stale ? "true" : "false")
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
  if (scan_sub_) {
    const bool scan_stale =
      !scan_seen_ || (now - last_scan_stamp_).seconds() > source_timeout_s_;
    if (scan_stale) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "collision_voxel_layer missing or stale scan input: topic=%s active_source=%s seen=%s timeout_s=%.2f",
        scan_topic_.c_str(), active_source.c_str(), scan_seen_ ? "true" : "false",
        source_timeout_s_);
    }
  }

  if (depth_sub_) {
    const bool depth_stale =
      !depth_seen_ || (now - last_depth_stamp_).seconds() > source_timeout_s_;
    if (depth_stale) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "collision_voxel_layer missing or stale depth input: topic=%s active_source=%s seen=%s timeout_s=%.2f",
        depth_cloud_topic_.c_str(), active_source.c_str(), depth_seen_ ? "true" : "false",
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

  if (scan_sub_) {
    const auto publishers = this->get_publishers_info_by_topic(scan_topic_);
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
        scan_topic_.c_str());
    } else if (!type_ok) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "collision_voxel_layer scan topic type mismatch: topic=%s expected=sensor_msgs/msg/LaserScan",
        scan_topic_.c_str());
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
    voxel_marker.colors.push_back(make_color(cell.occupancy, 1.0F));
  }

  marker_array.markers.push_back(voxel_marker);
  return marker_array;
}

std::vector<tf2::Vector3> CollisionVoxelLayerNode::prefilter_depth_points(
  const std::vector<tf2::Vector3> & points) const
{
  if (points.empty() || depth_voxel_prefilter_ <= 0.0) {
    return points;
  }

  const double resolution = std::max(0.001, depth_voxel_prefilter_);
  std::unordered_set<VoxelKey, VoxelKeyHash> seen;
  std::vector<tf2::Vector3> filtered;
  filtered.reserve(points.size());

  for (const auto & point : points) {
    const VoxelKey key{
      quantize(point.x(), resolution),
      quantize(point.y(), resolution),
      quantize(point.z(), resolution)
    };
    if (seen.insert(key).second) {
      filtered.push_back(point);
    }
  }

  return filtered;
}

}  // namespace collision_voxel_layer
