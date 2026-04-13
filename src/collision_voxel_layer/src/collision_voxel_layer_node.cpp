#include "collision_voxel_layer/collision_voxel_layer_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
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

rclcpp::Time newest_stamp(const builtin_interfaces::msg::Time & lhs, const builtin_interfaces::msg::Time & rhs)
{
  const rclcpp::Time lhs_time(lhs);
  const rclcpp::Time rhs_time(rhs);
  return lhs_time >= rhs_time ? lhs_time : rhs_time;
}

}  // namespace

CollisionVoxelLayerNode::CollisionVoxelLayerNode(const rclcpp::NodeOptions & options)
: Node("collision_voxel_layer", options),
  tf_buffer_(this->get_clock())
{
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  const auto scan_topic = this->declare_parameter<std::string>("scan_topic", "/scan");
  const auto depth_cloud_topic = this->declare_parameter<std::string>(
    "depth_cloud_topic", "/camera/depth/points");
  grid_topic_ = this->declare_parameter<std::string>(
    "grid_topic", "/collision_voxel_layer/grid");
  markers_topic_ = this->declare_parameter<std::string>(
    "markers_topic", "/collision_voxel_layer/markers");
  debug_cloud_topic_ = this->declare_parameter<std::string>(
    "debug_cloud_topic", "/collision_voxel_layer/debug_cloud");

  publish_rate_hz_ = std::max(0.5, this->declare_parameter<double>("publish_rate", 10.0));
  tf_timeout_s_ = std::max(0.0, this->declare_parameter<double>("tf_timeout_s", 0.05));
  sync_queue_size_ = static_cast<std::size_t>(std::max<int>(
    2, this->declare_parameter<int>("sync_queue_size", 20)));
  sync_slop_s_ = std::max(0.0, this->declare_parameter<double>("sync_slop_s", 0.15));

  const auto voxel_size_xy = this->declare_parameter<double>("voxel_size_xy", 0.10);
  const auto voxel_size_z = this->declare_parameter<double>("voxel_size_z", 0.10);
  const auto voxel_decay_time_s = this->declare_parameter<double>("voxel_decay_time_s", 1.0);
  const auto prune_threshold = this->declare_parameter<double>("prune_threshold", 0.05);
  const auto occupancy_max = static_cast<float>(
    this->declare_parameter<double>("occupancy_max", 1.0));

  scan_params_.min_range = this->declare_parameter<double>("scan_min_range", 0.05);
  scan_params_.max_range = this->declare_parameter<double>("scan_max_range", 8.0);
  scan_params_.z_min = this->declare_parameter<double>("scan_z_min", 0.0);
  scan_params_.z_max = this->declare_parameter<double>("scan_z_max", 0.4);
  scan_params_.voxel_size_z = voxel_size_z;
  scan_weight_ = this->declare_parameter<double>("scan_weight", 0.6);

  depth_params_.min_range = this->declare_parameter<double>("depth_min_range", 0.1);
  depth_params_.max_range = this->declare_parameter<double>("depth_max_range", 4.0);
  depth_params_.min_height = this->declare_parameter<double>("depth_min_height", -0.1);
  depth_params_.max_height = this->declare_parameter<double>("depth_max_height", 1.5);
  depth_weight_ = this->declare_parameter<double>("depth_weight", 0.8);
  depth_voxel_prefilter_ = std::max(
    0.0, this->declare_parameter<double>("depth_voxel_prefilter", voxel_size_xy));

  sparse_grid_ = std::make_unique<SparseVoxelGrid>(
    voxel_size_xy, voxel_size_z, voxel_decay_time_s, prune_threshold, occupancy_max);

  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_, this, true);

  grid_pub_ = this->create_publisher<msg::VoxelGrid>(
    grid_topic_, rclcpp::QoS(1).transient_local().reliable());
  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    markers_topic_, rclcpp::QoS(1).transient_local().reliable());
  debug_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    debug_cloud_topic_, rclcpp::QoS(1).transient_local().reliable());

  scan_sub_.subscribe(this, scan_topic, rmw_qos_profile_sensor_data);
  depth_sub_.subscribe(this, depth_cloud_topic, rmw_qos_profile_sensor_data);
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(static_cast<uint32_t>(sync_queue_size_)), scan_sub_, depth_sub_);
  sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_s_));
  sync_->registerCallback(std::bind(
    &CollisionVoxelLayerNode::on_synced_inputs,
    this,
    std::placeholders::_1,
    std::placeholders::_2));

  decay_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_hz_),
    std::bind(&CollisionVoxelLayerNode::on_decay_timer, this));

  RCLCPP_INFO(
    get_logger(),
    "collision_voxel_layer started: scan=%s depth=%s base_frame=%s grid=%s markers=%s debug_cloud=%s",
    scan_topic.c_str(), depth_cloud_topic.c_str(), base_frame_.c_str(),
    grid_topic_.c_str(), markers_topic_.c_str(), debug_cloud_topic_.c_str());
}

void CollisionVoxelLayerNode::on_synced_inputs(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & depth_msg)
{
  tf2::Transform scan_transform;
  tf2::Transform depth_transform;
  if (!lookup_transform(scan_msg->header.frame_id, scan_msg->header.stamp, scan_transform)) {
    return;
  }
  if (!lookup_transform(depth_msg->header.frame_id, depth_msg->header.stamp, depth_transform)) {
    return;
  }

  const auto stamp = newest_stamp(scan_msg->header.stamp, depth_msg->header.stamp);
  sparse_grid_->decay_to(stamp);

  const bool scan_apply_transform = !scan_msg->header.frame_id.empty() &&
    scan_msg->header.frame_id != base_frame_;
  const bool depth_apply_transform = !depth_msg->header.frame_id.empty() &&
    depth_msg->header.frame_id != base_frame_;

  const auto scan_points = convert_scan_to_points(
    *scan_msg, scan_transform, scan_apply_transform, scan_params_);
  auto depth_points = filter_depth_cloud(
    *depth_msg, depth_transform, depth_apply_transform, depth_params_);
  depth_points = prefilter_depth_points(depth_points);

  for (const auto & point : scan_points) {
    sparse_grid_->insert_point(
      point.x(), point.y(), point.z(),
      static_cast<float>(scan_weight_),
      kScanSourceMask,
      stamp);
  }

  for (const auto & point : depth_points) {
    sparse_grid_->insert_point(
      point.x(), point.y(), point.z(),
      static_cast<float>(depth_weight_),
      kDepthSourceMask,
      stamp);
  }

  publish_state(stamp);
}

void CollisionVoxelLayerNode::on_decay_timer()
{
  const auto now = this->get_clock()->now();
  sparse_grid_->decay_to(now);
  publish_state(now);
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
