#ifndef COLLISION_VOXEL_LAYER__COLLISION_VOXEL_LAYER_NODE_HPP_
#define COLLISION_VOXEL_LAYER__COLLISION_VOXEL_LAYER_NODE_HPP_

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "collision_voxel_layer/source_adapter.hpp"
#include "collision_voxel_layer/sparse_voxel_grid.hpp"

namespace collision_voxel_layer
{

class CollisionVoxelLayerNode : public rclcpp::Node
{
public:
  explicit CollisionVoxelLayerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  bool reload_config_if_needed(bool force = false);

  const std::string & resolved_config_file() const {return resolved_config_file_;}
  double publish_rate_hz() const {return publish_rate_hz_;}
  double scan_weight() const {return scan_weight_;}
  double depth_weight() const {return depth_weight_;}
  double config_reload_period_s() const {return config_reload_period_s_;}
  std::size_t sync_queue_size() const {return sync_queue_size_;}

private:
  struct RuntimeConfig
  {
    std::string config_file{""};
    bool config_reload_enabled{true};
    double config_reload_period_s{1.0};
    std::string base_frame{"base_link"};
    std::string scan_topic{"/scan"};
    std::string depth_cloud_topic{"/camera/depth/points"};
    std::string grid_topic{"/collision_voxel_layer/grid"};
    std::string markers_topic{"/collision_voxel_layer/markers"};
    std::string debug_cloud_topic{"/collision_voxel_layer/debug_cloud"};
    std::string source_status_topic{"/collision_voxel_layer/source_status"};
    double publish_rate_hz{10.0};
    double tf_timeout_s{0.05};
    std::size_t sync_queue_size{20U};
    // Deprecated compatibility for old synchronized-input configs.
    double sync_slop_s{0.15};
    double source_timeout_s{1.0};
    double source_health_check_period_s{1.0};
    double voxel_size_xy{0.10};
    double voxel_size_z{0.10};
    double voxel_decay_time_s{1.0};
    double prune_threshold{0.05};
    double occupancy_max{1.0};
    double scan_min_range{0.05};
    double scan_max_range{8.0};
    double scan_z_min{0.0};
    double scan_z_max{0.4};
    double scan_weight{0.6};
    double depth_min_range{0.1};
    double depth_max_range{4.0};
    double depth_min_height{-0.1};
    double depth_max_height{1.5};
    double depth_weight{0.8};
    double depth_voxel_prefilter{0.10};
  };

  struct FileState
  {
    bool exists{false};
    std::filesystem::file_time_type mtime{};
  };

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void on_depth_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void on_decay_timer();
  void on_config_reload_timer();

  bool lookup_transform(
    const std::string & source_frame,
    const builtin_interfaces::msg::Time & stamp,
    tf2::Transform & transform);

  bool validate_runtime_config(const RuntimeConfig & config, std::string & error) const;
  void apply_runtime_config(const RuntimeConfig & config);
  void rebuild_publishers();
  void rebuild_input_pipeline();
  void rebuild_decay_timer();
  void rebuild_config_reload_timer();
  void configure_config_file_tracking(bool sync_state);
  FileState capture_config_file_state() const;
  bool poll_config_file_changed();
  std::vector<rclcpp::Parameter> load_parameters_from_file(const std::string & path) const;
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters);

  void publish_state(const rclcpp::Time & stamp);
  void publish_source_status(const std::string & active_source, const rclcpp::Time & now);
  void log_missing_sources(const std::string & active_source, const rclcpp::Time & now);
  void log_graph_health(bool force = false);
  sensor_msgs::msg::PointCloud2 build_debug_cloud(const msg::VoxelGrid & grid) const;
  visualization_msgs::msg::MarkerArray build_markers(const msg::VoxelGrid & grid) const;
  std::vector<tf2::Vector3> prefilter_depth_points(const std::vector<tf2::Vector3> & points) const;

  RuntimeConfig current_config_;
  std::string config_file_;
  std::string resolved_config_file_;
  bool config_reload_enabled_{true};
  double config_reload_period_s_{1.0};
  std::string base_frame_{"base_link"};
  std::string scan_topic_{"/scan"};
  std::string depth_cloud_topic_{"/camera/depth/points"};
  std::string grid_topic_{"/collision_voxel_layer/grid"};
  std::string markers_topic_{"/collision_voxel_layer/markers"};
  std::string debug_cloud_topic_{"/collision_voxel_layer/debug_cloud"};
  std::string source_status_topic_{"/collision_voxel_layer/source_status"};
  double publish_rate_hz_{10.0};
  double tf_timeout_s_{0.05};
  double scan_weight_{0.6};
  double depth_weight_{0.8};
  double depth_voxel_prefilter_{0.0};
  std::size_t sync_queue_size_{20U};
  double source_timeout_s_{1.0};
  double source_health_check_period_s_{1.0};
  rclcpp::Time last_graph_health_check_time_{0, 0, RCL_ROS_TIME};

  ScanColumnParams scan_params_;
  DepthFilterParams depth_params_;

  std::unique_ptr<SparseVoxelGrid> sparse_grid_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub_;
  bool scan_seen_{false};
  bool depth_seen_{false};
  rclcpp::Time last_scan_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_depth_stamp_{0, 0, RCL_ROS_TIME};

  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<msg::VoxelGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr source_status_pub_;
  rclcpp::TimerBase::SharedPtr decay_timer_;
  rclcpp::TimerBase::SharedPtr config_reload_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  FileState last_config_file_state_;
  bool config_file_state_initialized_{false};
};

}  // namespace collision_voxel_layer

#endif  // COLLISION_VOXEL_LAYER__COLLISION_VOXEL_LAYER_NODE_HPP_
