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
#include <std_msgs/msg/bool.hpp>
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
    std::vector<std::string> scan_topics{"/scan"};
    std::vector<int64_t> scan_point_steps{1};
    std::vector<int64_t> scan_max_points{0};
    std::vector<double> scan_voxel_prefilters{0.0};
    std::string depth_cloud_topic{"/camera/depth/points"};
    std::string lidar_cloud_topic{"/livox/points"};
    std::string depth_source_frame{"camera_color_optical_frame"};
    std::vector<double> depth_extrinsic_xyz{0.363, -0.040, -0.284};
    std::vector<double> depth_extrinsic_qxyzw{0.0, 0.216440, 0.0, 0.976296};
    bool depth_use_extrinsic_fallback{true};
    std::string points_topic{"/collision_voxel_layer/points"};
    std::string fused_scan_topic{"/collision_voxel_layer/scan"};
    std::string grid_topic{"/collision_voxel_layer/grid"};
    std::string markers_topic{"/collision_voxel_layer/markers"};
    std::string debug_cloud_topic{"/collision_voxel_layer/debug_cloud"};
    std::string source_status_topic{"/collision_voxel_layer/source_status"};
    std::string visualization_control_topic{"/collision_voxel_layer/visualization_enabled"};
    bool publish_points{true};
    bool publish_fused_scan{true};
    bool publish_voxel_grid{false};
    bool visualization_enabled{false};
    bool publish_markers{false};
    bool publish_debug_cloud{false};
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
    std::vector<double> voxel_region_xy{-1.0, -2.5, 4.0, -2.5, 4.0, 2.5, -1.0, 2.5};
    double scan_min_range{0.05};
    double scan_max_range{8.0};
    double scan_weight{0.6};
    double fused_scan_angle_min{-3.141592653589793};
    double fused_scan_angle_max{3.141592653589793};
    double fused_scan_angle_increment{0.017453292519943295};
    double fused_scan_range_min{0.05};
    double fused_scan_range_max{8.0};
    double depth_min_range{0.1};
    double depth_max_range{4.0};
    double depth_min_height{-0.1};
    double depth_max_height{1.5};
    double depth_min_base_height{0.08};
    double depth_max_base_height{1.8};
    double depth_weight{0.8};
    double depth_voxel_prefilter{0.10};
    double depth_process_rate_hz{5.0};
    std::size_t depth_point_step{4U};
    std::size_t depth_max_points{1200U};
    double lidar_min_range{0.1};
    double lidar_max_range{8.0};
    double lidar_min_height{0.05};
    double lidar_max_height{1.8};
    double lidar_weight{0.8};
    double lidar_voxel_prefilter{0.10};
    double lidar_process_rate_hz{5.0};
    std::size_t lidar_point_step{8U};
    std::size_t lidar_max_points{1800U};
  };

  struct FileState
  {
    bool exists{false};
    std::filesystem::file_time_type mtime{};
  };

  struct ScanSourceState
  {
    std::string topic;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription;
    bool seen{false};
    rclcpp::Time last_stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_receive_time{0, 0, RCL_ROS_TIME};
    std::vector<tf2::Vector3> points;
    ScanColumnParams params;
    double voxel_prefilter{0.0};
  };

  void on_scan(std::size_t source_index, const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void on_depth_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void on_lidar_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void on_visualization_control(const std_msgs::msg::Bool::SharedPtr msg);
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
  void publish_grid_only(const rclcpp::Time & stamp);
  void publish_fused_outputs(const rclcpp::Time & stamp);
  void publish_source_status(const std::string & active_source, const rclcpp::Time & now);
  void log_missing_sources(const std::string & active_source, const rclcpp::Time & now);
  void log_graph_health(bool force = false);
  bool any_scan_source_seen() const;
  bool any_scan_source_active(const rclcpp::Time & now) const;
  bool point_in_voxel_region(const tf2::Vector3 & point) const;
  void rebuild_scan_voxels(const rclcpp::Time & now);
  sensor_msgs::msg::PointCloud2 build_debug_cloud(const msg::VoxelGrid & grid) const;
  sensor_msgs::msg::PointCloud2 build_sparse_cloud(
    const std_msgs::msg::Header & header,
    const std::vector<tf2::Vector3> & points) const;
  sensor_msgs::msg::LaserScan build_fused_scan(
    const std_msgs::msg::Header & header,
    const std::vector<tf2::Vector3> & points) const;
  visualization_msgs::msg::MarkerArray build_markers(const msg::VoxelGrid & grid) const;
  std::vector<tf2::Vector3> prefilter_depth_points(const std::vector<tf2::Vector3> & points) const;
  std::vector<tf2::Vector3> prefilter_lidar_points(const std::vector<tf2::Vector3> & points) const;
  std::vector<tf2::Vector3> prefilter_points(
    const std::vector<tf2::Vector3> & points,
    double resolution) const;
  std::vector<tf2::Vector3> collect_sparse_points(const rclcpp::Time & now) const;
  bool voxel_output_enabled() const;

  RuntimeConfig current_config_;
  std::string config_file_;
  std::string resolved_config_file_;
  bool config_reload_enabled_{true};
  double config_reload_period_s_{1.0};
  std::string base_frame_{"base_link"};
  std::string scan_topic_{"/scan"};
  std::vector<std::string> scan_topics_{"/scan"};
  std::vector<int64_t> scan_point_steps_{1};
  std::vector<int64_t> scan_max_points_{0};
  std::vector<double> scan_voxel_prefilters_{0.0};
  std::string depth_cloud_topic_{"/camera/depth/points"};
  std::string lidar_cloud_topic_{"/livox/points"};
  std::string depth_source_frame_{"camera_color_optical_frame"};
  std::vector<double> depth_extrinsic_xyz_{0.363, -0.040, -0.284};
  std::vector<double> depth_extrinsic_qxyzw_{0.0, 0.216440, 0.0, 0.976296};
  bool depth_use_extrinsic_fallback_{true};
  std::string points_topic_{"/collision_voxel_layer/points"};
  std::string fused_scan_topic_{"/collision_voxel_layer/scan"};
  std::string grid_topic_{"/collision_voxel_layer/grid"};
  std::string markers_topic_{"/collision_voxel_layer/markers"};
  std::string debug_cloud_topic_{"/collision_voxel_layer/debug_cloud"};
  std::string source_status_topic_{"/collision_voxel_layer/source_status"};
  std::string visualization_control_topic_{"/collision_voxel_layer/visualization_enabled"};
  bool publish_points_{true};
  bool publish_fused_scan_{true};
  bool publish_voxel_grid_{false};
  bool visualization_enabled_{false};
  bool publish_markers_{false};
  bool publish_debug_cloud_{false};
  double publish_rate_hz_{10.0};
  double tf_timeout_s_{0.05};
  double scan_weight_{0.6};
  double fused_scan_angle_min_{-3.141592653589793};
  double fused_scan_angle_max_{3.141592653589793};
  double fused_scan_angle_increment_{0.017453292519943295};
  double fused_scan_range_min_{0.05};
  double fused_scan_range_max_{8.0};
  double depth_weight_{0.8};
  double depth_voxel_prefilter_{0.0};
  double depth_process_period_s_{0.2};
  double lidar_weight_{0.8};
  double lidar_voxel_prefilter_{0.0};
  double lidar_process_period_s_{0.2};
  double depth_min_base_height_{0.08};
  double depth_max_base_height_{1.8};
  std::vector<double> voxel_region_xy_{-1.0, -2.5, 4.0, -2.5, 4.0, 2.5, -1.0, 2.5};
  std::size_t sync_queue_size_{20U};
  double source_timeout_s_{1.0};
  double source_health_check_period_s_{1.0};
  rclcpp::Time last_graph_health_check_time_{0, 0, RCL_ROS_TIME};

  ScanColumnParams scan_params_;
  DepthFilterParams depth_params_;
  DepthFilterParams lidar_params_;
  ExtrinsicTransformParams depth_extrinsic_params_;

  std::unique_ptr<SparseVoxelGrid> sparse_grid_;

  std::vector<ScanSourceState> scan_sources_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  bool depth_seen_{false};
  rclcpp::Time last_depth_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_depth_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_depth_process_time_{0, 0, RCL_ROS_TIME};
  std::vector<tf2::Vector3> depth_points_;
  bool lidar_seen_{false};
  rclcpp::Time last_lidar_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_lidar_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_lidar_process_time_{0, 0, RCL_ROS_TIME};
  std::vector<tf2::Vector3> lidar_points_;

  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<msg::VoxelGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr fused_scan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr source_status_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr visualization_control_sub_;
  rclcpp::TimerBase::SharedPtr decay_timer_;
  rclcpp::TimerBase::SharedPtr config_reload_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  FileState last_config_file_state_;
  bool config_file_state_initialized_{false};
};

}  // namespace collision_voxel_layer

#endif  // COLLISION_VOXEL_LAYER__COLLISION_VOXEL_LAYER_NODE_HPP_
