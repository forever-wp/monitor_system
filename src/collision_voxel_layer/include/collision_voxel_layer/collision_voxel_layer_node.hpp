#ifndef COLLISION_VOXEL_LAYER__COLLISION_VOXEL_LAYER_NODE_HPP_
#define COLLISION_VOXEL_LAYER__COLLISION_VOXEL_LAYER_NODE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::LaserScan,
    sensor_msgs::msg::PointCloud2>;

  void on_synced_inputs(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & depth_msg);
  void on_decay_timer();

  bool lookup_transform(
    const std::string & source_frame,
    const builtin_interfaces::msg::Time & stamp,
    tf2::Transform & transform);

  void publish_state(const rclcpp::Time & stamp);
  sensor_msgs::msg::PointCloud2 build_debug_cloud(const msg::VoxelGrid & grid) const;
  visualization_msgs::msg::MarkerArray build_markers(const msg::VoxelGrid & grid) const;
  std::vector<tf2::Vector3> prefilter_depth_points(const std::vector<tf2::Vector3> & points) const;

  std::string base_frame_{"base_link"};
  std::string grid_topic_{"/collision_voxel_layer/grid"};
  std::string markers_topic_{"/collision_voxel_layer/markers"};
  std::string debug_cloud_topic_{"/collision_voxel_layer/debug_cloud"};
  double publish_rate_hz_{10.0};
  double tf_timeout_s_{0.05};
  double scan_weight_{0.6};
  double depth_weight_{0.8};
  double depth_voxel_prefilter_{0.0};
  std::size_t sync_queue_size_{20U};
  double sync_slop_s_{0.15};

  ScanColumnParams scan_params_;
  DepthFilterParams depth_params_;

  std::unique_ptr<SparseVoxelGrid> sparse_grid_;

  message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<msg::VoxelGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;
  rclcpp::TimerBase::SharedPtr decay_timer_;
};

}  // namespace collision_voxel_layer

#endif  // COLLISION_VOXEL_LAYER__COLLISION_VOXEL_LAYER_NODE_HPP_
