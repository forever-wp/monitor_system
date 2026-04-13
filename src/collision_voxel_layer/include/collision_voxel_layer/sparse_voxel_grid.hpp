#ifndef COLLISION_VOXEL_LAYER__SPARSE_VOXEL_GRID_HPP_
#define COLLISION_VOXEL_LAYER__SPARSE_VOXEL_GRID_HPP_

#include <cstddef>
#include <cstdint>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "collision_voxel_layer/msg/voxel_grid.hpp"
#include "collision_voxel_layer/voxel_types.hpp"

namespace collision_voxel_layer
{

class SparseVoxelGrid
{
public:
  SparseVoxelGrid(
    double resolution_xy,
    double resolution_z,
    double decay_time_s,
    double prune_threshold,
    float occupancy_max);

  void insert_point(
    double x,
    double y,
    double z,
    float occupancy_delta,
    uint8_t source_mask,
    const rclcpp::Time & stamp);

  void decay_to(const rclcpp::Time & now);

  msg::VoxelGrid export_grid(const std_msgs::msg::Header & header) const;

  std::size_t size() const;

private:
  VoxelKey make_key(double x, double y, double z) const;
  void decay_state_to(VoxelState & state, const rclcpp::Time & now) const;

  double resolution_xy_{0.05};
  double resolution_z_{0.10};
  double decay_time_s_{1.0};
  double prune_threshold_{0.01};
  float occupancy_max_{1.0F};
  std::unordered_map<VoxelKey, VoxelState, VoxelKeyHash> voxels_;
};

}  // namespace collision_voxel_layer

#endif  // COLLISION_VOXEL_LAYER__SPARSE_VOXEL_GRID_HPP_
