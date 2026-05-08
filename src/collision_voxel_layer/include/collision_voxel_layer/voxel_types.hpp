#ifndef COLLISION_VOXEL_LAYER__VOXEL_TYPES_HPP_
#define COLLISION_VOXEL_LAYER__VOXEL_TYPES_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>

#include <rclcpp/rclcpp.hpp>

namespace collision_voxel_layer
{

struct VoxelKey
{
  int32_t ix{0};
  int32_t iy{0};
  int32_t iz{0};

  bool operator==(const VoxelKey & other) const
  {
    return ix == other.ix && iy == other.iy && iz == other.iz;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const noexcept
  {
    const auto hx = static_cast<std::size_t>(std::hash<int32_t>{}(key.ix));
    const auto hy = static_cast<std::size_t>(std::hash<int32_t>{}(key.iy));
    const auto hz = static_cast<std::size_t>(std::hash<int32_t>{}(key.iz));
    return hx ^ (hy << 1U) ^ (hz << 2U);
  }
};

struct VoxelState
{
  float occupancy{0.0F};
  uint8_t source_mask{0U};
  std::array<float, 8> source_occupancy{};
  rclcpp::Time last_update{0, 0, RCL_ROS_TIME};
};

}  // namespace collision_voxel_layer

#endif  // COLLISION_VOXEL_LAYER__VOXEL_TYPES_HPP_
