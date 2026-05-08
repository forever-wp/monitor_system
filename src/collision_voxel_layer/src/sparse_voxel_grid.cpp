#include "collision_voxel_layer/sparse_voxel_grid.hpp"

#include <algorithm>
#include <cmath>

namespace collision_voxel_layer
{

namespace
{

int32_t quantize(double value, double resolution)
{
  return static_cast<int32_t>(std::floor(value / resolution));
}

float clamp_occupancy(float value, float occupancy_max)
{
  return std::clamp(value, 0.0F, occupancy_max);
}

bool source_bit_set(uint8_t mask, std::size_t bit)
{
  return (mask & static_cast<uint8_t>(1U << bit)) != 0U;
}

}  // namespace

SparseVoxelGrid::SparseVoxelGrid(
  double resolution_xy,
  double resolution_z,
  double decay_time_s,
  double prune_threshold,
  float occupancy_max)
: resolution_xy_(std::max(0.001, resolution_xy)),
  resolution_z_(std::max(0.001, resolution_z)),
  decay_time_s_(std::max(0.001, decay_time_s)),
  prune_threshold_(std::max(0.0, prune_threshold)),
  occupancy_max_(std::max(0.001F, occupancy_max))
{
}

VoxelKey SparseVoxelGrid::make_key(double x, double y, double z) const
{
  return VoxelKey{
    quantize(x, resolution_xy_),
    quantize(y, resolution_xy_),
    quantize(z, resolution_z_)
  };
}

void SparseVoxelGrid::recompute_state(VoxelState & state) const
{
  float occupancy_sum = 0.0F;
  uint8_t source_mask = 0U;
  for (std::size_t bit = 0; bit < state.source_occupancy.size(); ++bit) {
    if (state.source_occupancy[bit] <= 0.0F) {
      state.source_occupancy[bit] = 0.0F;
      continue;
    }
    occupancy_sum += state.source_occupancy[bit];
    source_mask |= static_cast<uint8_t>(1U << bit);
  }

  state.occupancy = clamp_occupancy(occupancy_sum, occupancy_max_);
  state.source_mask = source_mask;
}

void SparseVoxelGrid::decay_state_to(VoxelState & state, const rclcpp::Time & now) const
{
  if (state.last_update.nanoseconds() == 0) {
    state.last_update = now;
    return;
  }

  const double dt = (now - state.last_update).seconds();
  if (dt <= 0.0) {
    return;
  }

  const auto decay_factor = static_cast<float>(std::exp(-dt / decay_time_s_));
  for (auto & source_occupancy : state.source_occupancy) {
    source_occupancy *= decay_factor;
  }
  recompute_state(state);
  state.last_update = now;
}

void SparseVoxelGrid::insert_point(
  double x,
  double y,
  double z,
  float occupancy_delta,
  uint8_t source_mask,
  const rclcpp::Time & stamp)
{
  auto & state = voxels_[make_key(x, y, z)];
  decay_state_to(state, stamp);
  for (std::size_t bit = 0; bit < state.source_occupancy.size(); ++bit) {
    if (source_bit_set(source_mask, bit)) {
      state.source_occupancy[bit] = clamp_occupancy(
        state.source_occupancy[bit] + occupancy_delta, occupancy_max_);
    }
  }
  recompute_state(state);
  state.last_update = stamp;
}

void SparseVoxelGrid::clear_source(uint8_t source_mask, const rclcpp::Time & now)
{
  for (auto it = voxels_.begin(); it != voxels_.end();) {
    decay_state_to(it->second, now);
    for (std::size_t bit = 0; bit < it->second.source_occupancy.size(); ++bit) {
      if (source_bit_set(source_mask, bit)) {
        it->second.source_occupancy[bit] = 0.0F;
      }
    }
    recompute_state(it->second);
    it->second.last_update = now;
    if (it->second.occupancy < static_cast<float>(prune_threshold_)) {
      it = voxels_.erase(it);
      continue;
    }
    ++it;
  }
}

void SparseVoxelGrid::decay_to(const rclcpp::Time & now)
{
  for (auto it = voxels_.begin(); it != voxels_.end();) {
    decay_state_to(it->second, now);
    if (it->second.occupancy < static_cast<float>(prune_threshold_)) {
      it = voxels_.erase(it);
      continue;
    }
    ++it;
  }
}

msg::VoxelGrid SparseVoxelGrid::export_grid(const std_msgs::msg::Header & header) const
{
  msg::VoxelGrid msg;
  msg.header = header;
  msg.resolution_xy = static_cast<float>(resolution_xy_);
  msg.resolution_z = static_cast<float>(resolution_z_);
  msg.decay_time_s = static_cast<float>(decay_time_s_);
  msg.cells.reserve(voxels_.size());

  for (const auto & [key, state] : voxels_) {
    if (state.occupancy < static_cast<float>(prune_threshold_)) {
      continue;
    }

    msg::VoxelCell cell;
    cell.x = static_cast<float>((static_cast<double>(key.ix) + 0.5) * resolution_xy_);
    cell.y = static_cast<float>((static_cast<double>(key.iy) + 0.5) * resolution_xy_);
    cell.z = static_cast<float>((static_cast<double>(key.iz) + 0.5) * resolution_z_);
    cell.occupancy = state.occupancy;
    cell.source_mask = state.source_mask;
    msg.cells.push_back(cell);
  }

  return msg;
}

std::size_t SparseVoxelGrid::size() const
{
  return voxels_.size();
}

}  // namespace collision_voxel_layer
