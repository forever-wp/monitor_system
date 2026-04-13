#ifndef COLLISION_VOXEL_LAYER__SOURCE_ADAPTER_HPP_
#define COLLISION_VOXEL_LAYER__SOURCE_ADAPTER_HPP_

#include <vector>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace collision_voxel_layer
{

struct ScanColumnParams
{
  double min_range{0.0};
  double max_range{10.0};
  double z_min{0.0};
  double z_max{0.3};
  double voxel_size_z{0.1};
};

struct DepthFilterParams
{
  double min_range{0.0};
  double max_range{5.0};
  double min_height{0.0};
  double max_height{2.0};
};

std::vector<tf2::Vector3> expand_scan_hit_column(
  double x,
  double y,
  double z_min,
  double z_max,
  double voxel_size_z);

std::vector<tf2::Vector3> convert_scan_to_points(
  const sensor_msgs::msg::LaserScan & scan,
  const tf2::Transform & transform,
  bool apply_transform,
  const ScanColumnParams & params);

std::vector<tf2::Vector3> filter_depth_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const tf2::Transform & transform,
  bool apply_transform,
  const DepthFilterParams & params);

}  // namespace collision_voxel_layer

#endif  // COLLISION_VOXEL_LAYER__SOURCE_ADAPTER_HPP_
