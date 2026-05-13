#ifndef COLLISION_VOXEL_LAYER__SOURCE_ADAPTER_HPP_
#define COLLISION_VOXEL_LAYER__SOURCE_ADAPTER_HPP_

#include <cstddef>
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
  std::size_t point_step{1U};
  std::size_t max_points{0U};
};

struct DepthFilterParams
{
  double min_range{0.0};
  double max_range{5.0};
  double min_height{0.0};
  double max_height{2.0};
  std::size_t point_step{1U};
  std::size_t max_points{0U};
};

struct ExtrinsicTransformParams
{
  bool enabled{false};
  tf2::Vector3 translation{0.0, 0.0, 0.0};
  tf2::Quaternion rotation{0.0, 0.0, 0.0, 1.0};
};

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

tf2::Transform make_extrinsic_transform(const ExtrinsicTransformParams & params);

}  // namespace collision_voxel_layer

#endif  // COLLISION_VOXEL_LAYER__SOURCE_ADAPTER_HPP_
