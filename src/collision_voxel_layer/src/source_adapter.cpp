#include "collision_voxel_layer/source_adapter.hpp"

#include <algorithm>
#include <cmath>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace collision_voxel_layer
{

std::vector<tf2::Vector3> expand_scan_hit_column(
  double x,
  double y,
  double z_min,
  double z_max,
  double voxel_size_z)
{
  std::vector<tf2::Vector3> points;
  const double z_step = std::max(0.001, voxel_size_z);

  for (double z = z_min; z <= z_max + 1e-9; z += z_step) {
    points.emplace_back(x, y, z);
  }

  return points;
}

std::vector<tf2::Vector3> convert_scan_to_points(
  const sensor_msgs::msg::LaserScan & scan,
  const tf2::Transform & transform,
  bool apply_transform,
  const ScanColumnParams & params)
{
  std::vector<tf2::Vector3> points;

  double angle = scan.angle_min;
  for (const auto range : scan.ranges) {
    if (!std::isfinite(range) ||
      range < scan.range_min || range > scan.range_max ||
      range < params.min_range || range > params.max_range)
    {
      angle += scan.angle_increment;
      continue;
    }

    tf2::Vector3 hit(range * std::cos(angle), range * std::sin(angle), 0.0);
    if (apply_transform) {
      hit = transform * hit;
    }

    auto column = expand_scan_hit_column(
      hit.x(), hit.y(), params.z_min, params.z_max, params.voxel_size_z);
    points.insert(points.end(), column.begin(), column.end());
    angle += scan.angle_increment;
  }

  return points;
}

std::vector<tf2::Vector3> filter_depth_cloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const tf2::Transform & transform,
  bool apply_transform,
  const DepthFilterParams & params)
{
  std::vector<tf2::Vector3> points;
  points.reserve(cloud.width * cloud.height);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
      continue;
    }

    tf2::Vector3 point(*iter_x, *iter_y, *iter_z);
    if (apply_transform) {
      point = transform * point;
    }

    const double range_xy = std::hypot(point.x(), point.y());
    if (range_xy < params.min_range || range_xy > params.max_range) {
      continue;
    }
    if (point.z() < params.min_height || point.z() > params.max_height) {
      continue;
    }

    points.push_back(point);
  }

  return points;
}

}  // namespace collision_voxel_layer
