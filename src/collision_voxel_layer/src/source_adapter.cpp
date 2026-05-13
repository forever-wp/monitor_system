#include "collision_voxel_layer/source_adapter.hpp"

#include <algorithm>
#include <cmath>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace collision_voxel_layer
{

std::vector<tf2::Vector3> convert_scan_to_points(
  const sensor_msgs::msg::LaserScan & scan,
  const tf2::Transform & transform,
  bool apply_transform,
  const ScanColumnParams & params)
{
  std::vector<tf2::Vector3> points;

  auto point_step = std::max<std::size_t>(1U, params.point_step);
  if (params.max_points > 0U && scan.ranges.size() > params.max_points) {
    const auto max_points_step = (scan.ranges.size() + params.max_points - 1U) / params.max_points;
    point_step = std::max(point_step, max_points_step);
  }
  const auto reserve_size = params.max_points > 0U ?
    std::min(params.max_points, (scan.ranges.size() + point_step - 1U) / point_step) :
    (scan.ranges.size() + point_step - 1U) / point_step;
  points.reserve(reserve_size);

  double angle = scan.angle_min;
  std::size_t point_index = 0U;
  for (const auto range : scan.ranges) {
    const bool sampled = point_index % point_step == 0U;
    ++point_index;
    if (!sampled) {
      angle += scan.angle_increment;
      continue;
    }
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

    points.push_back(hit);
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

  const auto total_points = static_cast<std::size_t>(cloud.width) * cloud.height;
  auto point_step = std::max<std::size_t>(1U, params.point_step);
  if (params.max_points > 0U && total_points > params.max_points) {
    const auto max_points_step = (total_points + params.max_points - 1U) / params.max_points;
    point_step = std::max(point_step, max_points_step);
  }
  const auto reserve_size = params.max_points > 0U ?
    std::min(params.max_points, (total_points + point_step - 1U) / point_step) :
    (total_points + point_step - 1U) / point_step;
  points.reserve(reserve_size);

  std::size_t point_index = 0U;
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++point_index) {
    if (point_index % point_step != 0U) {
      continue;
    }
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
      continue;
    }

    const tf2::Vector3 source_point(*iter_x, *iter_y, *iter_z);
    const double range_xy = std::hypot(source_point.x(), source_point.y());
    if (range_xy < params.min_range || range_xy > params.max_range) {
      continue;
    }
    if (source_point.z() < params.min_height || source_point.z() > params.max_height) {
      continue;
    }

    tf2::Vector3 point = source_point;
    if (apply_transform) {
      point = transform * point;
    }

    points.push_back(point);
  }

  return points;
}

tf2::Transform make_extrinsic_transform(const ExtrinsicTransformParams & params)
{
  tf2::Quaternion rotation = params.rotation;
  if (rotation.length2() <= 1e-12) {
    rotation.setValue(0.0, 0.0, 0.0, 1.0);
  } else {
    rotation.normalize();
  }
  return tf2::Transform(rotation, params.translation);
}

}  // namespace collision_voxel_layer
