#include "collision_voxel_layer/source_adapter.hpp"

#include <limits>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "gtest/gtest.h"

TEST(SourceAdapterTest, ScanHitExpandsIntoVerticalColumn)
{
  sensor_msgs::msg::LaserScan scan;
  scan.angle_min = 0.0F;
  scan.angle_increment = 0.1F;
  scan.range_min = 0.05F;
  scan.range_max = 5.0F;
  scan.ranges = {1.0F};

  collision_voxel_layer::ScanColumnParams params;
  params.min_range = 0.1;
  params.max_range = 2.0;
  params.z_min = 0.0;
  params.z_max = 0.3;
  params.voxel_size_z = 0.1;

  tf2::Transform identity;
  identity.setIdentity();
  const auto points = collision_voxel_layer::convert_scan_to_points(scan, identity, false, params);

  ASSERT_EQ(points.size(), 4u);
  EXPECT_NEAR(points.front().x(), 1.0, 1e-5);
  EXPECT_NEAR(points.front().y(), 0.0, 1e-5);
  EXPECT_NEAR(points.front().z(), 0.0, 1e-5);
  EXPECT_NEAR(points.back().z(), 0.3, 1e-5);
}

TEST(SourceAdapterTest, DepthFilterDropsInvalidAndOutOfBoundsPoints)
{
  sensor_msgs::msg::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(4);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  *iter_x = 0.5F; *iter_y = 0.0F; *iter_z = 0.3F;
  ++iter_x; ++iter_y; ++iter_z;
  *iter_x = 0.4F; *iter_y = 0.0F; *iter_z = -0.1F;
  ++iter_x; ++iter_y; ++iter_z;
  *iter_x = 6.0F; *iter_y = 0.0F; *iter_z = 0.4F;
  ++iter_x; ++iter_y; ++iter_z;
  *iter_x = std::numeric_limits<float>::quiet_NaN(); *iter_y = 0.0F; *iter_z = 0.2F;

  collision_voxel_layer::DepthFilterParams params;
  params.min_range = 0.1;
  params.max_range = 5.0;
  params.min_height = 0.0;
  params.max_height = 1.0;

  tf2::Transform identity;
  identity.setIdentity();
  const auto points = collision_voxel_layer::filter_depth_cloud(cloud, identity, false, params);

  ASSERT_EQ(points.size(), 1u);
  EXPECT_NEAR(points.front().x(), 0.5, 1e-5);
  EXPECT_NEAR(points.front().y(), 0.0, 1e-5);
  EXPECT_NEAR(points.front().z(), 0.3, 1e-5);
}
