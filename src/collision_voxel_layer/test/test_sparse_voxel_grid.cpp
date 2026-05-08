#include "collision_voxel_layer/sparse_voxel_grid.hpp"

#include <cmath>

#include "gtest/gtest.h"

TEST(SparseVoxelGridTest, InsertsAndExportsOccupiedCells)
{
  collision_voxel_layer::SparseVoxelGrid grid(0.05, 0.10, 1.0, 0.01, 1.0F);
  const rclcpp::Time stamp(10, 0, RCL_ROS_TIME);
  grid.insert_point(0.20, 0.10, 0.15, 0.6F, 0x01U, stamp);

  std_msgs::msg::Header header;
  header.stamp = builtin_interfaces::msg::Time{};
  header.frame_id = "base_link";
  const auto msg = grid.export_grid(header);

  ASSERT_EQ(msg.cells.size(), 1u);
  EXPECT_FLOAT_EQ(msg.resolution_xy, 0.05F);
  EXPECT_FLOAT_EQ(msg.resolution_z, 0.10F);
  EXPECT_FLOAT_EQ(msg.decay_time_s, 1.0F);
  EXPECT_NEAR(msg.cells.front().x, 0.225F, 1e-5);
  EXPECT_NEAR(msg.cells.front().y, 0.125F, 1e-5);
  EXPECT_NEAR(msg.cells.front().z, 0.150F, 1e-5);
  EXPECT_FLOAT_EQ(msg.cells.front().occupancy, 0.6F);
  EXPECT_EQ(msg.cells.front().source_mask, 0x01U);
}

TEST(SparseVoxelGridTest, DecaysCellsExponentiallyOverTime)
{
  collision_voxel_layer::SparseVoxelGrid grid(0.05, 0.10, 1.0, 0.01, 1.0F);
  const rclcpp::Time stamp(10, 0, RCL_ROS_TIME);
  grid.insert_point(0.20, 0.10, 0.15, 1.0F, 0x01U, stamp);

  grid.decay_to(stamp + rclcpp::Duration::from_seconds(1.0));

  std_msgs::msg::Header header;
  header.frame_id = "base_link";
  const auto msg = grid.export_grid(header);

  ASSERT_EQ(msg.cells.size(), 1u);
  EXPECT_NEAR(msg.cells.front().occupancy, std::exp(-1.0), 1e-4);
}

TEST(SparseVoxelGridTest, PrunesCellsBelowThreshold)
{
  collision_voxel_layer::SparseVoxelGrid grid(0.05, 0.10, 1.0, 0.4, 1.0F);
  const rclcpp::Time stamp(10, 0, RCL_ROS_TIME);
  grid.insert_point(0.20, 0.10, 0.15, 1.0F, 0x01U, stamp);

  grid.decay_to(stamp + rclcpp::Duration::from_seconds(2.0));

  std_msgs::msg::Header header;
  header.frame_id = "base_link";
  const auto msg = grid.export_grid(header);

  EXPECT_TRUE(msg.cells.empty());
  EXPECT_EQ(grid.size(), 0u);
}

TEST(SparseVoxelGridTest, ClearingSourceRemovesOnlyThatSourceContribution)
{
  collision_voxel_layer::SparseVoxelGrid grid(0.05, 0.10, 1.0, 0.01, 1.0F);
  const rclcpp::Time stamp(10, 0, RCL_ROS_TIME);
  grid.insert_point(0.20, 0.10, 0.15, 0.6F, 0x01U, stamp);
  grid.insert_point(0.20, 0.10, 0.15, 0.8F, 0x02U, stamp);

  grid.clear_source(0x01U, stamp + rclcpp::Duration::from_seconds(0.1));

  std_msgs::msg::Header header;
  header.frame_id = "base_link";
  const auto msg = grid.export_grid(header);

  ASSERT_EQ(msg.cells.size(), 1u);
  EXPECT_NEAR(msg.cells.front().occupancy, 0.8F * std::exp(-0.1), 1e-4);
  EXPECT_EQ(msg.cells.front().source_mask, 0x02U);
}

TEST(SparseVoxelGridTest, ReplacingSourceFrameRemovesOldSourceTrail)
{
  collision_voxel_layer::SparseVoxelGrid grid(0.05, 0.10, 1.0, 0.01, 1.0F);
  const rclcpp::Time first_stamp(10, 0, RCL_ROS_TIME);
  const auto second_stamp = first_stamp + rclcpp::Duration::from_seconds(0.1);
  grid.insert_point(0.20, 0.10, 0.15, 0.6F, 0x01U, first_stamp);

  grid.clear_source(0x01U, second_stamp);
  grid.insert_point(0.40, 0.10, 0.15, 0.6F, 0x01U, second_stamp);

  std_msgs::msg::Header header;
  header.frame_id = "base_link";
  const auto msg = grid.export_grid(header);

  ASSERT_EQ(msg.cells.size(), 1u);
  EXPECT_NEAR(msg.cells.front().x, 0.425F, 1e-5);
  EXPECT_FLOAT_EQ(msg.cells.front().occupancy, 0.6F);
  EXPECT_EQ(msg.cells.front().source_mask, 0x01U);
}
