#include "collision_voxel_layer/msg/voxel_grid.hpp"

#include "gtest/gtest.h"

TEST(CollisionVoxelLayerMessageSmokeTest, VoxelGridMessageCompiles)
{
  collision_voxel_layer::msg::VoxelGrid msg;
  msg.resolution_xy = 0.05F;
  msg.resolution_z = 0.10F;
  msg.decay_time_s = 1.0F;

  EXPECT_FLOAT_EQ(msg.resolution_xy, 0.05F);
  EXPECT_FLOAT_EQ(msg.resolution_z, 0.10F);
  EXPECT_FLOAT_EQ(msg.decay_time_s, 1.0F);
}
