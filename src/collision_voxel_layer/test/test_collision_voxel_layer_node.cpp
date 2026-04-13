#include "collision_voxel_layer/collision_voxel_layer_node.hpp"

#include "gtest/gtest.h"

namespace
{

class RosContextGuard
{
public:
  RosContextGuard()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      owns_shutdown_ = true;
    }
  }

  ~RosContextGuard()
  {
    if (owns_shutdown_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

private:
  bool owns_shutdown_{false};
};

}  // namespace

TEST(CollisionVoxelLayerNodeTest, NodeConstructsWithDefaultParameters)
{
  RosContextGuard guard;
  auto node = std::make_shared<collision_voxel_layer::CollisionVoxelLayerNode>();

  EXPECT_STREQ(node->get_name(), "collision_voxel_layer");
}
