#include "collision_voxel_layer/collision_voxel_layer_node.hpp"

#include <fstream>
#include <string>

#include "gtest/gtest.h"

namespace
{

std::string write_temp_config(const std::string & content, const std::string & suffix)
{
  const std::string path = "/tmp/collision_voxel_layer_" + suffix + ".yaml";
  std::ofstream out(path);
  out << content;
  return path;
}

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

TEST(CollisionVoxelLayerNodeTest, ReloadsParametersFromConfigFile)
{
  RosContextGuard guard;
  const std::string config_path = write_temp_config(R"(
collision_voxel_layer:
  ros__parameters:
    publish_rate: 4.0
    scan_weight: 0.25
    depth_weight: 0.55
    sync_queue_size: 7
)", "reload");

  auto node = std::make_shared<collision_voxel_layer::CollisionVoxelLayerNode>(
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("config_file", config_path),
      rclcpp::Parameter("config_reload_enabled", false)
    }));

  EXPECT_TRUE(node->reload_config_if_needed(true));
  EXPECT_EQ(node->resolved_config_file(), config_path);
  EXPECT_DOUBLE_EQ(node->publish_rate_hz(), 4.0);
  EXPECT_DOUBLE_EQ(node->scan_weight(), 0.25);
  EXPECT_DOUBLE_EQ(node->depth_weight(), 0.55);
  EXPECT_EQ(node->sync_queue_size(), 7u);
}

TEST(CollisionVoxelLayerNodeTest, AppliesRuntimeParameterUpdates)
{
  RosContextGuard guard;
  auto node = std::make_shared<collision_voxel_layer::CollisionVoxelLayerNode>();

  const auto result = node->set_parameters_atomically({
    rclcpp::Parameter("publish_rate", 6.0),
    rclcpp::Parameter("scan_weight", 0.9),
    rclcpp::Parameter("config_reload_period_s", 2.5)
  });

  ASSERT_TRUE(result.successful) << result.reason;
  EXPECT_DOUBLE_EQ(node->publish_rate_hz(), 6.0);
  EXPECT_DOUBLE_EQ(node->scan_weight(), 0.9);
  EXPECT_DOUBLE_EQ(node->config_reload_period_s(), 2.5);
}

TEST(CollisionVoxelLayerNodeTest, RejectsInvalidRuntimeParameterUpdates)
{
  RosContextGuard guard;
  auto node = std::make_shared<collision_voxel_layer::CollisionVoxelLayerNode>();

  const auto result = node->set_parameters_atomically({
    rclcpp::Parameter("publish_rate", 0.0)
  });

  EXPECT_FALSE(result.successful);
}
