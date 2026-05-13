#include "collision_voxel_layer/msg/voxel_grid.hpp"
#include "collision_voxel_layer/collision_voxel_layer_node.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

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

sensor_msgs::msg::LaserScan make_test_scan()
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "base_link";
  scan.header.stamp = builtin_interfaces::msg::Time{};
  scan.angle_min = 0.0F;
  scan.angle_increment = 1.0F;
  scan.range_min = 0.05F;
  scan.range_max = 8.0F;
  scan.ranges = {1.0F};
  return scan;
}

sensor_msgs::msg::LaserScan make_test_uss_scan()
{
  auto scan = make_test_scan();
  scan.ranges = {2.0F};
  return scan;
}

sensor_msgs::msg::PointCloud2 make_test_depth_cloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "base_link";
  cloud.header.stamp = builtin_interfaces::msg::Time{};

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(2);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  *iter_x = 1.02F;
  *iter_y = 0.01F;
  *iter_z = 0.24F;
  ++iter_x;
  ++iter_y;
  ++iter_z;
  *iter_x = 0.50F;
  *iter_y = 0.01F;
  *iter_z = 0.02F;
  return cloud;
}

sensor_msgs::msg::PointCloud2 make_stale_header_depth_cloud()
{
  auto cloud = make_test_depth_cloud();
  cloud.header.stamp.sec = 1;
  cloud.header.stamp.nanosec = 0;
  return cloud;
}

sensor_msgs::msg::PointCloud2 make_test_camera_depth_cloud()
{
  auto cloud = make_test_depth_cloud();
  cloud.header.frame_id = "camera_color_optical_frame";
  return cloud;
}

bool nearly_equal(float lhs, float rhs, float epsilon = 1e-3F)
{
  return std::fabs(lhs - rhs) <= epsilon;
}

class VoxelOutputHarness : public rclcpp::Node
{
public:
  VoxelOutputHarness()
  : Node("collision_voxel_layer_test_harness")
  {
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "/test/collision_voxel_layer/scan", rclcpp::SensorDataQoS());
    uss_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "/test/collision_voxel_layer/uss_laserscan", rclcpp::SensorDataQoS());
    depth_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/test/collision_voxel_layer/depth", rclcpp::SensorDataQoS());
    lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/test/collision_voxel_layer/lidar", rclcpp::SensorDataQoS());
    visualization_control_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/test/collision_voxel_layer/visualization_enabled",
      rclcpp::QoS(1).transient_local().reliable());
    points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/test/collision_voxel_layer/points",
      rclcpp::QoS(1).transient_local().reliable(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_points_ = *msg;
        ++points_count_;
      });
    grid_sub_ = this->create_subscription<collision_voxel_layer::msg::VoxelGrid>(
      "/test/collision_voxel_layer/grid",
      rclcpp::QoS(1).transient_local().reliable(),
      [this](const collision_voxel_layer::msg::VoxelGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_grid_ = *msg;
        ++grid_count_;
      });
    status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/test/collision_voxel_layer/source_status",
      rclcpp::QoS(1).transient_local().reliable(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_status_ = msg->data;
      });
  }

  bool wait_for_graph_ready(std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (
        scan_pub_->get_subscription_count() >= 1 &&
        depth_pub_->get_subscription_count() >= 1 &&
        grid_sub_->get_publisher_count() >= 1 &&
        status_sub_->get_publisher_count() >= 1)
      {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  bool wait_for_points_graph_ready(std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (
        scan_pub_->get_subscription_count() >= 1 &&
        depth_pub_->get_subscription_count() >= 1 &&
        lidar_pub_->get_subscription_count() >= 1 &&
        points_sub_->get_publisher_count() >= 1 &&
        status_sub_->get_publisher_count() >= 1)
      {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  bool wait_for_depth_points_graph_ready(std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (
        depth_pub_->get_subscription_count() >= 1 &&
        points_sub_->get_publisher_count() >= 1 &&
        status_sub_->get_publisher_count() >= 1)
      {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  bool wait_for_uss_scan_subscription(std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (uss_scan_pub_->get_subscription_count() >= 1) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  bool wait_for_visualization_control_subscription(std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (visualization_control_pub_->get_subscription_count() >= 1) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  void publish_inputs()
  {
    scan_pub_->publish(make_test_scan());
    depth_pub_->publish(make_test_depth_cloud());
  }

  void publish_scan()
  {
    scan_pub_->publish(make_test_scan());
  }

  void publish_uss_scan()
  {
    uss_scan_pub_->publish(make_test_uss_scan());
  }

  void publish_depth()
  {
    depth_pub_->publish(make_test_depth_cloud());
  }

  void publish_lidar()
  {
    lidar_pub_->publish(make_test_depth_cloud());
  }

  void publish_depth_with_stale_header()
  {
    depth_pub_->publish(make_stale_header_depth_cloud());
  }

  void publish_camera_depth()
  {
    depth_pub_->publish(make_test_camera_depth_cloud());
  }

  void set_visualization_enabled(bool enabled)
  {
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    visualization_control_pub_->publish(msg);
  }

  bool wait_for_grid_count_at_least(size_t expected, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (grid_count_ >= expected) {
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  bool wait_for_nonempty_grid(std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_grid_.has_value() && !latest_grid_->cells.empty()) {
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  bool wait_for_nonempty_points(std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_points_.has_value() && latest_points_->width * latest_points_->height > 0) {
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  collision_voxel_layer::msg::VoxelGrid latest_grid() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_grid_.value();
  }

  sensor_msgs::msg::PointCloud2 latest_points() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_points_.value();
  }

  bool wait_for_status_containing(
    const std::string & expected,
    std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (latest_status_.find(expected) != std::string::npos) {
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

private:
  mutable std::mutex mutex_;
  size_t grid_count_{0};
  size_t points_count_{0};
  std::optional<sensor_msgs::msg::PointCloud2> latest_points_;
  std::optional<collision_voxel_layer::msg::VoxelGrid> latest_grid_;
  std::string latest_status_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr uss_scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visualization_control_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<collision_voxel_layer::msg::VoxelGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
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
    source_timeout_s: 1.5
    source_health_check_period_s: 2.0
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

  const auto timeout_result = node->set_parameters_atomically({
    rclcpp::Parameter("source_timeout_s", 0.0)
  });
  EXPECT_FALSE(timeout_result.successful);

  const auto health_check_result = node->set_parameters_atomically({
    rclcpp::Parameter("source_health_check_period_s", 0.0)
  });
  EXPECT_FALSE(health_check_result.successful);

  const auto queue_result = node->set_parameters_atomically({
    rclcpp::Parameter("sync_queue_size", -1)
  });
  EXPECT_FALSE(queue_result.successful);

  const auto voxel_region_result = node->set_parameters_atomically({
    rclcpp::Parameter("voxel_region_xy", std::vector<double>{0.0, 0.0, 1.0, 0.0})
  });
  EXPECT_FALSE(voxel_region_result.successful);

  const auto base_height_result = node->set_parameters_atomically({
    rclcpp::Parameter("depth_min_base_height", 1.0),
    rclcpp::Parameter("depth_max_base_height", 0.5)
  });
  EXPECT_FALSE(base_height_result.successful);
}

TEST(CollisionVoxelLayerNodeTest, PublishesVoxelGridForSimulatedScanAndDepthInputs)
{
  RosContextGuard guard;
  auto harness = std::make_shared<VoxelOutputHarness>();

  auto node = std::make_shared<collision_voxel_layer::CollisionVoxelLayerNode>(
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("config_reload_enabled", false),
      rclcpp::Parameter("base_frame", "base_link"),
      rclcpp::Parameter("scan_topic", "/test/collision_voxel_layer/scan"),
      rclcpp::Parameter("depth_cloud_topic", "/test/collision_voxel_layer/depth"),
      rclcpp::Parameter("grid_topic", "/test/collision_voxel_layer/grid"),
      rclcpp::Parameter("markers_topic", "/test/collision_voxel_layer/markers"),
      rclcpp::Parameter("debug_cloud_topic", "/test/collision_voxel_layer/debug_cloud"),
      rclcpp::Parameter("source_status_topic", "/test/collision_voxel_layer/source_status"),
      rclcpp::Parameter("lidar_cloud_topic", ""),
      rclcpp::Parameter("publish_voxel_grid", true)
    }));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness);
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});
  struct ExecutorCleanup
  {
    rclcpp::executors::SingleThreadedExecutor & executor;
    std::thread & spin_thread;
    ~ExecutorCleanup()
    {
      executor.cancel();
      if (spin_thread.joinable()) {
        spin_thread.join();
      }
    }
  } cleanup{executor, spin_thread};

  ASSERT_TRUE(harness->wait_for_graph_ready(std::chrono::milliseconds(2000)));
  harness->publish_inputs();
  ASSERT_TRUE(harness->wait_for_nonempty_grid(std::chrono::milliseconds(2000)));

  const auto grid = harness->latest_grid();
  EXPECT_EQ(grid.header.frame_id, "base_link");
  ASSERT_EQ(grid.cells.size(), 2u);

  const auto depth_cell = std::find_if(
    grid.cells.begin(), grid.cells.end(),
    [](const auto & cell) {
      return nearly_equal(cell.x, 1.05F) &&
             nearly_equal(cell.y, 0.05F) &&
             nearly_equal(cell.z, 0.25F);
    });
  ASSERT_NE(depth_cell, grid.cells.end());
  EXPECT_TRUE(nearly_equal(depth_cell->occupancy, 0.8F));
  EXPECT_EQ(depth_cell->source_mask, 0x02U);

  const auto scan_only_cell = std::find_if(
    grid.cells.begin(), grid.cells.end(),
    [](const auto & cell) {
      return nearly_equal(cell.x, 1.05F) &&
             nearly_equal(cell.y, 0.05F) &&
             nearly_equal(cell.z, 0.05F);
  });
  ASSERT_NE(scan_only_cell, grid.cells.end());
  EXPECT_LE(scan_only_cell->occupancy, 0.6F);
  EXPECT_GT(scan_only_cell->occupancy, 0.5F);
  EXPECT_EQ(scan_only_cell->source_mask, 0x01U);
}

TEST(CollisionVoxelLayerNodeTest, PublishesSparsePointCloudFromScanDepthAndLidarInputs)
{
  RosContextGuard guard;
  auto harness = std::make_shared<VoxelOutputHarness>();

  auto node = std::make_shared<collision_voxel_layer::CollisionVoxelLayerNode>(
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("config_reload_enabled", false),
      rclcpp::Parameter("base_frame", "base_link"),
      rclcpp::Parameter("scan_topic", "/test/collision_voxel_layer/scan"),
      rclcpp::Parameter("depth_cloud_topic", "/test/collision_voxel_layer/depth"),
      rclcpp::Parameter("lidar_cloud_topic", "/test/collision_voxel_layer/lidar"),
      rclcpp::Parameter("points_topic", "/test/collision_voxel_layer/points"),
      rclcpp::Parameter("grid_topic", "/test/collision_voxel_layer/grid"),
      rclcpp::Parameter("source_status_topic", "/test/collision_voxel_layer/source_status"),
      rclcpp::Parameter("publish_points", true),
      rclcpp::Parameter("publish_voxel_grid", false)
    }));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness);
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});
  struct ExecutorCleanup
  {
    rclcpp::executors::SingleThreadedExecutor & executor;
    std::thread & spin_thread;
    ~ExecutorCleanup()
    {
      executor.cancel();
      if (spin_thread.joinable()) {
        spin_thread.join();
      }
    }
  } cleanup{executor, spin_thread};

  ASSERT_TRUE(harness->wait_for_points_graph_ready(std::chrono::milliseconds(2000)));
  harness->publish_scan();
  harness->publish_depth();
  harness->publish_lidar();
  ASSERT_TRUE(harness->wait_for_nonempty_points(std::chrono::milliseconds(2000)));

  const auto points = harness->latest_points();
  EXPECT_EQ(points.header.frame_id, "base_link");
  EXPECT_GE(points.width * points.height, 3u);
}

TEST(CollisionVoxelLayerNodeTest, DepthCloudUsesConfiguredExtrinsicWhenTfIsDisconnected)
{
  RosContextGuard guard;
  auto harness = std::make_shared<VoxelOutputHarness>();

  auto node = std::make_shared<collision_voxel_layer::CollisionVoxelLayerNode>(
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("config_reload_enabled", false),
      rclcpp::Parameter("base_frame", "base_link"),
      rclcpp::Parameter("scan_topic", ""),
      rclcpp::Parameter("depth_cloud_topic", "/test/collision_voxel_layer/depth"),
      rclcpp::Parameter("lidar_cloud_topic", ""),
      rclcpp::Parameter("points_topic", "/test/collision_voxel_layer/points"),
      rclcpp::Parameter("source_status_topic", "/test/collision_voxel_layer/source_status"),
      rclcpp::Parameter("publish_points", true),
      rclcpp::Parameter("publish_voxel_grid", false),
      rclcpp::Parameter("depth_source_frame", "camera_color_optical_frame"),
      rclcpp::Parameter("depth_use_extrinsic_fallback", true),
      rclcpp::Parameter("depth_extrinsic_xyz", std::vector<double>{0.363, -0.040, -0.284}),
      rclcpp::Parameter(
        "depth_extrinsic_qxyzw",
        std::vector<double>{0.0, 0.216440, 0.0, 0.976296}),
      rclcpp::Parameter("depth_min_height", -1.0),
      rclcpp::Parameter("depth_max_height", 2.0),
      rclcpp::Parameter("depth_min_base_height", -1.0),
      rclcpp::Parameter("depth_max_base_height", 2.0)
    }));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness);
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});
  struct ExecutorCleanup
  {
    rclcpp::executors::SingleThreadedExecutor & executor;
    std::thread & spin_thread;
    ~ExecutorCleanup()
    {
      executor.cancel();
      if (spin_thread.joinable()) {
        spin_thread.join();
      }
    }
  } cleanup{executor, spin_thread};

  ASSERT_TRUE(harness->wait_for_depth_points_graph_ready(std::chrono::milliseconds(2000)));
  harness->publish_camera_depth();
  ASSERT_TRUE(harness->wait_for_nonempty_points(std::chrono::milliseconds(2000)));

  const auto points = harness->latest_points();
  EXPECT_EQ(points.header.frame_id, "base_link");
  EXPECT_GE(points.width * points.height, 1u);
}

TEST(CollisionVoxelLayerNodeTest, PublishesVoxelGridWhenOnlyScanInputArrives)
{
  RosContextGuard guard;
  auto harness = std::make_shared<VoxelOutputHarness>();

  auto node = std::make_shared<collision_voxel_layer::CollisionVoxelLayerNode>(
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("config_reload_enabled", false),
      rclcpp::Parameter("base_frame", "base_link"),
      rclcpp::Parameter("scan_topic", "/test/collision_voxel_layer/scan"),
      rclcpp::Parameter("depth_cloud_topic", "/test/collision_voxel_layer/depth"),
      rclcpp::Parameter("grid_topic", "/test/collision_voxel_layer/grid"),
      rclcpp::Parameter("markers_topic", "/test/collision_voxel_layer/markers"),
      rclcpp::Parameter("debug_cloud_topic", "/test/collision_voxel_layer/debug_cloud"),
      rclcpp::Parameter("source_status_topic", "/test/collision_voxel_layer/source_status"),
      rclcpp::Parameter("lidar_cloud_topic", ""),
      rclcpp::Parameter("publish_voxel_grid", true),
      rclcpp::Parameter("source_timeout_s", 1.0)
    }));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness);
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});
  struct ExecutorCleanup
  {
    rclcpp::executors::SingleThreadedExecutor & executor;
    std::thread & spin_thread;
    ~ExecutorCleanup()
    {
      executor.cancel();
      if (spin_thread.joinable()) {
        spin_thread.join();
      }
    }
  } cleanup{executor, spin_thread};

  ASSERT_TRUE(harness->wait_for_graph_ready(std::chrono::milliseconds(2000)));
  harness->publish_scan();
  ASSERT_TRUE(harness->wait_for_nonempty_grid(std::chrono::milliseconds(2000)));
  ASSERT_TRUE(harness->wait_for_status_containing(
    "\"topic\":\"/test/collision_voxel_layer/depth\"", std::chrono::milliseconds(2000)));

  const auto grid = harness->latest_grid();
  EXPECT_EQ(grid.header.frame_id, "base_link");
  ASSERT_EQ(grid.cells.size(), 1u);

  for (const auto & cell : grid.cells) {
    EXPECT_TRUE(nearly_equal(cell.occupancy, 0.6F));
    EXPECT_EQ(cell.source_mask, 0x01U);
  }
}

TEST(CollisionVoxelLayerNodeTest, PublishesVoxelGridFromMultipleScanTopics)
{
  RosContextGuard guard;
  auto harness = std::make_shared<VoxelOutputHarness>();

  auto node = std::make_shared<collision_voxel_layer::CollisionVoxelLayerNode>(
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("config_reload_enabled", false),
      rclcpp::Parameter("base_frame", "base_link"),
      rclcpp::Parameter(
        "scan_topics",
        std::vector<std::string>{
          "/test/collision_voxel_layer/scan",
          "/test/collision_voxel_layer/uss_laserscan"}),
      rclcpp::Parameter("depth_cloud_topic", "/test/collision_voxel_layer/depth"),
      rclcpp::Parameter("grid_topic", "/test/collision_voxel_layer/grid"),
      rclcpp::Parameter("markers_topic", "/test/collision_voxel_layer/markers"),
      rclcpp::Parameter("debug_cloud_topic", "/test/collision_voxel_layer/debug_cloud"),
      rclcpp::Parameter("source_status_topic", "/test/collision_voxel_layer/source_status"),
      rclcpp::Parameter("lidar_cloud_topic", ""),
      rclcpp::Parameter("publish_voxel_grid", true),
      rclcpp::Parameter("source_timeout_s", 1.0)
    }));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness);
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});
  struct ExecutorCleanup
  {
    rclcpp::executors::SingleThreadedExecutor & executor;
    std::thread & spin_thread;
    ~ExecutorCleanup()
    {
      executor.cancel();
      if (spin_thread.joinable()) {
        spin_thread.join();
      }
    }
  } cleanup{executor, spin_thread};

  ASSERT_TRUE(harness->wait_for_graph_ready(std::chrono::milliseconds(2000)));
  ASSERT_TRUE(harness->wait_for_uss_scan_subscription(std::chrono::milliseconds(2000)));
  harness->publish_scan();
  ASSERT_TRUE(harness->wait_for_nonempty_grid(std::chrono::milliseconds(2000)));
  harness->publish_uss_scan();
  ASSERT_TRUE(harness->wait_for_grid_count_at_least(2, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(harness->wait_for_status_containing(
    "\"topic\":\"/test/collision_voxel_layer/uss_laserscan\"", std::chrono::milliseconds(2000)));

  const auto grid = harness->latest_grid();
  const auto scan_cell = std::find_if(
    grid.cells.begin(), grid.cells.end(),
    [](const auto & cell) {
      return nearly_equal(cell.x, 1.05F) &&
             nearly_equal(cell.y, 0.05F) &&
             nearly_equal(cell.z, 0.05F);
    });
  const auto uss_cell = std::find_if(
    grid.cells.begin(), grid.cells.end(),
    [](const auto & cell) {
      return nearly_equal(cell.x, 2.05F) &&
             nearly_equal(cell.y, 0.05F) &&
             nearly_equal(cell.z, 0.05F);
    });
  ASSERT_NE(scan_cell, grid.cells.end());
  ASSERT_NE(uss_cell, grid.cells.end());
  EXPECT_EQ(scan_cell->source_mask, 0x01U);
  EXPECT_EQ(uss_cell->source_mask, 0x01U);
}

TEST(CollisionVoxelLayerNodeTest, AppliesIndependentScanSourceThinning)
{
  RosContextGuard guard;
  auto harness = std::make_shared<VoxelOutputHarness>();

  auto node = std::make_shared<collision_voxel_layer::CollisionVoxelLayerNode>(
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("config_reload_enabled", false),
      rclcpp::Parameter("base_frame", "base_link"),
      rclcpp::Parameter(
        "scan_topics",
        std::vector<std::string>{
          "/test/collision_voxel_layer/scan",
          "/test/collision_voxel_layer/uss_laserscan"}),
      rclcpp::Parameter("scan_point_steps", std::vector<int64_t>{1, 8}),
      rclcpp::Parameter("scan_max_points", std::vector<int64_t>{0, 1}),
      rclcpp::Parameter("scan_voxel_prefilters", std::vector<double>{0.0, 0.0}),
      rclcpp::Parameter("depth_cloud_topic", ""),
      rclcpp::Parameter("grid_topic", "/test/collision_voxel_layer/grid"),
      rclcpp::Parameter("markers_topic", "/test/collision_voxel_layer/markers"),
      rclcpp::Parameter("debug_cloud_topic", "/test/collision_voxel_layer/debug_cloud"),
      rclcpp::Parameter("source_status_topic", "/test/collision_voxel_layer/source_status"),
      rclcpp::Parameter("lidar_cloud_topic", ""),
      rclcpp::Parameter("publish_voxel_grid", true),
      rclcpp::Parameter("source_timeout_s", 1.0)
    }));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness);
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});
  struct ExecutorCleanup
  {
    rclcpp::executors::SingleThreadedExecutor & executor;
    std::thread & spin_thread;
    ~ExecutorCleanup()
    {
      executor.cancel();
      if (spin_thread.joinable()) {
        spin_thread.join();
      }
    }
  } cleanup{executor, spin_thread};

  ASSERT_TRUE(harness->wait_for_graph_ready(std::chrono::milliseconds(2000)));
  ASSERT_TRUE(harness->wait_for_uss_scan_subscription(std::chrono::milliseconds(2000)));
  harness->publish_scan();
  ASSERT_TRUE(harness->wait_for_nonempty_grid(std::chrono::milliseconds(2000)));
  harness->publish_uss_scan();
  ASSERT_TRUE(harness->wait_for_grid_count_at_least(2, std::chrono::milliseconds(2000)));

  const auto grid = harness->latest_grid();
  const auto uss_cells = std::count_if(
    grid.cells.begin(), grid.cells.end(),
    [](const auto & cell) {
      return cell.x > 1.5F;
    });
  EXPECT_EQ(uss_cells, 1);
}

TEST(CollisionVoxelLayerNodeTest, VoxelRegionCullOutOfRangeScanPointsByXYOnly)
{
  RosContextGuard guard;
  auto harness = std::make_shared<VoxelOutputHarness>();

  auto node = std::make_shared<collision_voxel_layer::CollisionVoxelLayerNode>(
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("config_reload_enabled", false),
      rclcpp::Parameter("base_frame", "base_link"),
      rclcpp::Parameter(
        "scan_topics",
        std::vector<std::string>{
          "/test/collision_voxel_layer/scan",
          "/test/collision_voxel_layer/uss_laserscan"}),
      rclcpp::Parameter("depth_cloud_topic", "/test/collision_voxel_layer/depth"),
      rclcpp::Parameter("grid_topic", "/test/collision_voxel_layer/grid"),
      rclcpp::Parameter("markers_topic", "/test/collision_voxel_layer/markers"),
      rclcpp::Parameter("debug_cloud_topic", "/test/collision_voxel_layer/debug_cloud"),
      rclcpp::Parameter("source_status_topic", "/test/collision_voxel_layer/source_status"),
      rclcpp::Parameter("lidar_cloud_topic", ""),
      rclcpp::Parameter("publish_voxel_grid", true),
      rclcpp::Parameter(
        "voxel_region_xy",
        std::vector<double>{-1.0, -1.0, 1.5, -1.0, 1.5, 1.0, -1.0, 1.0})
    }));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness);
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});
  struct ExecutorCleanup
  {
    rclcpp::executors::SingleThreadedExecutor & executor;
    std::thread & spin_thread;
    ~ExecutorCleanup()
    {
      executor.cancel();
      if (spin_thread.joinable()) {
        spin_thread.join();
      }
    }
  } cleanup{executor, spin_thread};

  ASSERT_TRUE(harness->wait_for_graph_ready(std::chrono::milliseconds(2000)));
  ASSERT_TRUE(harness->wait_for_uss_scan_subscription(std::chrono::milliseconds(2000)));
  harness->publish_scan();
  ASSERT_TRUE(harness->wait_for_nonempty_grid(std::chrono::milliseconds(2000)));
  harness->publish_uss_scan();
  ASSERT_TRUE(harness->wait_for_grid_count_at_least(2, std::chrono::milliseconds(2000)));

  const auto grid = harness->latest_grid();
  EXPECT_FALSE(grid.cells.empty());
  const auto planar_scan_cell = std::find_if(
    grid.cells.begin(), grid.cells.end(),
    [](const auto & cell) {
      return nearly_equal(cell.x, 1.05F) && nearly_equal(cell.z, 0.05F);
    });
  EXPECT_NE(planar_scan_cell, grid.cells.end());
  const auto out_of_bounds_cell = std::find_if(
    grid.cells.begin(), grid.cells.end(),
    [](const auto & cell) {
      return cell.x > 1.5F;
    });
  EXPECT_EQ(out_of_bounds_cell, grid.cells.end());
}

TEST(CollisionVoxelLayerNodeTest, PublishesVoxelGridWhenOnlyDepthInputArrives)
{
  RosContextGuard guard;
  auto harness = std::make_shared<VoxelOutputHarness>();

  auto node = std::make_shared<collision_voxel_layer::CollisionVoxelLayerNode>(
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("config_reload_enabled", false),
      rclcpp::Parameter("base_frame", "base_link"),
      rclcpp::Parameter("scan_topic", "/test/collision_voxel_layer/scan"),
      rclcpp::Parameter("depth_cloud_topic", "/test/collision_voxel_layer/depth"),
      rclcpp::Parameter("grid_topic", "/test/collision_voxel_layer/grid"),
      rclcpp::Parameter("markers_topic", "/test/collision_voxel_layer/markers"),
      rclcpp::Parameter("debug_cloud_topic", "/test/collision_voxel_layer/debug_cloud"),
      rclcpp::Parameter("source_status_topic", "/test/collision_voxel_layer/source_status"),
      rclcpp::Parameter("lidar_cloud_topic", ""),
      rclcpp::Parameter("publish_voxel_grid", true),
      rclcpp::Parameter("source_timeout_s", 1.0)
    }));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness);
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});
  struct ExecutorCleanup
  {
    rclcpp::executors::SingleThreadedExecutor & executor;
    std::thread & spin_thread;
    ~ExecutorCleanup()
    {
      executor.cancel();
      if (spin_thread.joinable()) {
        spin_thread.join();
      }
    }
  } cleanup{executor, spin_thread};

  ASSERT_TRUE(harness->wait_for_graph_ready(std::chrono::milliseconds(2000)));
  harness->publish_depth();
  ASSERT_TRUE(harness->wait_for_nonempty_grid(std::chrono::milliseconds(2000)));

  const auto grid = harness->latest_grid();
  EXPECT_EQ(grid.header.frame_id, "base_link");
  ASSERT_EQ(grid.cells.size(), 1u);
  EXPECT_TRUE(nearly_equal(grid.cells.front().occupancy, 0.8F));
  EXPECT_EQ(grid.cells.front().source_mask, 0x02U);
}

TEST(CollisionVoxelLayerNodeTest, VisualizationCanBeToggledByTopic)
{
  RosContextGuard guard;
  auto harness = std::make_shared<VoxelOutputHarness>();

  auto node = std::make_shared<collision_voxel_layer::CollisionVoxelLayerNode>(
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("config_reload_enabled", false),
      rclcpp::Parameter("base_frame", "base_link"),
      rclcpp::Parameter("scan_topic", "/test/collision_voxel_layer/scan"),
      rclcpp::Parameter("depth_cloud_topic", "/test/collision_voxel_layer/depth"),
      rclcpp::Parameter("grid_topic", "/test/collision_voxel_layer/grid"),
      rclcpp::Parameter("markers_topic", "/test/collision_voxel_layer/markers"),
      rclcpp::Parameter("debug_cloud_topic", "/test/collision_voxel_layer/debug_cloud"),
      rclcpp::Parameter("source_status_topic", "/test/collision_voxel_layer/source_status"),
      rclcpp::Parameter("lidar_cloud_topic", ""),
      rclcpp::Parameter("publish_voxel_grid", true),
      rclcpp::Parameter(
        "visualization_control_topic",
        "/test/collision_voxel_layer/visualization_enabled"),
      rclcpp::Parameter("visualization_enabled", false),
      rclcpp::Parameter("publish_markers", true),
      rclcpp::Parameter("publish_debug_cloud", true)
    }));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness);
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});
  struct ExecutorCleanup
  {
    rclcpp::executors::SingleThreadedExecutor & executor;
    std::thread & spin_thread;
    ~ExecutorCleanup()
    {
      executor.cancel();
      if (spin_thread.joinable()) {
        spin_thread.join();
      }
    }
  } cleanup{executor, spin_thread};

  ASSERT_TRUE(harness->wait_for_graph_ready(std::chrono::milliseconds(2000)));
  ASSERT_TRUE(harness->wait_for_visualization_control_subscription(std::chrono::milliseconds(2000)));
  harness->set_visualization_enabled(true);
  ASSERT_TRUE(harness->wait_for_status_containing(
    "\"visualization\":{\"enabled\":true", std::chrono::milliseconds(2000)));
  harness->set_visualization_enabled(false);
  ASSERT_TRUE(harness->wait_for_status_containing(
    "\"visualization\":{\"enabled\":false", std::chrono::milliseconds(2000)));
}

TEST(CollisionVoxelLayerNodeTest, DepthSourceFreshnessUsesReceiveTimeNotHeaderStamp)
{
  RosContextGuard guard;
  auto harness = std::make_shared<VoxelOutputHarness>();

  auto node = std::make_shared<collision_voxel_layer::CollisionVoxelLayerNode>(
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("config_reload_enabled", false),
      rclcpp::Parameter("base_frame", "base_link"),
      rclcpp::Parameter("scan_topic", "/test/collision_voxel_layer/scan"),
      rclcpp::Parameter("depth_cloud_topic", "/test/collision_voxel_layer/depth"),
      rclcpp::Parameter("grid_topic", "/test/collision_voxel_layer/grid"),
      rclcpp::Parameter("markers_topic", "/test/collision_voxel_layer/markers"),
      rclcpp::Parameter("debug_cloud_topic", "/test/collision_voxel_layer/debug_cloud"),
      rclcpp::Parameter("source_status_topic", "/test/collision_voxel_layer/source_status"),
      rclcpp::Parameter("lidar_cloud_topic", ""),
      rclcpp::Parameter("publish_voxel_grid", true),
      rclcpp::Parameter("source_timeout_s", 1.0)
    }));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness);
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});
  struct ExecutorCleanup
  {
    rclcpp::executors::SingleThreadedExecutor & executor;
    std::thread & spin_thread;
    ~ExecutorCleanup()
    {
      executor.cancel();
      if (spin_thread.joinable()) {
        spin_thread.join();
      }
    }
  } cleanup{executor, spin_thread};

  ASSERT_TRUE(harness->wait_for_graph_ready(std::chrono::milliseconds(2000)));
  harness->publish_depth_with_stale_header();
  ASSERT_TRUE(harness->wait_for_nonempty_grid(std::chrono::milliseconds(2000)));
  ASSERT_TRUE(harness->wait_for_status_containing(
    "\"stale\":false", std::chrono::milliseconds(2000)));
}
