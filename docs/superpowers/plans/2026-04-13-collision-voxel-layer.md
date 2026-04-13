# Collision Voxel Layer Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an independent `collision_voxel_layer` package that fuses `LaserScan + depth PointCloud2` into a decaying sparse voxel layer, then add a voxel-backed collision input path to `nav2_monitor`.

**Architecture:** Implement a new ROS 2 package responsible for synchronization, preprocessing, sparse voxel maintenance, decay, and voxel publishing. Keep `nav2_monitor` as the collision policy engine by adding a `voxel_topic` input path that consumes voxel occupancy for zone and TTC evaluation without depending on Nav2.

**Tech Stack:** C++17, ROS 2 Humble, `rclcpp`, ROS 2 official `message_filters`, `sensor_msgs`, `visualization_msgs`, `geometry_msgs`, `tf2_ros`, `tf2_sensor_msgs`, `rosidl_default_generators`, `ament_cmake_gtest`, YAML config files, git worktrees

---

## File Map

- Create: `src/collision_voxel_layer/CMakeLists.txt`
  - Build the new package, generate voxel messages, define node and tests.
- Create: `src/collision_voxel_layer/package.xml`
  - Declare runtime and build dependencies for messages, TF, and synchronization.
- Create: `src/collision_voxel_layer/msg/VoxelCell.msg`
  - Sparse voxel cell payload with occupancy and source provenance.
- Create: `src/collision_voxel_layer/msg/VoxelGrid.msg`
  - Sparse voxel grid output message for downstream consumers.
- Create: `src/collision_voxel_layer/include/collision_voxel_layer/voxel_types.hpp`
  - Internal key / state structs for sparse voxel storage.
- Create: `src/collision_voxel_layer/include/collision_voxel_layer/sparse_voxel_grid.hpp`
  - Sparse grid API for insertion, decay, pruning, and export.
- Create: `src/collision_voxel_layer/src/sparse_voxel_grid.cpp`
  - Sparse grid implementation.
- Create: `src/collision_voxel_layer/include/collision_voxel_layer/source_adapter.hpp`
  - Source preprocessing helpers for scan columns and depth cloud filtering.
- Create: `src/collision_voxel_layer/src/source_adapter.cpp`
  - Source adapter implementation.
- Create: `src/collision_voxel_layer/include/collision_voxel_layer/collision_voxel_layer_node.hpp`
  - Node declaration, subscriptions, synchronization wiring, timers, publishers.
- Create: `src/collision_voxel_layer/src/collision_voxel_layer_node.cpp`
  - Node implementation.
- Create: `src/collision_voxel_layer/src/main.cpp`
  - Executable entrypoint.
- Create: `src/collision_voxel_layer/config/collision_voxel_layer_params.yaml`
  - Default parameters for scan/depth sources and decay behavior.
- Create: `src/collision_voxel_layer/launch/collision_voxel_layer.launch.py`
  - Launch file for local validation.
- Create: `src/collision_voxel_layer/README.md`
  - Package usage and topic contract.
- Create: `src/collision_voxel_layer/test/test_sparse_voxel_grid.cpp`
  - Unit tests for insertion, decay, and pruning.
- Create: `src/collision_voxel_layer/test/test_source_adapter.cpp`
  - Unit tests for scan columnization and depth filtering.
- Modify: `src/nav2_monitor/CMakeLists.txt`
  - Add dependency on `collision_voxel_layer` messages.
- Modify: `src/nav2_monitor/package.xml`
  - Declare dependency on `collision_voxel_layer`.
- Modify: `src/nav2_monitor/include/nav2_monitor/fault_detector.hpp`
  - Extend collision config with voxel input fields.
- Modify: `src/nav2_monitor/src/fault_detector.cpp`
  - Parse voxel config fields.
- Modify: `src/nav2_monitor/include/nav2_monitor/monitor_data_store.hpp`
  - Add voxel runtime state and accessors.
- Modify: `src/nav2_monitor/src/monitor_data_store.cpp`
  - Store and expose voxel occupancy state.
- Modify: `src/nav2_monitor/include/nav2_monitor/nav2_monitor_node.hpp`
  - Add voxel subscription declaration.
- Modify: `src/nav2_monitor/src/nav2_monitor_node.cpp`
  - Subscribe to voxel grid topic and feed data store.
- Modify: `src/nav2_monitor/include/nav2_monitor/collision_evaluator.hpp`
  - Declare voxel-backed collision helpers.
- Modify: `src/nav2_monitor/src/collision_evaluator.cpp`
  - Consume voxel occupancy for zone and TTC evaluation.
- Modify: `src/nav2_monitor/test/test_fault_detector.cpp`
  - Add config parser and evaluator tests for voxel-backed collision input.
- Modify: `src/nav2_monitor/README.md`
  - Document new voxel input mode and topic examples.
- Modify: `src/nav2_monitor/config/fault_detector_config.yaml`
  - Add example voxel configuration.
- Modify: `src/nav2_monitor/config/profiles/fault_detector_elevator.yaml`
- Modify: `src/nav2_monitor/config/profiles/fault_detector_reverse.yaml`
- Modify: `src/nav2_monitor/config/profiles/fault_detector_todoor.yaml`
  - Keep profile mirrors aligned.
- Modify: `config/Monitor/nav2_monitor/fault_detector_config.yaml`
- Modify: `config/Monitor/nav2_monitor/profiles/fault_detector_elevator.yaml`
- Modify: `config/Monitor/nav2_monitor/profiles/fault_detector_reverse.yaml`
- Modify: `config/Monitor/nav2_monitor/profiles/fault_detector_todoor.yaml`
  - Sync OTA/runtime config bundle after repo-side validation.

## Chunk 1: Baseline Protection And Package Skeleton

### Task 0: Save The Current Worktree Baseline Before Implementation

**Files:**
- Modify: current tracked source/config files already changed in `/home/tokou/claude/ry_work/.worktrees/monitor-ota-config`
- Modify later: the mirrored files in `/home/tokou/claude/ry_work/.worktrees/control-source-routing-integration`

- [ ] **Step 1: Inspect the current monitor worktree state**

Run:

```bash
git -C /home/tokou/claude/ry_work/.worktrees/monitor-ota-config status --short
```

Expected:
- tracked changes under `src/nav2_monitor` and `config/Monitor/nav2_monitor`
- untracked build artifacts like `build/`, `install/`, `log/` remain excluded from commits

- [ ] **Step 2: Stage only the current engineering baseline**

Run:

```bash
git -C /home/tokou/claude/ry_work/.worktrees/monitor-ota-config add \
  config/Monitor/nav2_monitor/fault_detector_config.yaml \
  config/Monitor/nav2_monitor/profiles/fault_detector_elevator.yaml \
  config/Monitor/nav2_monitor/profiles/fault_detector_reverse.yaml \
  config/Monitor/nav2_monitor/profiles/fault_detector_todoor.yaml \
  src/nav2_monitor/CMakeLists.txt \
  src/nav2_monitor/README.md \
  src/nav2_monitor/config/fault_detector_config.yaml \
  src/nav2_monitor/config/profiles/fault_detector_elevator.yaml \
  src/nav2_monitor/config/profiles/fault_detector_reverse.yaml \
  src/nav2_monitor/config/profiles/fault_detector_todoor.yaml \
  src/nav2_monitor/include/nav2_monitor/fault_detector.hpp \
  src/nav2_monitor/include/nav2_monitor/nav2_monitor_node.hpp \
  src/nav2_monitor/src/collision_evaluator.cpp \
  src/nav2_monitor/src/fault_detector.cpp \
  src/nav2_monitor/src/nav2_monitor_node.cpp \
  src/nav2_monitor/test/test_fault_detector.cpp \
  src/nav2_monitor/include/nav2_monitor/collision_prediction_router.hpp \
  src/nav2_monitor/src/collision_prediction_router.cpp
```

Expected:
- no build artifacts or unrelated files staged

- [ ] **Step 3: Commit the baseline save point**

Run:

```bash
git -C /home/tokou/claude/ry_work/.worktrees/monitor-ota-config commit -m "chore: save pre-voxel-layer baseline"
```

Expected:
- one rollback anchor commit exists before any voxel-layer implementation work

- [ ] **Step 4: Record the baseline commit SHA in the implementation notes**

Append the resulting SHA to `docs/superpowers/plans/2026-04-13-collision-voxel-layer.md` under this task once created.

- [ ] **Step 5: Commit the updated plan note**

Run:

```bash
git -C /home/tokou/claude/ry_work/.worktrees/monitor-ota-config add docs/superpowers/plans/2026-04-13-collision-voxel-layer.md
git -C /home/tokou/claude/ry_work/.worktrees/monitor-ota-config commit -m "docs: record pre-voxel-layer baseline"
```

### Task 1: Scaffold The New Package And Message Contracts

**Files:**
- Create: `src/collision_voxel_layer/CMakeLists.txt`
- Create: `src/collision_voxel_layer/package.xml`
- Create: `src/collision_voxel_layer/msg/VoxelCell.msg`
- Create: `src/collision_voxel_layer/msg/VoxelGrid.msg`
- Create: `src/collision_voxel_layer/README.md`

- [ ] **Step 1: Write failing package-level message/build tests**

Create `src/collision_voxel_layer/test/test_sparse_voxel_grid.cpp` with a placeholder include that depends on generated voxel messages:

```cpp
#include "collision_voxel_layer/msg/voxel_grid.hpp"
#include "gtest/gtest.h"

TEST(CollisionVoxelLayerMessageSmokeTest, VoxelGridMessageCompiles)
{
  collision_voxel_layer::msg::VoxelGrid msg;
  msg.resolution_xy = 0.05F;
  EXPECT_FLOAT_EQ(msg.resolution_xy, 0.05F);
}
```

- [ ] **Step 2: Run build/test to verify the package is missing**

Run:

```bash
source /opt/ros/humble/setup.bash && colcon build --packages-select collision_voxel_layer
```

Expected:
- package not found or target missing

- [ ] **Step 3: Create the package skeleton**

Add:

`src/collision_voxel_layer/msg/VoxelCell.msg`

```text
float32 x
float32 y
float32 z
float32 occupancy
uint8 source_mask
```

`src/collision_voxel_layer/msg/VoxelGrid.msg`

```text
std_msgs/Header header
float32 resolution_xy
float32 resolution_z
float32 decay_time_s
collision_voxel_layer/VoxelCell[] cells
```

Set up `package.xml` and `CMakeLists.txt` with:

- `ament_cmake`
- `rosidl_default_generators`
- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`
- `visualization_msgs`
- `std_msgs`
- `builtin_interfaces`
- `tf2_ros`
- `tf2_sensor_msgs`
- `message_filters`

- [ ] **Step 4: Build the new package and run the placeholder test**

Run:

```bash
source /opt/ros/humble/setup.bash && colcon build --packages-select collision_voxel_layer
source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test --packages-select collision_voxel_layer --event-handlers console_direct+
```

Expected:
- package builds
- placeholder test passes

- [ ] **Step 5: Commit the scaffold**

```bash
git add src/collision_voxel_layer
git commit -m "feat(collision_voxel_layer): scaffold package and voxel messages"
```

## Chunk 2: Sparse Voxel Core And Sensor Adapters

### Task 2: Implement Sparse Voxel Storage With Decay

**Files:**
- Create: `src/collision_voxel_layer/include/collision_voxel_layer/voxel_types.hpp`
- Create: `src/collision_voxel_layer/include/collision_voxel_layer/sparse_voxel_grid.hpp`
- Create: `src/collision_voxel_layer/src/sparse_voxel_grid.cpp`
- Modify: `src/collision_voxel_layer/test/test_sparse_voxel_grid.cpp`

- [ ] **Step 1: Write failing decay / pruning tests**

Add tests:

```cpp
TEST(SparseVoxelGridTest, InsertsAndExportsOccupiedCells);
TEST(SparseVoxelGridTest, DecaysCellsExponentiallyOverTime);
TEST(SparseVoxelGridTest, PrunesCellsBelowThreshold);
```

Use a deterministic API like:

```cpp
SparseVoxelGrid grid(0.05, 0.10, 1.0, 0.01);
grid.insert_point(0.20, 0.10, 0.15, 0.6F, 0x01, rclcpp::Time(10, 0, RCL_ROS_TIME));
```

- [ ] **Step 2: Run the voxel grid test to verify failure**

Run:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && ./build/collision_voxel_layer/test_sparse_voxel_grid
```

Expected:
- compile or link failure because `SparseVoxelGrid` is not implemented yet

- [ ] **Step 3: Implement the minimal sparse voxel grid**

Define internal structs in `voxel_types.hpp`:

```cpp
struct VoxelKey
{
  int32_t ix;
  int32_t iy;
  int32_t iz;
  bool operator==(const VoxelKey & other) const = default;
};

struct VoxelState
{
  float occupancy{0.0F};
  uint8_t source_mask{0U};
  rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
};
```

Implement methods in `SparseVoxelGrid`:

- `insert_point(...)`
- `decay_to(const rclcpp::Time & now)`
- `export_grid(const std_msgs::msg::Header & header) const`

Use exponential decay:

```cpp
occupancy *= std::exp(-dt / decay_time_s_);
```

- [ ] **Step 4: Re-run the voxel grid unit test and make it pass**

Run the same direct gtest binary or `colcon test` command.

Expected:
- all new sparse-grid tests pass

- [ ] **Step 5: Commit the sparse voxel core**

```bash
git add src/collision_voxel_layer/include/collision_voxel_layer/voxel_types.hpp src/collision_voxel_layer/include/collision_voxel_layer/sparse_voxel_grid.hpp src/collision_voxel_layer/src/sparse_voxel_grid.cpp src/collision_voxel_layer/test/test_sparse_voxel_grid.cpp
git commit -m "feat(collision_voxel_layer): add sparse voxel grid core"
```

### Task 3: Implement Scan Columnization And Depth Filtering

**Files:**
- Create: `src/collision_voxel_layer/include/collision_voxel_layer/source_adapter.hpp`
- Create: `src/collision_voxel_layer/src/source_adapter.cpp`
- Create: `src/collision_voxel_layer/test/test_source_adapter.cpp`

- [ ] **Step 1: Write failing source adapter tests**

Add tests covering:

- `LaserScan` hit expands into vertical voxel candidates between `scan_z_min` and `scan_z_max`
- depth points outside `depth_min_height / depth_max_height` are rejected
- invalid depth points are dropped

Example expectation:

```cpp
EXPECT_GT(scan_points.size(), 1u);  // vertical column, not single slice
EXPECT_TRUE(std::ranges::all_of(scan_points, [](const auto & p) { return p.x > 0.0; }));
```

- [ ] **Step 2: Run the source adapter test to verify failure**

Run:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test --packages-select collision_voxel_layer --event-handlers console_direct+ --ctest-args -R test_source_adapter
```

Expected:
- missing adapter helpers or failing assertions

- [ ] **Step 3: Implement source preprocessing helpers**

Create adapter APIs similar to:

```cpp
std::vector<tf2::Vector3> expand_scan_hit_column(double x, double y, double z_min, double z_max, double voxel_size_z);
std::vector<tf2::Vector3> convert_scan_to_points(...);
std::vector<tf2::Vector3> filter_depth_cloud(...);
```

Rules:

- scan points use configured min/max range
- scan hits expand vertically
- depth points use trusted range and height limits

- [ ] **Step 4: Re-run the adapter tests and make them pass**

Run the same focused `colcon test` command.

Expected:
- `test_source_adapter` passes

- [ ] **Step 5: Commit the adapter layer**

```bash
git add src/collision_voxel_layer/include/collision_voxel_layer/source_adapter.hpp src/collision_voxel_layer/src/source_adapter.cpp src/collision_voxel_layer/test/test_source_adapter.cpp
git commit -m "feat(collision_voxel_layer): add scan and depth adapters"
```

## Chunk 3: Node, Synchronization, And Voxel Publishing

### Task 4: Add The Runtime Node And Debug Outputs

**Files:**
- Create: `src/collision_voxel_layer/include/collision_voxel_layer/collision_voxel_layer_node.hpp`
- Create: `src/collision_voxel_layer/src/collision_voxel_layer_node.cpp`
- Create: `src/collision_voxel_layer/src/main.cpp`
- Create: `src/collision_voxel_layer/config/collision_voxel_layer_params.yaml`
- Create: `src/collision_voxel_layer/launch/collision_voxel_layer.launch.py`
- Modify: `src/collision_voxel_layer/CMakeLists.txt`
- Modify: `src/collision_voxel_layer/package.xml`

- [ ] **Step 1: Write a failing smoke test or compile target expectation**

Add a basic node smoke test or at minimum a constructor test that instantiates the node with parameters.

If a lightweight constructor test is simpler:

```cpp
TEST(CollisionVoxelLayerNodeTest, NodeConstructsWithDefaultParameters);
```

- [ ] **Step 2: Run build/test to verify the runtime node is missing**

Run:

```bash
source /opt/ros/humble/setup.bash && colcon build --packages-select collision_voxel_layer
```

Expected:
- node target missing or constructor test failing

- [ ] **Step 3: Implement the node**

Wire:

- `message_filters::Subscriber<sensor_msgs::msg::LaserScan>`
- `message_filters::Subscriber<sensor_msgs::msg::PointCloud2>`
- `message_filters::Synchronizer<ApproximateTime<...>>`
- periodic decay timer
- publishers:
  - `/collision_voxel_layer/grid`
  - `/collision_voxel_layer/markers`
  - `/collision_voxel_layer/debug_cloud`

Minimal publish path:

```cpp
auto grid_msg = sparse_grid_.export_grid(header);
voxel_pub_->publish(grid_msg);
```

- [ ] **Step 4: Build and run package tests**

Run:

```bash
source /opt/ros/humble/setup.bash && colcon build --packages-select collision_voxel_layer
source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test --packages-select collision_voxel_layer --event-handlers console_direct+
```

Expected:
- package builds
- all `collision_voxel_layer` tests pass

- [ ] **Step 5: Commit the runtime node**

```bash
git add src/collision_voxel_layer/CMakeLists.txt src/collision_voxel_layer/package.xml src/collision_voxel_layer/include/collision_voxel_layer/collision_voxel_layer_node.hpp src/collision_voxel_layer/src/collision_voxel_layer_node.cpp src/collision_voxel_layer/src/main.cpp src/collision_voxel_layer/config/collision_voxel_layer_params.yaml src/collision_voxel_layer/launch/collision_voxel_layer.launch.py
git commit -m "feat(collision_voxel_layer): add voxel layer runtime node"
```

## Chunk 4: `nav2_monitor` Voxel Consumer Path

### Task 5: Parse Voxel Config And Store Voxel State

**Files:**
- Modify: `src/nav2_monitor/CMakeLists.txt`
- Modify: `src/nav2_monitor/package.xml`
- Modify: `src/nav2_monitor/include/nav2_monitor/fault_detector.hpp`
- Modify: `src/nav2_monitor/src/fault_detector.cpp`
- Modify: `src/nav2_monitor/include/nav2_monitor/monitor_data_store.hpp`
- Modify: `src/nav2_monitor/src/monitor_data_store.cpp`
- Modify: `src/nav2_monitor/include/nav2_monitor/nav2_monitor_node.hpp`
- Modify: `src/nav2_monitor/src/nav2_monitor_node.cpp`
- Modify: `src/nav2_monitor/test/test_fault_detector.cpp`

- [ ] **Step 1: Write failing parser and store tests**

Add parser test examples:

```cpp
TEST_F(FaultDetectorTest, CollisionDetectionParsesVoxelTopic);
TEST_F(FaultDetectorTest, CollisionDetectionVoxelStoreFiltersByOccupancy);
```

Expected config fields:

```yaml
collision_detection:
  voxel_topic: "/collision_voxel_layer/grid"
  voxel_min_occupancy: 0.35
```

- [ ] **Step 2: Run focused `nav2_monitor` tests to verify failure**

Run:

```bash
source /opt/ros/humble/setup.bash && colcon build --packages-select collision_voxel_layer nav2_monitor
source /opt/ros/humble/setup.bash && source install/setup.bash && ./build/nav2_monitor/test_fault_detector --gtest_filter='*Voxel*'
```

Expected:
- parser/store tests fail because voxel fields do not exist

- [ ] **Step 3: Add config parsing and runtime storage**

Add config fields in `fault_detector.hpp`:

```cpp
std::string voxel_topic;
double voxel_min_occupancy{0.0};
double voxel_min_height{-std::numeric_limits<double>::infinity()};
double voxel_max_height{std::numeric_limits<double>::infinity()};
```

Add `VoxelGrid` runtime storage in `MonitorDataStore` and a subscriber in `nav2_monitor_node.cpp`.

- [ ] **Step 4: Re-run focused `nav2_monitor` voxel tests and make them pass**

Run the same focused test command.

Expected:
- parser and storage tests pass

- [ ] **Step 5: Commit the config/storage changes**

```bash
git add src/nav2_monitor/CMakeLists.txt src/nav2_monitor/package.xml src/nav2_monitor/include/nav2_monitor/fault_detector.hpp src/nav2_monitor/src/fault_detector.cpp src/nav2_monitor/include/nav2_monitor/monitor_data_store.hpp src/nav2_monitor/src/monitor_data_store.cpp src/nav2_monitor/include/nav2_monitor/nav2_monitor_node.hpp src/nav2_monitor/src/nav2_monitor_node.cpp src/nav2_monitor/test/test_fault_detector.cpp
git commit -m "feat(nav2_monitor): add voxel collision input path"
```

### Task 6: Use Voxel Occupancy In Zone And TTC Evaluation

**Files:**
- Modify: `src/nav2_monitor/include/nav2_monitor/collision_evaluator.hpp`
- Modify: `src/nav2_monitor/src/collision_evaluator.cpp`
- Modify: `src/nav2_monitor/test/test_fault_detector.cpp`

- [ ] **Step 1: Write failing evaluator tests for voxel-backed zones and TTC**

Add tests:

```cpp
TEST_F(FaultDetectorTest, CollisionZoneUsesVoxelOccupancyWeight);
TEST_F(FaultDetectorTest, CollisionTtcUsesVoxelCellCentersAsCandidates);
TEST_F(FaultDetectorTest, CollisionTtcIgnoresLowOccupancyVoxelCells);
```

Use synthetic voxel cells rather than raw scan points.

- [ ] **Step 2: Run focused evaluator tests and verify failure**

Run:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && ./build/nav2_monitor/test_fault_detector --gtest_filter='*Collision*Voxel*:*CollisionTtcUsesVoxelCellCentersAsCandidates*'
```

Expected:
- no voxel path exists yet

- [ ] **Step 3: Implement voxel-backed collision candidate extraction**

Add helper path in `collision_evaluator.cpp`:

- convert voxel cells to weighted `CollisionPoint`
- filter by occupancy and height
- prefer voxel evidence when `voxel_topic` is configured

Minimal weighting rule:

```cpp
point.weight = voxel_cell.occupancy;
```

Then reuse existing zone and TTC evaluators with those weighted points.

- [ ] **Step 4: Re-run focused evaluator tests, then run the full `nav2_monitor` test suite**

Run:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && ./build/nav2_monitor/test_fault_detector --gtest_filter='*Voxel*'
source /opt/ros/humble/setup.bash && export ROS_LOG_DIR=/tmp/ros_logs && colcon test --packages-select nav2_monitor --event-handlers console_direct+
```

Expected:
- voxel-focused tests pass
- existing collision tests remain green aside from any known pre-existing unrelated reds

- [ ] **Step 5: Commit voxel evaluator support**

```bash
git add src/nav2_monitor/include/nav2_monitor/collision_evaluator.hpp src/nav2_monitor/src/collision_evaluator.cpp src/nav2_monitor/test/test_fault_detector.cpp
git commit -m "feat(nav2_monitor): evaluate collisions from voxel occupancy"
```

## Chunk 5: Config, Docs, Verification, And Worktree Sync

### Task 7: Document And Wire Default Configs

**Files:**
- Modify: `src/nav2_monitor/README.md`
- Modify: `src/nav2_monitor/config/fault_detector_config.yaml`
- Modify: `src/nav2_monitor/config/profiles/fault_detector_elevator.yaml`
- Modify: `src/nav2_monitor/config/profiles/fault_detector_reverse.yaml`
- Modify: `src/nav2_monitor/config/profiles/fault_detector_todoor.yaml`
- Modify: `config/Monitor/nav2_monitor/fault_detector_config.yaml`
- Modify: `config/Monitor/nav2_monitor/profiles/fault_detector_elevator.yaml`
- Modify: `config/Monitor/nav2_monitor/profiles/fault_detector_reverse.yaml`
- Modify: `config/Monitor/nav2_monitor/profiles/fault_detector_todoor.yaml`
- Modify: `src/collision_voxel_layer/README.md`

- [ ] **Step 1: Update configs with voxel examples**

Add fields like:

```yaml
collision_detection:
  voxel_topic: "/collision_voxel_layer/grid"
  voxel_min_occupancy: 0.35
```

For first repo defaults, keep raw source topics present but commented with migration guidance if needed.

- [ ] **Step 2: Update READMEs**

Document:

- package topics and parameters
- `LaserScan + depth` input model
- voxel decay behavior
- `nav2_monitor` voxel consumption path
- migration rule: use one authoritative obstacle source path to avoid double counting

- [ ] **Step 3: Verify config mirrors stay aligned**

Run:

```bash
diff -u src/nav2_monitor/config/fault_detector_config.yaml config/Monitor/nav2_monitor/fault_detector_config.yaml
diff -u src/nav2_monitor/config/profiles/fault_detector_elevator.yaml config/Monitor/nav2_monitor/profiles/fault_detector_elevator.yaml
diff -u src/nav2_monitor/config/profiles/fault_detector_reverse.yaml config/Monitor/nav2_monitor/profiles/fault_detector_reverse.yaml
diff -u src/nav2_monitor/config/profiles/fault_detector_todoor.yaml config/Monitor/nav2_monitor/profiles/fault_detector_todoor.yaml
```

Expected:
- no diffs for mirrored config content

- [ ] **Step 4: Commit docs and config wiring**

```bash
git add src/nav2_monitor/README.md src/nav2_monitor/config/fault_detector_config.yaml src/nav2_monitor/config/profiles/fault_detector_elevator.yaml src/nav2_monitor/config/profiles/fault_detector_reverse.yaml src/nav2_monitor/config/profiles/fault_detector_todoor.yaml config/Monitor/nav2_monitor/fault_detector_config.yaml config/Monitor/nav2_monitor/profiles/fault_detector_elevator.yaml config/Monitor/nav2_monitor/profiles/fault_detector_reverse.yaml config/Monitor/nav2_monitor/profiles/fault_detector_todoor.yaml src/collision_voxel_layer/README.md
git commit -m "docs: wire voxel collision configuration"
```

### Task 8: Final Verification And Sync To The Control Worktree

**Files:**
- Modify: mirrored implementation files in `/home/tokou/claude/ry_work/.worktrees/control-source-routing-integration`

- [ ] **Step 1: Run final package verification in the monitor worktree**

Run:

```bash
source /opt/ros/humble/setup.bash && colcon build --packages-select collision_voxel_layer nav2_monitor
source /opt/ros/humble/setup.bash && export ROS_LOG_DIR=/tmp/ros_logs && colcon test --packages-select collision_voxel_layer nav2_monitor --event-handlers console_direct+
```

Expected:
- both packages build
- new tests pass
- note any pre-existing unrelated test failures explicitly

- [ ] **Step 2: Sync the same source changes into the control worktree**

Mirror:

- `src/collision_voxel_layer/**`
- `src/nav2_monitor/**`

Do not copy build artifacts or unrelated files like `INTERFACES.md`.

- [ ] **Step 3: Build and test the mirrored code in the control worktree**

Run:

```bash
source /opt/ros/humble/setup.bash && colcon build --packages-select collision_voxel_layer nav2_monitor
source /opt/ros/humble/setup.bash && export ROS_LOG_DIR=/tmp/ros_logs && colcon test --packages-select collision_voxel_layer nav2_monitor --event-handlers console_direct+
```

Expected:
- mirrored code builds
- the same voxel-related tests pass

- [ ] **Step 4: Commit the mirrored control worktree changes**

```bash
git -C /home/tokou/claude/ry_work/.worktrees/control-source-routing-integration add src/collision_voxel_layer src/nav2_monitor
git -C /home/tokou/claude/ry_work/.worktrees/control-source-routing-integration commit -m "feat: add collision voxel layer integration"
```

- [ ] **Step 5: Record verification results and residual risks**

Document in the final handoff:

- baseline save-point SHA
- package build/test commands executed
- known unrelated red tests, if any
- rollout warning that voxel and raw source paths should not be enabled together in production unless explicitly supported
