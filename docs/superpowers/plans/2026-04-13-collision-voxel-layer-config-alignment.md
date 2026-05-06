# Collision Voxel Layer Config Alignment Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Align `collision_voxel_layer` with the Monitor runtime module conventions: unified config storage, ROS 2 parameter updates, and file-based hot reload.

**Architecture:** Keep `collision_voxel_layer` as an independent node, but add the same operational shape already used by Monitor modules: `/opt/ry` runtime config entry, config file watcher, and live parameter callback. `nav2_monitor` continues to consume only the exported voxel topic.

**Tech Stack:** ROS 2 Humble, `rclcpp`, `message_filters`, filesystem mtime polling, ROS parameter YAML parsing, CMake/gtest

---

## Chunk 1: Runtime Config Entry

### Task 1: Add the monitor runtime config bundle

**Files:**
- Create: `config/Monitor/collision_voxel_layer/collision_voxel_layer_params.yaml`
- Modify: `src/collision_voxel_layer/README.md`
- Test: `src/bridge/test/test_monitor_ota_layout.py`

- [ ] **Step 1: Write the failing test**
Add OTA layout assertions for the new runtime config bundle and launch path.

- [ ] **Step 2: Run test to verify it fails**
Run: `source /opt/ros/humble/setup.bash && colcon test --packages-select bridge --event-handlers console_direct+`
Expected: failing assertions because the new config bundle is not present yet.

- [ ] **Step 3: Write minimal implementation**
Create the runtime config file and update docs/launch references.

- [ ] **Step 4: Run test to verify it passes**
Run the same bridge layout test command and confirm the new assertions pass.

- [ ] **Step 5: Commit**
Commit message: `feat(collision_voxel_layer): add monitor runtime config bundle`

## Chunk 2: Hot Reloadable Node

### Task 2: Add runtime parameter reload support to the node

**Files:**
- Modify: `src/collision_voxel_layer/include/collision_voxel_layer/collision_voxel_layer_node.hpp`
- Modify: `src/collision_voxel_layer/src/collision_voxel_layer_node.cpp`
- Modify: `src/collision_voxel_layer/CMakeLists.txt`
- Modify: `src/collision_voxel_layer/package.xml`
- Test: `src/collision_voxel_layer/test/test_collision_voxel_layer_node.cpp`

- [ ] **Step 1: Write the failing test**
Add node tests for file reload and parameter callback driven reconfiguration.

- [ ] **Step 2: Run test to verify it fails**
Run: `source /opt/ros/humble/setup.bash && colcon test --packages-select collision_voxel_layer --event-handlers console_direct+`
Expected: new reload tests fail because the node cannot hot reload yet.

- [ ] **Step 3: Write minimal implementation**
Add config path resolution, file watcher polling, parameter callback validation, and in-place reconfiguration.

- [ ] **Step 4: Run test to verify it passes**
Run the same package test command and confirm all node tests pass.

- [ ] **Step 5: Commit**
Commit message: `feat(collision_voxel_layer): add live config reload`

## Chunk 3: Worktree Sync And Verification

### Task 3: Sync the control worktree and verify both sides

**Files:**
- Modify: `src/collision_voxel_layer/launch/collision_voxel_layer.launch.py`
- Modify: `src/collision_voxel_layer/config/collision_voxel_layer_params.yaml`
- Modify: `config/Monitor/collision_voxel_layer/collision_voxel_layer_params.yaml`
- Test: both worktrees

- [ ] **Step 1: Sync the control worktree code**
Copy the monitor implementation into `control-source-routing-integration`, preserving only the config entry path differences.

- [ ] **Step 2: Run targeted verification in the monitor worktree**
Run: `source /opt/ros/humble/setup.bash && colcon test --packages-select collision_voxel_layer nav2_monitor bridge --event-handlers console_direct+`
Expected: all targeted suites pass.

- [ ] **Step 3: Run targeted verification in the control worktree**
Run: `source /opt/ros/humble/setup.bash && colcon test --packages-select collision_voxel_layer nav2_monitor --event-handlers console_direct+`
Expected: all targeted suites pass.

- [ ] **Step 4: Commit**
Commit message: `feat(collision_voxel_layer): align runtime config across worktrees`
