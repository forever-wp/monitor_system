# TTC Visualization Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add RViz2 TTC visualization for `approach / TTC` collision detection with a single config switch.

**Architecture:** Keep the existing TTC logic unchanged and only add optional `MarkerArray` publishing around the already computed trajectory/clearance/TTC result. Default the feature off and publish on a dedicated topic so existing polygon visualization stays untouched.

**Tech Stack:** ROS 2 C++, `visualization_msgs/msg/MarkerArray`, YAML config, gtest, Markdown docs

---

## File Map

- `src/nav2_monitor/include/nav2_monitor/fault_detector.hpp`
  Responsibility: add TTC visualization config flag to collision detection config.
- `src/nav2_monitor/src/fault_detector.cpp`
  Responsibility: parse `ttc_visualization_enabled`.
- `src/nav2_monitor/include/nav2_monitor/collision_evaluator.hpp`
  Responsibility: expose minimal TTC visualization snapshot data structures.
- `src/nav2_monitor/src/collision_evaluator.cpp`
  Responsibility: capture the latest TTC trajectory/debug state when enabled.
- `src/nav2_monitor/include/nav2_monitor/nav2_monitor_node.hpp`
  Responsibility: declare `MarkerArray` publisher and visualization helpers.
- `src/nav2_monitor/src/nav2_monitor_node.cpp`
  Responsibility: publish `/nav2_monitor/collision_ttc_markers` only when enabled.
- `src/nav2_monitor/test/test_fault_detector.cpp`
  Responsibility: regression tests for the new config parsing and non-regression behavior.
- `config/Monitor/nav2_monitor/fault_detector_config.yaml`
  Responsibility: authoritative OTA default for `ttc_visualization_enabled`.
- `config/Monitor/nav2_monitor/profiles/*.yaml`
  Responsibility: profile copies of the TTC visualization switch.
- `src/nav2_monitor/config/*.yaml`
  Responsibility: mirrored package-local copies of the TTC visualization switch.
- `src/nav2_monitor/README.md`
  Responsibility: user-facing topic/config documentation.
- `src/nav2_monitor/docs/architecture.md`
  Responsibility: architecture notes for TTC markers.

## Task 1: Add failing tests for TTC visualization config support

**Files:**
- Modify: `src/nav2_monitor/test/test_fault_detector.cpp`

- [ ] **Step 1: Add a failing config parse test for `ttc_visualization_enabled`**

```cpp
TEST_F(FaultDetectorTest, CollisionDetectionParsesTtcVisualizationEnabled)
{
  // load config with ttc_visualization_enabled: 1
  // expect getter/config field to reflect true
}
```

- [ ] **Step 2: Add a failing regression test that disabled remains default**

```cpp
TEST_F(FaultDetectorTest, CollisionDetectionTtcVisualizationDefaultsToDisabled)
{
  // load config without the field
  // expect false
}
```

- [ ] **Step 3: Run nav2_monitor tests and verify the new parse test fails**

Run: `source /opt/ros/humble/setup.bash && source /home/tokou/claude/ry_work/install/setup.bash && source /home/tokou/claude/ry_work/.worktrees/monitor-ota-config/install/setup.bash && colcon build --packages-select nav2_monitor --cmake-clean-first --event-handlers console_direct+ && colcon test --packages-select nav2_monitor --event-handlers console_direct+`

Expected: FAIL because the config struct/parser does not yet know `ttc_visualization_enabled`.

## Task 2: Implement the TTC visualization switch and data plumbing

**Files:**
- Modify: `src/nav2_monitor/include/nav2_monitor/fault_detector.hpp`
- Modify: `src/nav2_monitor/src/fault_detector.cpp`
- Modify: `src/nav2_monitor/include/nav2_monitor/collision_evaluator.hpp`
- Modify: `src/nav2_monitor/src/collision_evaluator.cpp`

- [ ] **Step 1: Add the config flag**

```cpp
bool ttc_visualization_enabled{false};
```

- [ ] **Step 2: Parse the YAML field**

```yaml
ttc_visualization_enabled: 1
```

- [ ] **Step 3: Add a minimal TTC debug snapshot in `CollisionEvaluator`**

Capture only what is already computed:

- active zone name
- trajectory points
- footprint poses
- nearest collision point
- current TTC

- [ ] **Step 4: Re-run nav2_monitor tests to verify the config tests pass**

Run: `source /opt/ros/humble/setup.bash && source /home/tokou/claude/ry_work/install/setup.bash && source /home/tokou/claude/ry_work/.worktrees/monitor-ota-config/install/setup.bash && colcon build --packages-select nav2_monitor --cmake-clean-first --event-handlers console_direct+ && colcon test --packages-select nav2_monitor --event-handlers console_direct+`

Expected: PASS, with TTC behavior unchanged.

## Task 3: Publish `MarkerArray` in nav2_monitor

**Files:**
- Modify: `src/nav2_monitor/include/nav2_monitor/nav2_monitor_node.hpp`
- Modify: `src/nav2_monitor/src/nav2_monitor_node.cpp`

- [ ] **Step 1: Add a dedicated publisher**

Topic:

```text
/nav2_monitor/collision_ttc_markers
```

- [ ] **Step 2: Publish only when `ttc_visualization_enabled` is true**

Markers:

- trajectory `LINE_STRIP`
- footprint `LINE_LIST` or `LINE_STRIP`
- nearest point `SPHERE`
- TTC text `TEXT_VIEW_FACING`

- [ ] **Step 3: Clear markers when no TTC candidate is active**

- [ ] **Step 4: Run the package tests again**

Run: `source /opt/ros/humble/setup.bash && source /home/tokou/claude/ry_work/install/setup.bash && source /home/tokou/claude/ry_work/.worktrees/monitor-ota-config/install/setup.bash && colcon build --packages-select nav2_monitor --cmake-clean-first --event-handlers console_direct+ && colcon test --packages-select nav2_monitor --event-handlers console_direct+`

Expected: PASS.

## Task 4: Wire the config files and docs

**Files:**
- Modify: `config/Monitor/nav2_monitor/fault_detector_config.yaml`
- Modify: `config/Monitor/nav2_monitor/profiles/fault_detector_todoor.yaml`
- Modify: `config/Monitor/nav2_monitor/profiles/fault_detector_elevator.yaml`
- Modify: `config/Monitor/nav2_monitor/profiles/fault_detector_reverse.yaml`
- Modify: `src/nav2_monitor/config/fault_detector_config.yaml`
- Modify: `src/nav2_monitor/config/profiles/fault_detector_todoor.yaml`
- Modify: `src/nav2_monitor/config/profiles/fault_detector_elevator.yaml`
- Modify: `src/nav2_monitor/config/profiles/fault_detector_reverse.yaml`
- Modify: `src/nav2_monitor/README.md`
- Modify: `src/nav2_monitor/docs/architecture.md`

- [ ] **Step 1: Add `ttc_visualization_enabled: 0` to the collision configs**

- [ ] **Step 2: Mirror `config/Monitor` changes back into `src/nav2_monitor/config/`**

- [ ] **Step 3: Document the marker topic and single switch**

- [ ] **Step 4: Run the full verification suite**

Run: `source /opt/ros/humble/setup.bash && source /home/tokou/claude/ry_work/install/setup.bash && source /home/tokou/claude/ry_work/.worktrees/monitor-ota-config/install/setup.bash && colcon test --packages-select bridge nav2_monitor safety_emergency_executor --event-handlers console_direct+`

Expected: PASS.

Plan complete and saved to `docs/superpowers/plans/2026-04-01-ttc-visualization.md`. Ready to execute?
