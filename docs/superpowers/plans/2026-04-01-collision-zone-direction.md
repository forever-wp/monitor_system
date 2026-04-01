# Collision Zone Direction Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `zone`-based collision slow/stop regions in `nav2_monitor` automatically filter front/rear zones by current motion direction without changing `approach / TTC` behavior.

**Architecture:** Extend the collision config schema with a global direction threshold and per-zone `motion_direction`, then apply direction filtering only inside the `zone` branch of `CollisionEvaluator`. Keep legacy configs working by defaulting unspecified directions to `both`, and cover the new behavior with focused gtests plus mirrored YAML/config updates.

**Tech Stack:** ROS 2 C++, YAML config parsing, gtest, Markdown docs

---

## File Map

- `src/nav2_monitor/include/nav2_monitor/fault_detector.hpp`
  Responsibility: extend collision config structs and add explicit zone motion direction enum/fields.
- `src/nav2_monitor/src/fault_detector.cpp`
  Responsibility: parse `direction_speed_threshold` and `zones[*].motion_direction`.
- `src/nav2_monitor/include/nav2_monitor/collision_evaluator.hpp`
  Responsibility: store runtime motion-direction state and helper declarations.
- `src/nav2_monitor/src/collision_evaluator.cpp`
  Responsibility: apply direction-aware filtering only for `zone` evaluation.
- `src/nav2_monitor/test/test_fault_detector.cpp`
  Responsibility: regression tests for forward/reverse/near-zero direction handling and compatibility.
- `config/Monitor/nav2_monitor/fault_detector_config.yaml`
  Responsibility: authoritative OTA default collision zone directions.
- `config/Monitor/nav2_monitor/profiles/fault_detector_reverse.yaml`
  Responsibility: authoritative OTA reverse profile zone directions.
- `config/Monitor/nav2_monitor/profiles/fault_detector_todoor.yaml`
  Responsibility: authoritative OTA to-door profile zone directions.
- `config/Monitor/nav2_monitor/profiles/fault_detector_elevator.yaml`
  Responsibility: authoritative OTA elevator profile zone directions.
- `src/nav2_monitor/config/fault_detector_config.yaml`
  Responsibility: package-local mirror of the default OTA collision config.
- `src/nav2_monitor/config/profiles/fault_detector_reverse.yaml`
  Responsibility: package-local mirror of reverse profile.
- `src/nav2_monitor/config/profiles/fault_detector_todoor.yaml`
  Responsibility: package-local mirror of to-door profile.
- `src/nav2_monitor/config/profiles/fault_detector_elevator.yaml`
  Responsibility: package-local mirror of elevator profile.
- `src/nav2_monitor/README.md`
  Responsibility: user-facing behavior and config docs.
- `src/nav2_monitor/docs/architecture.md`
  Responsibility: internal architecture notes for zone direction filtering.

## Task 1: Add failing tests for direction-aware zone filtering

**Files:**
- Modify: `src/nav2_monitor/test/test_fault_detector.cpp`

- [ ] **Step 1: Add a failing forward-only zone test**

```cpp
TEST_F(FaultDetectorTest, CollisionZoneIgnoresRearStopWhenMovingForward)
{
  // config defines rear_stop with motion_direction=reverse
  // feed forward prediction_motion and a point inside rear zone
  // expect no faults
}
```

- [ ] **Step 2: Add a failing reverse-only zone test**

```cpp
TEST_F(FaultDetectorTest, CollisionZoneIgnoresFrontStopWhenReversing)
{
  // config defines front_stop with motion_direction=forward
  // feed reverse prediction_motion and a point inside front zone
  // expect no faults
}
```

- [ ] **Step 3: Add a failing near-zero speed test for keeping last direction**

```cpp
TEST_F(FaultDetectorTest, CollisionZoneKeepsLastDirectionWhenSpeedNearZero)
{
  // first move forward to establish direction
  // then send near-zero motion and a rear point
  // expect forward filtering to remain active
}
```

- [ ] **Step 4: Run the focused gtest target and verify the new tests fail for the expected reason**

Run: `source /opt/ros/humble/setup.bash && source /home/tokou/claude/ry_work/install/setup.bash && source /home/tokou/claude/ry_work/.worktrees/monitor-ota-config/install/setup.bash && colcon test --packages-select nav2_monitor --event-handlers console_direct+`

Expected: FAIL because `zone` evaluation currently ignores motion direction.

## Task 2: Implement motion-direction-aware zone filtering

**Files:**
- Modify: `src/nav2_monitor/include/nav2_monitor/fault_detector.hpp`
- Modify: `src/nav2_monitor/src/fault_detector.cpp`
- Modify: `src/nav2_monitor/include/nav2_monitor/collision_evaluator.hpp`
- Modify: `src/nav2_monitor/src/collision_evaluator.cpp`

- [ ] **Step 1: Extend the config model with explicit direction fields**

Add:

```cpp
enum class CollisionMotionDirectionType
{
  BOTH,
  FORWARD,
  REVERSE
};
```

and fields:

```cpp
CollisionMotionDirectionType motion_direction{CollisionMotionDirectionType::BOTH};
double direction_speed_threshold{0.05};
```

- [ ] **Step 2: Parse the new YAML fields**

Support:

```yaml
direction_speed_threshold: 0.05
motion_direction: "forward"
```

with default `both` when omitted.

- [ ] **Step 3: Implement runtime direction tracking in `CollisionEvaluator`**

Add minimal state:

```cpp
enum class RuntimeMotionDirection { UNKNOWN, FORWARD, REVERSE };
mutable RuntimeMotionDirection last_motion_direction_{RuntimeMotionDirection::UNKNOWN};
```

Update on each evaluation using `prediction_linear_x`.

- [ ] **Step 4: Filter only `zone` model zones by direction**

Pseudo-code:

```cpp
if (zone.model == CollisionModelType::ZONE) {
  const auto direction = resolve_runtime_direction(chassis_state.prediction_linear_x, cfg.direction_speed_threshold);
  if (!zone_matches_direction(zone.motion_direction, direction)) {
    continue;
  }
}
```

Keep `approach / TTC` logic unchanged.

- [ ] **Step 5: Re-run nav2_monitor tests and verify they pass**

Run: `source /opt/ros/humble/setup.bash && source /home/tokou/claude/ry_work/install/setup.bash && source /home/tokou/claude/ry_work/.worktrees/monitor-ota-config/install/setup.bash && colcon build --packages-select nav2_monitor --cmake-clean-first --event-handlers console_direct+ && colcon test --packages-select nav2_monitor --event-handlers console_direct+`

Expected: PASS, including the new direction tests and all existing TTC tests.

## Task 3: Apply the new direction fields to OTA and mirrored configs

**Files:**
- Modify: `config/Monitor/nav2_monitor/fault_detector_config.yaml`
- Modify: `config/Monitor/nav2_monitor/profiles/fault_detector_reverse.yaml`
- Modify: `config/Monitor/nav2_monitor/profiles/fault_detector_todoor.yaml`
- Modify: `config/Monitor/nav2_monitor/profiles/fault_detector_elevator.yaml`
- Modify: `src/nav2_monitor/config/fault_detector_config.yaml`
- Modify: `src/nav2_monitor/config/profiles/fault_detector_reverse.yaml`
- Modify: `src/nav2_monitor/config/profiles/fault_detector_todoor.yaml`
- Modify: `src/nav2_monitor/config/profiles/fault_detector_elevator.yaml`

- [ ] **Step 1: Add `direction_speed_threshold` to collision_detection**

```yaml
direction_speed_threshold: 0.05
```

- [ ] **Step 2: Add `motion_direction` to zone polygons**

Examples:

```yaml
- name: "front_slow"
  motion_direction: "forward"
```

```yaml
- name: "rear_stop"
  motion_direction: "reverse"
```

- [ ] **Step 3: Mirror OTA config changes back into `src/nav2_monitor/config/`**

Use the same structure for the package-local copy to avoid drift between the two sources.

- [ ] **Step 4: Run the bridge/nav2/safety verification suite**

Run: `source /opt/ros/humble/setup.bash && source /home/tokou/claude/ry_work/install/setup.bash && source /home/tokou/claude/ry_work/.worktrees/monitor-ota-config/install/setup.bash && colcon test --packages-select bridge nav2_monitor safety_emergency_executor --event-handlers console_direct+`

Expected: PASS.

## Task 4: Update docs and operator guidance

**Files:**
- Modify: `src/nav2_monitor/README.md`
- Modify: `src/nav2_monitor/docs/architecture.md`

- [ ] **Step 1: Document the new fields and behavior**

Add notes for:

- `collision_detection.direction_speed_threshold`
- `zones[*].motion_direction`
- `zone` mode is direction-filtered
- `approach / TTC` remains independent

- [ ] **Step 2: Re-run a focused doc-sensitive verification**

Run: `source /opt/ros/humble/setup.bash && source /home/tokou/claude/ry_work/install/setup.bash && source /home/tokou/claude/ry_work/.worktrees/monitor-ota-config/install/setup.bash && colcon test --packages-select bridge nav2_monitor safety_emergency_executor --event-handlers console_direct+`

Expected: PASS with no lint regressions.

Plan complete and saved to `docs/superpowers/plans/2026-04-01-collision-zone-direction.md`. Ready to execute?
