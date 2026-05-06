# Dynamic TTC Zone Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace static `approach` polygons with lightweight dynamic TTC zones that directly trigger safety actions while preserving near-field hard zones and the current TTC preview behavior.

**Architecture:** Extend the collision config schema with a canonical `model: "ttc"` rule and runtime parameters for horizon, corridor margin, and candidate downsampling. Reuse the existing footprint-based trajectory TTC math, but replace the outer candidate stage with a dynamic corridor built from sampled prediction motion. Treat the current uncommitted TTC preview fixes in this worktree as the baseline; do not revert them while implementing the dynamic TTC migration.

**Tech Stack:** C++17, ROS 2 Humble, `rclcpp`, `yaml-cpp`, `visualization_msgs`, `geometry_msgs`, `ament_cmake_gtest`, `pytest`

---

## File Map

- Modify: `src/nav2_monitor/include/nav2_monitor/fault_detector.hpp`
  - Add the canonical TTC model enum and per-zone TTC runtime fields.
  - Extend TTC visualization state with dynamic corridor geometry.
- Modify: `src/nav2_monitor/src/fault_detector.cpp`
  - Parse `model: "ttc"` and alias legacy `model: "approach"` to TTC with a warning.
  - Parse `ttc_horizon_s`, `corridor_margin`, and `candidate_downsample_resolution`.
- Modify: `src/nav2_monitor/include/nav2_monitor/collision_evaluator.hpp`
  - Declare helpers for bounding-radius computation, centerline sampling, corridor filtering, candidate downsampling, and visualization capture.
- Modify: `src/nav2_monitor/src/collision_evaluator.cpp`
  - Replace static `approach.points` gating with dynamic TTC corridor filtering.
  - Preserve the current “moving without hit still shows trajectory” visualization behavior.
- Modify: `src/nav2_monitor/include/nav2_monitor/nav2_monitor_node.hpp`
  - Add any new marker helper declarations needed for corridor rendering.
- Modify: `src/nav2_monitor/src/nav2_monitor_node.cpp`
  - Publish TTC corridor markers.
  - Stop treating TTC rules as static polygon publishers.
  - Keep zero-stamp marker behavior for RViz TF stability.
- Modify: `src/nav2_monitor/test/test_fault_detector.cpp`
  - Add parser and evaluator tests for the new TTC model.
  - Rename TTC-related expectations from `front_approach/rear_approach` to `front_ttc/rear_ttc`.
- Modify: `src/bridge/test/test_monitor_ota_layout.py`
  - Assert the OTA config bundle now uses TTC rule names and dynamic TTC fields.
- Modify: `config/Monitor/nav2_monitor/fault_detector_config.yaml`
  - Migrate repo OTA source config from `approach` to `ttc`.
- Modify: `config/Monitor/nav2_monitor/profiles/fault_detector_reverse.yaml`
- Modify: `config/Monitor/nav2_monitor/profiles/fault_detector_elevator.yaml`
- Modify: `config/Monitor/nav2_monitor/profiles/fault_detector_todoor.yaml`
  - Mirror the TTC migration across all profiles.
- Modify: `src/nav2_monitor/config/fault_detector_config.yaml`
- Modify: `src/nav2_monitor/config/profiles/fault_detector_reverse.yaml`
- Modify: `src/nav2_monitor/config/profiles/fault_detector_elevator.yaml`
- Modify: `src/nav2_monitor/config/profiles/fault_detector_todoor.yaml`
  - Keep the package-local mirror synchronized with `config/Monitor`.
- Modify: `src/nav2_monitor/README.md`
- Modify: `src/nav2_monitor/docs/architecture.md`
  - Update docs to describe dynamic TTC zones, near-field hard zones, and RViz corridor visualization.

## Chunk 1: Schema And Parsing

### Task 1: Protect The Current Baseline And Add TTC Parsing Coverage

**Files:**
- Modify: `src/nav2_monitor/test/test_fault_detector.cpp`
- Modify: `src/bridge/test/test_monitor_ota_layout.py`
- Modify: `src/nav2_monitor/include/nav2_monitor/fault_detector.hpp`
- Modify: `src/nav2_monitor/src/fault_detector.cpp`

- [ ] **Step 1: Write failing tests for the canonical TTC schema**

Add focused parser tests in `src/nav2_monitor/test/test_fault_detector.cpp`:

```cpp
TEST_F(FaultDetectorTest, CollisionDetectionParsesDynamicTtcModel)
{
  const std::string config_text = R"(
collision_detection:
  enabled: 1
  footprint_points: [-0.3, -0.2, -0.3, 0.2, 0.3, 0.2, 0.3, -0.2]
  zones:
    - name: "front_ttc"
      model: "ttc"
      motion_direction: "forward"
      time_before_collision: 3.0
      ttc_horizon_s: 3.5
      corridor_margin: 0.10
      candidate_downsample_resolution: 0.08
modules:
  - name: "dummy"
    supervisor: 0
    safety_system: 0
)";
  // Expect cfg.zones[0].model == CollisionModelType::TTC and fields parsed.
}
```

Add a static config test in `src/bridge/test/test_monitor_ota_layout.py`:

```python
def test_dynamic_ttc_defaults_exist_in_monitor_bundle():
    config = repo_root / "config/Monitor/nav2_monitor/fault_detector_config.yaml"
    data = yaml.safe_load(config.read_text())
    ttc_zone = next(zone for zone in data["collision_detection"]["zones"] if zone["name"] == "front_ttc")
    assert ttc_zone["model"] == "ttc"
    assert "ttc_horizon_s" in ttc_zone
    assert "corridor_margin" in ttc_zone
    assert "candidate_downsample_resolution" in ttc_zone
```

- [ ] **Step 2: Run the targeted tests to verify they fail**

Run:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_LOG_DIR=/tmp/ros_logs && colcon test --packages-select nav2_monitor --event-handlers console_direct+ --ctest-args -R test_fault_detector
```

Expected:
- `CollisionDetectionParsesDynamicTtcModel` fails because `model: "ttc"` is not parsed yet.

Run:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_LOG_DIR=/tmp/ros_logs && colcon test --packages-select bridge --event-handlers console_direct+ --ctest-args -R test_monitor_ota_layout
```

Expected:
- the new bundle assertion fails because configs still use `front_approach/rear_approach`.

- [ ] **Step 3: Implement minimal schema support**

Update `src/nav2_monitor/include/nav2_monitor/fault_detector.hpp`:

```cpp
enum class CollisionModelType
{
  ZONE = 0,
  TTC = 1
};

struct CollisionZoneConfig
{
  // ...
  double ttc_horizon_s{0.0};
  double corridor_margin{0.10};
  double candidate_downsample_resolution{0.08};
};
```

Update `src/nav2_monitor/src/fault_detector.cpp`:

```cpp
if (zone_node["model"]) {
  const auto model = to_lower(zone_node["model"].as<std::string>());
  if (model == "ttc") {
    zone.model = CollisionModelType::TTC;
  } else if (model == "approach") {
    zone.model = CollisionModelType::TTC;
    RCLCPP_WARN(node_->get_logger(), "[collision_detection][%s] model=approach is deprecated; use model=ttc", zone.name.c_str());
  }
}

if (zone_node["ttc_horizon_s"]) {
  zone.ttc_horizon_s = std::max(0.0, zone_node["ttc_horizon_s"].as<double>());
}
if (zone_node["corridor_margin"]) {
  zone.corridor_margin = std::max(0.0, zone_node["corridor_margin"].as<double>());
}
if (zone_node["candidate_downsample_resolution"]) {
  zone.candidate_downsample_resolution = std::max(0.01, zone_node["candidate_downsample_resolution"].as<double>());
}
```

- [ ] **Step 4: Re-run the targeted tests and make them pass**

Run the same `nav2_monitor` and `bridge` test commands.

Expected:
- parser test passes
- bundle test may still fail on config names until config migration task; keep only the parser assertion enabled in this step if needed

- [ ] **Step 5: Commit the schema-only change**

```bash
git add src/nav2_monitor/include/nav2_monitor/fault_detector.hpp src/nav2_monitor/src/fault_detector.cpp src/nav2_monitor/test/test_fault_detector.cpp
git commit -m "feat(nav2_monitor): parse dynamic ttc config"
```

## Chunk 2: Dynamic TTC Evaluation

### Task 2: Replace Static Approach Gating With Dynamic Corridor Filtering

**Files:**
- Modify: `src/nav2_monitor/include/nav2_monitor/collision_evaluator.hpp`
- Modify: `src/nav2_monitor/src/collision_evaluator.cpp`
- Modify: `src/nav2_monitor/test/test_fault_detector.cpp`

- [ ] **Step 1: Write failing evaluator tests for dynamic TTC behavior**

Add tests in `src/nav2_monitor/test/test_fault_detector.cpp` that no longer provide `points` on the TTC zone:

```cpp
TEST_F(FaultDetectorTest, CollisionDetectionTtcTriggersWithoutStaticPolygon)
{
  const std::string config_text = R"(
multi_value_judge:
  trigger_count: 1
  recover_count: 1
collision_detection:
  enabled: 1
  source_timeout_s: 1.0
  footprint_points: [-0.37, -0.28, -0.37, 0.28, 0.37, 0.28, 0.37, -0.28]
  zones:
    - name: "front_ttc"
      model: "ttc"
      motion_direction: "forward"
      time_before_collision: 1.0
      ttc_horizon_s: 1.5
      corridor_margin: 0.10
      candidate_downsample_resolution: 0.08
      actions: ["safety_system"]
modules:
  - name: "dummy"
    supervisor: 0
    safety_system: 0
)";
  // Set prediction motion forward and a collision point near the forward trajectory.
  // Expect one fault with zone=front_ttc.
}
```

Add a negative test:

```cpp
TEST_F(FaultDetectorTest, CollisionDetectionTtcSkipsWhenFootprintMissing)
{
  // model=ttc with no footprint_points
  // Expect no faults and no crash.
}
```

Add a direction test:

```cpp
TEST_F(FaultDetectorTest, CollisionDetectionRearTtcOnlyTriggersWhenReversing)
{
  // rear_ttc should ignore forward motion and trigger on reverse motion.
}
```

- [ ] **Step 2: Run the focused gtest suite and verify failure**

Run:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_LOG_DIR=/tmp/ros_logs && colcon test --packages-select nav2_monitor --event-handlers console_direct+ --ctest-args -R test_fault_detector
```

Expected:
- TTC tests fail because evaluator still requires `zone.points` and `CollisionModelType::APPROACH`.

- [ ] **Step 3: Implement the dynamic corridor helpers and TTC path**

Extend `src/nav2_monitor/include/nav2_monitor/collision_evaluator.hpp` with helper declarations:

```cpp
static double compute_bounding_radius(const std::vector<CollisionPoint> & footprint);
static std::vector<CollisionPoint> sample_centerline(
  double linear_x, double linear_y, double angular_z, double horizon_s, double dt);
static std::vector<CollisionPoint> collect_ttc_candidate_points(
  const std::vector<CollisionPoint> & points,
  const std::vector<CollisionPoint> & centerline,
  double corridor_radius);
static std::vector<CollisionPoint> downsample_candidate_points(
  const std::vector<CollisionPoint> & points,
  double resolution);
```

Implement in `src/nav2_monitor/src/collision_evaluator.cpp`:

```cpp
const bool is_ttc_zone = zone.model == CollisionModelType::TTC;
if (is_ttc_zone && current_speed > 1e-3) {
  const double active_horizon = std::max({
    zone.ttc_horizon_s,
    zone.time_before_collision,
    zone.recover_time_before_collision,
  });
  const auto centerline = sample_centerline(
    chassis_state.prediction_linear_x,
    chassis_state.prediction_linear_y,
    chassis_state.prediction_angular_z,
    active_horizon,
    zone.simulation_time_step);
  const double corridor_radius =
    compute_bounding_radius(cfg.footprint_points) + zone.corridor_margin;
  const auto corridor_candidates = collect_ttc_candidate_points(points, centerline, corridor_radius);
  const auto ttc_candidates = downsample_candidate_points(
    corridor_candidates, zone.candidate_downsample_resolution);
  // Run estimate_trajectory_collision_time() on ttc_candidates.
}
```

Rules:
- No static `zone.points` requirement for TTC.
- If `cfg.footprint_points.empty()`, skip the TTC rule and log once per evaluation.
- Keep `zone_matches_motion_direction()` for TTC rules too.
- Preserve the current uncommitted “moving without hit still shows trajectory” preview behavior.

- [ ] **Step 4: Re-run the focused gtests and make them pass**

Run the same `nav2_monitor` test command.

Expected:
- TTC-without-polygon test passes
- missing-footprint test passes
- reverse TTC direction test passes

- [ ] **Step 5: Commit the evaluator migration**

```bash
git add src/nav2_monitor/include/nav2_monitor/collision_evaluator.hpp src/nav2_monitor/src/collision_evaluator.cpp src/nav2_monitor/test/test_fault_detector.cpp
git commit -m "feat(nav2_monitor): replace approach with dynamic ttc corridor"
```

### Task 3: Extend TTC Visualization To Show Dynamic Corridor

**Files:**
- Modify: `src/nav2_monitor/include/nav2_monitor/fault_detector.hpp`
- Modify: `src/nav2_monitor/include/nav2_monitor/nav2_monitor_node.hpp`
- Modify: `src/nav2_monitor/src/nav2_monitor_node.cpp`
- Modify: `src/nav2_monitor/src/collision_evaluator.cpp`
- Modify: `src/nav2_monitor/test/test_fault_detector.cpp`

- [ ] **Step 1: Write failing tests for dynamic TTC visualization data**

Add a new visualization assertion:

```cpp
TEST_F(FaultDetectorTest, CollisionDetectionTtcVisualizationCapturesDynamicCorridor)
{
  // model=ttc, moving forward, no collision hit
  // Expect vis.active == true and vis.corridor_outline not empty.
}
```

Update the existing snapshot test to expect the new TTC zone name:

```cpp
EXPECT_EQ(vis.zone_name, "front_ttc");
```

- [ ] **Step 2: Run the nav2_monitor test target and verify failure**

Run:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_LOG_DIR=/tmp/ros_logs && colcon test --packages-select nav2_monitor --event-handlers console_direct+ --ctest-args -R test_fault_detector
```

Expected:
- corridor visualization assertions fail because no corridor geometry is stored yet.

- [ ] **Step 3: Implement the visualization state and marker updates**

Extend `CollisionTtcVisualizationState` in `src/nav2_monitor/include/nav2_monitor/fault_detector.hpp`:

```cpp
std::vector<CollisionPoint> corridor_outline;
```

Populate it in `src/nav2_monitor/src/collision_evaluator.cpp` from the sampled centerline and corridor radius.

Update `src/nav2_monitor/src/nav2_monitor_node.cpp`:

```cpp
visualization_msgs::msg::Marker corridor_marker;
corridor_marker.ns = "collision_ttc_corridor";
corridor_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
// emit the corridor outline using zero stamp, like the existing markers
```

Also update `configure_collision_monitoring()` and `publish_collision_zones()` so TTC rules are not treated as static polygon publishers:

```cpp
if (zone.model == CollisionModelType::TTC) {
  continue;
}
```

- [ ] **Step 4: Rebuild and re-run the nav2_monitor tests**

Run:

```bash
source /opt/ros/humble/setup.bash && export ROS_LOG_DIR=/tmp/ros_logs && colcon build --packages-up-to nav2_monitor --cmake-clean-first --event-handlers console_direct+
```

Expected:
- build succeeds

Run:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_LOG_DIR=/tmp/ros_logs && colcon test --packages-select nav2_monitor --event-handlers console_direct+ --ctest-args -R test_fault_detector
```

Expected:
- visualization tests pass

- [ ] **Step 5: Commit the visualization update**

```bash
git add src/nav2_monitor/include/nav2_monitor/fault_detector.hpp src/nav2_monitor/include/nav2_monitor/nav2_monitor_node.hpp src/nav2_monitor/src/nav2_monitor_node.cpp src/nav2_monitor/src/collision_evaluator.cpp src/nav2_monitor/test/test_fault_detector.cpp
git commit -m "feat(nav2_monitor): visualize dynamic ttc corridor"
```

## Chunk 3: Configs, Docs, And Verification

### Task 4: Migrate Repo Configs And Mirrors From Approach To TTC

**Files:**
- Modify: `config/Monitor/nav2_monitor/fault_detector_config.yaml`
- Modify: `config/Monitor/nav2_monitor/profiles/fault_detector_reverse.yaml`
- Modify: `config/Monitor/nav2_monitor/profiles/fault_detector_elevator.yaml`
- Modify: `config/Monitor/nav2_monitor/profiles/fault_detector_todoor.yaml`
- Modify: `src/nav2_monitor/config/fault_detector_config.yaml`
- Modify: `src/nav2_monitor/config/profiles/fault_detector_reverse.yaml`
- Modify: `src/nav2_monitor/config/profiles/fault_detector_elevator.yaml`
- Modify: `src/nav2_monitor/config/profiles/fault_detector_todoor.yaml`
- Modify: `src/bridge/test/test_monitor_ota_layout.py`

- [ ] **Step 1: Write/update the config layout assertions first**

Update `src/bridge/test/test_monitor_ota_layout.py` to assert:

```python
assert "front_ttc" in zone_names
assert "front_approach" not in zone_names
assert "points" not in front_ttc
assert front_ttc["model"] == "ttc"
```

- [ ] **Step 2: Run the OTA layout test and verify it fails**

Run:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_LOG_DIR=/tmp/ros_logs && colcon test --packages-select bridge --event-handlers console_direct+ --ctest-args -R test_monitor_ota_layout
```

Expected:
- the layout test fails because configs still use `front_approach/rear_approach`.

- [ ] **Step 3: Migrate the YAML files**

Update the repo OTA source config and every mirrored profile:

```yaml
- name: "front_ttc"
  model: "ttc"
  motion_direction: "forward"
  time_before_collision: 3.0
  recover_time_before_collision: 3.5
  min_hold_time_s: 0.2
  ttc_horizon_s: 3.5
  corridor_margin: 0.10
  candidate_downsample_resolution: 0.08
  actions: ["safety_system"]
```

Rules:
- remove TTC `points` entirely
- remove TTC `polygon_pub_topic`
- keep `front_slow/front_stop/rear_slow/rear_stop` unchanged
- make the package-local `src/nav2_monitor/config/...` files byte-for-byte match `config/Monitor/...`

- [ ] **Step 4: Re-run the OTA layout test and confirm it passes**

Run the same `bridge` test command.

Expected:
- `test_monitor_ota_layout` passes

- [ ] **Step 5: Commit the config migration**

```bash
git add config/Monitor/nav2_monitor/fault_detector_config.yaml config/Monitor/nav2_monitor/profiles/fault_detector_reverse.yaml config/Monitor/nav2_monitor/profiles/fault_detector_elevator.yaml config/Monitor/nav2_monitor/profiles/fault_detector_todoor.yaml src/nav2_monitor/config/fault_detector_config.yaml src/nav2_monitor/config/profiles/fault_detector_reverse.yaml src/nav2_monitor/config/profiles/fault_detector_elevator.yaml src/nav2_monitor/config/profiles/fault_detector_todoor.yaml src/bridge/test/test_monitor_ota_layout.py
git commit -m "feat(config): migrate collision rules to dynamic ttc"
```

### Task 5: Update Docs And Run End-To-End Verification

**Files:**
- Modify: `src/nav2_monitor/README.md`
- Modify: `src/nav2_monitor/docs/architecture.md`

- [ ] **Step 1: Update user-facing docs**

Document the new canonical model and its role:

```md
- `model: "ttc"` uses a runtime-generated corridor from `prediction_motion`
- `front_slow/front_stop/rear_*` remain static hard-stop polygons
- `ttc_visualization_enabled` now shows the dynamic corridor in RViz2
```

- [ ] **Step 2: Run full package verification**

Run:

```bash
source /opt/ros/humble/setup.bash && export ROS_LOG_DIR=/tmp/ros_logs && colcon build --packages-up-to nav2_monitor --cmake-clean-first --event-handlers console_direct+
```

Run:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_LOG_DIR=/tmp/ros_logs && colcon test --packages-select bridge nav2_monitor safety_emergency_executor --event-handlers console_direct+
```

Expected:
- all selected packages pass

- [ ] **Step 3: Refresh the runtime OTA directory**

Sync the validated bundle to the runtime path:

```bash
sudo rsync -a config/Monitor/ /opt/ry/config/Monitor/
```

Expected:
- `/opt/ry/config/Monitor/nav2_monitor` contains the new `front_ttc/rear_ttc` configs

- [ ] **Step 4: Optional smoke check for RViz marker topic**

Run:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_LOG_DIR=/tmp/ros_logs && timeout 6s ros2 run nav2_monitor nav2_monitor_node --ros-args --params-file src/nav2_monitor/config/nav2_monitor_params.yaml
```

Expected:
- node starts without config parsing errors
- `/nav2_monitor/collision_ttc_markers` is advertised when `ttc_visualization_enabled` is on

- [ ] **Step 5: Commit the docs and final verification result**

```bash
git add src/nav2_monitor/README.md src/nav2_monitor/docs/architecture.md
git commit -m "docs(nav2_monitor): document dynamic ttc zones"
```
