# Control Source Routing Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Upgrade `safety_emergency_executor` into a multi-source velocity router that only forwards the active control source, uses ROS 2 standard parameter services for source switching/querying, and keeps the existing safety command chain intact.

**Architecture:** Add a focused `ControlSourceController` plus a node-level parameter callback for `active_control_source`, then route four separate `Twist` inputs through the existing `VelocityConverter -> PressureAdjuster -> SafetyPolicyExecutor -> /command` pipeline only when the source matches the active mode. Replace the old `manual_override` API surface entirely, publish `/control_source_state` as the passive mode feed, and rely on the built-in `/set_parameters` and `/get_parameters` services instead of custom ROSIDL interfaces.

**Tech Stack:** C++17, ROS 2 Humble, `rclcpp`, `geometry_msgs`, `std_msgs`, `nav_msgs`, `sensor_msgs`, `ament_cmake_gtest`, `pytest`, `colcon`

---

## File Map

- Create: `src/safety_emergency_executor/include/safety_emergency_executor/control_source_controller.hpp`
  - Declare the focused controller that owns source validation and switching semantics.
- Create: `src/safety_emergency_executor/src/control_source_controller.cpp`
  - Implement canonical-source validation, current-source storage, and helper responses.
- Delete: `src/safety_emergency_executor/include/safety_emergency_executor/external_override_controller.hpp`
  - Remove the old boolean manual-override API surface.
- Delete: `src/safety_emergency_executor/src/external_override_controller.cpp`
  - Remove the old manual-override implementation.
- Modify: `src/safety_emergency_executor/include/safety_emergency_executor/safety_emergency_executor_node.hpp`
  - Replace manual-override members with control-source state publisher, parameter callback handle, and per-source subscriptions.
- Modify: `src/safety_emergency_executor/src/safety_emergency_executor_node.cpp`
  - Add multi-source routing, parameter validation, state publication, and remove old manual-override behavior.
- Modify: `src/safety_emergency_executor/CMakeLists.txt`
  - Wire in the new controller/test files and remove obsolete manual-override sources and dependencies.
- Modify: `src/safety_emergency_executor/package.xml`
  - Remove `std_srvs` if it becomes unused after deleting the old manual-override services.
- Modify: `src/safety_emergency_executor/README.md`
  - Document the `control_source` model, parameter-service usage, topics, and migration from `manual_override`.
- Modify: `src/safety_emergency_executor/config/safety_emergency_executor_params.yaml`
  - Keep the package-local example config aligned with the OTA bundle.
- Modify: `config/Monitor/safety_emergency_executor/safety_emergency_executor_params.yaml`
  - Change runtime parameters from single-input/manual-override to four inputs plus `active_control_source`.
- Modify: `src/bridge/test/test_monitor_ota_layout.py`
  - Add static assertions for the new OTA config keys and removal of legacy manual-override keys.
- Create: `src/safety_emergency_executor/test/test_control_source_controller.cpp`
  - Add focused unit coverage for source validation and switching.
- Create: `src/safety_emergency_executor/test/test_control_source_routing.cpp`
  - Add node-level coverage for per-source routing, parameter-service switching, and state publication.
- Modify: `src/safety_emergency_executor/test/test_pipeline_components.cpp`
  - Remove manual-override-specific coverage and keep only component tests that still belong here.

## Chunk 1: Controller Extraction

### Task 1: Replace Boolean Manual Override With A Focused Controller

**Files:**
- Create: `src/safety_emergency_executor/include/safety_emergency_executor/control_source_controller.hpp`
- Create: `src/safety_emergency_executor/src/control_source_controller.cpp`
- Delete: `src/safety_emergency_executor/include/safety_emergency_executor/external_override_controller.hpp`
- Delete: `src/safety_emergency_executor/src/external_override_controller.cpp`
- Create: `src/safety_emergency_executor/test/test_control_source_controller.cpp`
- Modify: `src/safety_emergency_executor/CMakeLists.txt`

- [ ] **Step 1: Write the failing controller tests**

Create `src/safety_emergency_executor/test/test_control_source_controller.cpp`:

```cpp
#include <gtest/gtest.h>

#include "safety_emergency_executor/control_source_controller.hpp"

TEST(ControlSourceControllerTest, DefaultsToNavigation)
{
  safety_emergency_executor::ControlSourceController controller("navigation", false);
  EXPECT_EQ(controller.active_source(), "navigation");
}

TEST(ControlSourceControllerTest, SwitchesBetweenCanonicalSources)
{
  safety_emergency_executor::ControlSourceController controller("navigation", false);
  const auto result = controller.set_active_source("remote");
  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.changed);
  EXPECT_EQ(result.active_source, "remote");
  EXPECT_EQ(controller.active_source(), "remote");
}

TEST(ControlSourceControllerTest, RejectsInvalidSources)
{
  safety_emergency_executor::ControlSourceController controller("navigation", false);
  const auto result = controller.set_active_source("invalid");
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.changed);
  EXPECT_EQ(controller.active_source(), "navigation");
}
```

- [ ] **Step 2: Run the targeted test build to verify it fails**

Run:

```bash
source /opt/ros/humble/setup.bash && colcon build --packages-select safety_emergency_executor --event-handlers console_direct+
```

Expected:
- build fails because `control_source_controller.hpp/.cpp` do not exist yet

- [ ] **Step 3: Implement `ControlSourceController` and remove `ExternalOverrideController`**

Create `src/safety_emergency_executor/include/safety_emergency_executor/control_source_controller.hpp`:

```cpp
struct ControlSourceChange
{
  bool success{false};
  bool changed{false};
  std::string active_source;
  std::string message;
};

class ControlSourceController
{
public:
  explicit ControlSourceController(std::string initial_source, bool auto_preempt_enabled);

  const std::string & active_source() const;
  bool accepts(const std::string & source) const;
  ControlSourceChange set_active_source(const std::string & source);
  static bool is_valid_source(const std::string & source);

private:
  std::string active_source_;
  bool auto_preempt_enabled_{false};
};
```

Implement canonical-source validation in `src/safety_emergency_executor/src/control_source_controller.cpp` and remove both `external_override_controller.*` files from the build.

- [ ] **Step 4: Re-run the targeted tests and make them pass**

Run:

```bash
source /opt/ros/humble/setup.bash && colcon build --packages-select safety_emergency_executor --event-handlers console_direct+
source install/setup.bash && colcon test --packages-select safety_emergency_executor --event-handlers console_direct+ --ctest-args -R test_control_source_controller
```

Expected:
- package builds successfully
- `test_control_source_controller` passes

- [ ] **Step 5: Commit the controller extraction**

```bash
git add src/safety_emergency_executor/include/safety_emergency_executor/control_source_controller.hpp src/safety_emergency_executor/src/control_source_controller.cpp src/safety_emergency_executor/test/test_control_source_controller.cpp src/safety_emergency_executor/CMakeLists.txt
git rm src/safety_emergency_executor/include/safety_emergency_executor/external_override_controller.hpp src/safety_emergency_executor/src/external_override_controller.cpp
git commit -m "feat(safety_emergency_executor): add control source controller"
```

## Chunk 2: Parameter-Driven Routing

### Task 2: Add Multi-Source Routing And Parameter-Based Switching

**Files:**
- Modify: `src/safety_emergency_executor/include/safety_emergency_executor/safety_emergency_executor_node.hpp`
- Modify: `src/safety_emergency_executor/src/safety_emergency_executor_node.cpp`
- Create: `src/safety_emergency_executor/test/test_control_source_routing.cpp`
- Modify: `src/safety_emergency_executor/CMakeLists.txt`
- Modify: `src/safety_emergency_executor/test/test_pipeline_components.cpp`
- Modify: `src/safety_emergency_executor/package.xml`

- [ ] **Step 1: Write the failing routing and parameter-service tests**

Create `src/safety_emergency_executor/test/test_control_source_routing.cpp` with a dedicated harness:

```cpp
class ControlSourceHarness : public rclcpp::Node
{
public:
  ControlSourceHarness()
  : Node("control_source_harness")
  {
    navigation_pub_ = create_publisher<geometry_msgs::msg::Twist>("/test/cmd_vel/navigation", 20);
    remote_pub_ = create_publisher<geometry_msgs::msg::Twist>("/test/cmd_vel/remote", 20);
    miniapp_pub_ = create_publisher<geometry_msgs::msg::Twist>("/test/cmd_vel/miniapp", 20);
    other_pub_ = create_publisher<geometry_msgs::msg::Twist>("/test/cmd_vel/other", 20);
    command_sub_ = create_subscription<std_msgs::msg::String>("/test/command", 20, ...);
    state_sub_ = create_subscription<std_msgs::msg::String>(
      "/test/control_source_state", rclcpp::QoS(1).transient_local().reliable(), ...);
    params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(shared_from_this(), "/safety_emergency_executor");
  }
};
```

Add tests:

```cpp
TEST_F(SafetyExecutorRoutingTest, DefaultsToNavigationAndPublishesState)
TEST_F(SafetyExecutorRoutingTest, OnlyActiveSourceCommandsReachCommandTopic)
TEST_F(SafetyExecutorRoutingTest, ParameterUpdateSwitchesActiveSource)
TEST_F(SafetyExecutorRoutingTest, InvalidParameterUpdateIsRejected)
TEST_F(SafetyExecutorRoutingTest, SwitchingSourceDoesNotPublishSyntheticStop)
TEST_F(SafetyExecutorRoutingTest, SafetyCommandsStillApplyAfterSwitchingToRemote)
```

Trim `src/safety_emergency_executor/test/test_pipeline_components.cpp` so it keeps only reusable component tests (`VelocityConverter`, `PressureAdjuster`, `SafetyPolicyExecutor`) and removes the old manual-override harness.

- [ ] **Step 2: Run the targeted suite and verify it fails**

Run:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test --packages-select safety_emergency_executor --event-handlers console_direct+ --ctest-args -R "test_control_source_routing|test_pipeline_components"
```

Expected:
- the new routing suite fails because the node still subscribes to only one `cmd_vel` topic and does not validate `active_control_source`

- [ ] **Step 3: Implement parameter-driven routing in the node**

Update `src/safety_emergency_executor/include/safety_emergency_executor/safety_emergency_executor_node.hpp`:

```cpp
void on_cmd_vel(const std::string & source, const geometry_msgs::msg::Twist::SharedPtr msg);
void publish_control_source_state();
rcl_interfaces::msg::SetParametersResult on_parameter_update(
  const std::vector<rclcpp::Parameter> & parameters);

std::unordered_map<std::string, rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> cmd_vel_subscriptions_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_source_state_pub_;
OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
ControlSourceController control_source_controller_;
```

Update `src/safety_emergency_executor/src/safety_emergency_executor_node.cpp`:

```cpp
const auto active_control_source = declare_parameter<std::string>("active_control_source", "navigation");
const auto navigation_topic = declare_parameter<std::string>("cmd_vel_navigation_topic", "/cmd_vel");
const auto miniapp_topic = declare_parameter<std::string>("cmd_vel_miniapp_topic", "/cmd_vel_miniapp");
const auto remote_topic = declare_parameter<std::string>("cmd_vel_remote_topic", "/cmd_vel_remote");
const auto other_topic = declare_parameter<std::string>("cmd_vel_other_topic", "/cmd_vel_other");

control_source_controller_ = ControlSourceController(active_control_source, auto_preempt_enabled);
parameter_callback_handle_ = add_on_set_parameters_callback(
  std::bind(&SafetyEmergencyExecutorNode::on_parameter_update, this, std::placeholders::_1));
publish_control_source_state();
```

In `on_parameter_update`:

```cpp
if (parameter.get_name() == "active_control_source") {
  const auto result = control_source_controller_.set_active_source(parameter.as_string());
  response.successful = result.success;
  response.reason = result.message;
  if (result.success && result.changed) {
    publish_control_source_state();
  }
}
```

In `on_cmd_vel`:

```cpp
if (!control_source_controller_.accepts(source)) {
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Ignoring cmd_vel from inactive source: %s", source.c_str());
  return;
}
```

Remove all manual-override service/state logic and delete the now-unused `std_srvs` dependency from `CMakeLists.txt` and `package.xml` if nothing else in the package uses it.

- [ ] **Step 4: Re-run the routing and component tests**

Run:

```bash
source /opt/ros/humble/setup.bash && colcon build --packages-select safety_emergency_executor --event-handlers console_direct+
source install/setup.bash && colcon test --packages-select safety_emergency_executor --event-handlers console_direct+ --ctest-args -R "test_control_source_routing|test_pipeline_components"
```

Expected:
- `test_control_source_routing` passes all source-routing and parameter-switch scenarios
- `test_pipeline_components` still passes the component-level safety-policy assertions

- [ ] **Step 5: Commit the node routing upgrade**

```bash
git add src/safety_emergency_executor/include/safety_emergency_executor/safety_emergency_executor_node.hpp src/safety_emergency_executor/src/safety_emergency_executor_node.cpp src/safety_emergency_executor/test/test_control_source_routing.cpp src/safety_emergency_executor/test/test_pipeline_components.cpp src/safety_emergency_executor/CMakeLists.txt src/safety_emergency_executor/package.xml
git commit -m "feat(safety_emergency_executor): route velocity by control source"
```

## Chunk 3: Config And Documentation Migration

### Task 3: Migrate Runtime Configuration And Documentation

**Files:**
- Modify: `config/Monitor/safety_emergency_executor/safety_emergency_executor_params.yaml`
- Modify: `src/safety_emergency_executor/config/safety_emergency_executor_params.yaml`
- Modify: `src/safety_emergency_executor/README.md`
- Modify: `src/bridge/test/test_monitor_ota_layout.py`

- [ ] **Step 1: Write the failing static config assertions**

Extend `src/bridge/test/test_monitor_ota_layout.py`:

```python
def test_safety_executor_uses_control_source_routing_params():
    params = yaml.safe_load(
        (MONITOR_ROOT / "safety_emergency_executor" / "safety_emergency_executor_params.yaml").read_text()
    )["safety_emergency_executor"]["ros__parameters"]

    assert params["cmd_vel_navigation_topic"] == "/cmd_vel"
    assert params["active_control_source"] == "navigation"
    assert params["control_source_state_topic"] == "/control_source_state"
    assert "cmd_vel_topic" not in params
    assert "manual_override_service" not in params
    assert "manual_override_query_service" not in params
    assert "manual_override_state_topic" not in params
```

- [ ] **Step 2: Run the static config test and verify it fails**

Run:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test --packages-select bridge --event-handlers console_direct+ --ctest-args -R test_monitor_ota_layout
```

Expected:
- the new assertions fail because the YAML files still expose `cmd_vel_topic` and `manual_override_*`

- [ ] **Step 3: Migrate the YAML and README content**

Update both parameter files to:

```yaml
cmd_vel_navigation_topic: "/cmd_vel"
cmd_vel_miniapp_topic: "/cmd_vel_miniapp"
cmd_vel_remote_topic: "/cmd_vel_remote"
cmd_vel_other_topic: "/cmd_vel_other"
control_source_state_topic: "/control_source_state"
active_control_source: "navigation"
control_source_auto_preempt_enabled: false
```

Rewrite `src/safety_emergency_executor/README.md` so the public interface section explicitly states:

- `control_source` replaces `manual_override`
- source switching uses `/safety_emergency_executor/set_parameters` on `active_control_source`
- source querying uses `/safety_emergency_executor/get_parameters`
- only the active source reaches `/command`
- source switching does not synthesize stop commands
- safety commands stay effective regardless of active source

- [ ] **Step 4: Re-run static verification and the package test suite**

Run:

```bash
source /opt/ros/humble/setup.bash && colcon build --packages-select safety_emergency_executor bridge --event-handlers console_direct+
source install/setup.bash && colcon test --packages-select safety_emergency_executor bridge --event-handlers console_direct+
```

Expected:
- `bridge` layout/static config tests pass
- all `safety_emergency_executor` tests pass

- [ ] **Step 5: Commit the config and docs migration**

```bash
git add config/Monitor/safety_emergency_executor/safety_emergency_executor_params.yaml src/safety_emergency_executor/config/safety_emergency_executor_params.yaml src/safety_emergency_executor/README.md src/bridge/test/test_monitor_ota_layout.py
git commit -m "docs(safety_emergency_executor): document control source routing"
```

## Chunk 4: End-To-End Verification

### Task 4: Run A Final Focused Verification Pass

**Files:**
- Modify: none
- Test: `src/safety_emergency_executor/test/test_control_source_controller.cpp`
- Test: `src/safety_emergency_executor/test/test_control_source_routing.cpp`
- Test: `src/safety_emergency_executor/test/test_pipeline_components.cpp`
- Test: `src/bridge/test/test_monitor_ota_layout.py`

- [ ] **Step 1: Run the focused package build from a clean shell**

Run:

```bash
source /opt/ros/humble/setup.bash && colcon build --packages-select safety_emergency_executor bridge --event-handlers console_direct+
```

Expected:
- both packages build without missing-header or obsolete-service dependency errors

- [ ] **Step 2: Run the focused tests for source routing and static config**

Run:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test --packages-select safety_emergency_executor bridge --event-handlers console_direct+ --ctest-args -R "test_control_source_controller|test_control_source_routing|test_pipeline_components|test_monitor_ota_layout"
```

Expected:
- all targeted tests pass

- [ ] **Step 3: Inspect the test result summaries**

Run:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test-result --verbose --test-result-base build/safety_emergency_executor
source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test-result --verbose --test-result-base build/bridge
```

Expected:
- no failed tests listed in either package

- [ ] **Step 4: Record the migration impact in the final handoff notes**

Capture these points in the completion summary:

- old `manual_override` interfaces were removed
- control-source switching/querying now uses standard parameter services on `active_control_source`
- default source is `navigation`
- inactive-source messages are dropped without synthesized stop commands
- safety commands still apply across all sources

- [ ] **Step 5: Final commit if any verification-only note changed**

```bash
git status --short
```

Expected:
- working tree clean
- no extra commit needed beyond the feature commits above
