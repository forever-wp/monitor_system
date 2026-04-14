# External Pressure Override Implementation Plan

> **For agentic workers:** REQUIRED: Use superpowers:subagent-driven-development (if subagents available) or superpowers:executing-plans to implement this plan. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Change `/pressure_` to a single-value `std_msgs/Int32` pressure override topic, make each external pressure update hold off automatic pressure adjustment for 30 seconds, and resume automatic adjustment afterward using the external value as the new baseline.

**Architecture:** Keep the current `VelocityConverter -> PressureAdjuster -> SafetyPolicyExecutor -> /command` pipeline, but split the new behavior cleanly across responsibilities: `VelocityConverter` owns the baseline command template fields, `PressureAdjuster` owns the override hold-window gate for automatic pressure adjustment, and `SafetyEmergencyExecutorNode` only wires the new `Int32` topic into those two components. Preserve the existing embedded-Twist pressure override semantics and keep them higher priority than the topic-based override.

**Tech Stack:** C++17, ROS 2 Humble, `rclcpp`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `ament_cmake_gtest`, `colcon`

---

## File Map

- Modify: `src/safety_emergency_executor/include/safety_emergency_executor/velocity_converter.hpp`
  - Replace the JSON pressure-update API with a single-value pressure update method and keep `acc` override behavior intact.
- Modify: `src/safety_emergency_executor/src/velocity_converter.cpp`
  - Implement single-value pressure updates and remove JSON pressure parsing from the runtime path.
- Modify: `src/safety_emergency_executor/include/safety_emergency_executor/pressure_adjuster.hpp`
  - Add the external pressure hold-window state and public hook for starting or refreshing the hold period.
- Modify: `src/safety_emergency_executor/src/pressure_adjuster.cpp`
  - Implement the “skip automatic pressure adjustment while hold window is active” behavior.
- Modify: `src/safety_emergency_executor/include/safety_emergency_executor/safety_emergency_executor_node.hpp`
  - Change `/pressure_` subscription type from `std_msgs::msg::String` to `std_msgs::msg::Int32`.
- Modify: `src/safety_emergency_executor/src/safety_emergency_executor_node.cpp`
  - Subscribe to `std_msgs::msg::Int32`, update the baseline pressure in `VelocityConverter`, and refresh the hold window in `PressureAdjuster`.
- Modify: `src/safety_emergency_executor/test/test_pipeline_components.cpp`
  - Replace JSON pressure-update coverage with `Int32`-style baseline pressure tests and hold-window tests.
- Modify: `src/safety_emergency_executor/test/test_control_source_routing.cpp`
  - Add node-level coverage proving `/pressure_` affects outgoing command pressure, blocks auto-adjustment during the hold window, and resumes auto-adjustment after expiry.
- Modify: `src/safety_emergency_executor/config/safety_emergency_executor_params.yaml`
  - Document `pressure_update_topic` as `Int32` and add `external_pressure_hold_s`.
- Modify: `config/Monitor/safety_emergency_executor/safety_emergency_executor_params.yaml`
  - Keep the OTA runtime config in sync with the package-local example config.
- Modify: `src/safety_emergency_executor/README.md`
  - Replace the JSON `/pressure_` documentation with the `Int32` topic and 30-second hold-window behavior.

## Chunk 1: Pressure Topic Interface Refactor

### Task 1: Replace JSON Pressure Updates With A Single-Value Pressure API

**Files:**
- Modify: `src/safety_emergency_executor/include/safety_emergency_executor/velocity_converter.hpp`
- Modify: `src/safety_emergency_executor/src/velocity_converter.cpp`
- Modify: `src/safety_emergency_executor/test/test_pipeline_components.cpp`

- [ ] **Step 1: Write the failing component tests**

Update `src/safety_emergency_executor/test/test_pipeline_components.cpp` by replacing the JSON pressure-update coverage with tests like:

```cpp
TEST_F(SafetyExecutorComponentTest, VelocityConverterPressureTopicUpdatesBaselinePress)
{
  auto node = std::make_shared<rclcpp::Node>("velocity_converter_pressure_override_test");
  safety_emergency_executor::VelocityConverter converter;
  converter.configure(*node);

  converter.update_press_from_topic(1100);
  const auto frame = converter.template_frame();

  EXPECT_EQ(frame.press, 1100);
}

TEST_F(SafetyExecutorComponentTest, VelocityConverterAccTopicOverrideStillWorksIndependently)
{
  auto node = std::make_shared<rclcpp::Node>("velocity_converter_acc_override_test");
  safety_emergency_executor::VelocityConverter converter;
  converter.configure(*node);

  converter.update_press_from_topic(1100);
  converter.update_acc_from_topic(3200);
  const auto frame = converter.template_frame();

  EXPECT_EQ(frame.press, 1100);
  EXPECT_EQ(frame.acc, 3200);
}
```

- [ ] **Step 2: Run the targeted component test and verify it fails**

Run:

```bash
source /opt/ros/humble/setup.bash && colcon test --packages-select safety_emergency_executor --event-handlers console_direct+ --ctest-args -R test_pipeline_components
```

Expected:
- `test_pipeline_components` fails because `VelocityConverter` does not yet expose `update_press_from_topic(int)`

- [ ] **Step 3: Implement the minimal baseline pressure update API**

Update `src/safety_emergency_executor/include/safety_emergency_executor/velocity_converter.hpp`:

```cpp
class VelocityConverter
{
public:
  void update_press_from_topic(int press_value);
  void update_acc_from_topic(int acc_value);
  CommandFrame template_frame() const;
};
```

Update `src/safety_emergency_executor/src/velocity_converter.cpp`:

```cpp
void VelocityConverter::update_press_from_topic(int press_value)
{
  params_.press = press_value;
}
```

Also remove the obsolete JSON pressure-update path from the runtime API:

```cpp
bool update_params_from_json(...) = delete;
```

In practice, remove `update_params_from_json()` from both the header and the `.cpp`, and leave embedded-Twist parsing unchanged.

- [ ] **Step 4: Re-run the targeted component test and make it pass**

Run:

```bash
source /opt/ros/humble/setup.bash && colcon test --packages-select safety_emergency_executor --event-handlers console_direct+ --ctest-args -R test_pipeline_components
```

Expected:
- `test_pipeline_components` passes with the new baseline pressure API

- [ ] **Step 5: Commit the pressure API refactor**

```bash
git add src/safety_emergency_executor/include/safety_emergency_executor/velocity_converter.hpp src/safety_emergency_executor/src/velocity_converter.cpp src/safety_emergency_executor/test/test_pipeline_components.cpp
git commit -m "refactor(safety_emergency_executor): use int pressure override input"
```

## Chunk 2: External Pressure Hold Window

### Task 2: Block Automatic Pressure Adjustment For 30 Seconds After External Pressure Updates

**Files:**
- Modify: `src/safety_emergency_executor/include/safety_emergency_executor/pressure_adjuster.hpp`
- Modify: `src/safety_emergency_executor/src/pressure_adjuster.cpp`
- Modify: `src/safety_emergency_executor/test/test_pipeline_components.cpp`

- [ ] **Step 1: Write the failing hold-window tests**

Extend `src/safety_emergency_executor/test/test_pipeline_components.cpp` with focused tests:

```cpp
TEST_F(SafetyExecutorComponentTest, ExternalPressureHoldWindowBypassesAutomaticAdjustment)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("external_pressure_hold_s", 30.0)
  });
  auto node = std::make_shared<rclcpp::Node>("pressure_adjuster_hold_window_test", options);

  safety_emergency_executor::PressureAdjuster adjuster;
  adjuster.configure(*node);
  adjuster.note_external_pressure_override(node->get_clock()->now());

  safety_emergency_executor::CommandFrame frame;
  frame.press = 1100;
  adjuster.apply(frame);

  EXPECT_EQ(frame.press, 1100);
}

TEST_F(SafetyExecutorComponentTest, AutomaticAdjustmentResumesAfterHoldWindowExpires)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("external_pressure_hold_s", 0.01)
  });
  auto node = std::make_shared<rclcpp::Node>("pressure_adjuster_resume_test", options);

  safety_emergency_executor::PressureAdjuster adjuster;
  adjuster.configure(*node);
  adjuster.note_external_pressure_override(node->get_clock()->now());

  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  safety_emergency_executor::CommandFrame frame;
  frame.press = 1100;
  // add wheel/localization data here so auto-pressure has enough inputs to change the pressure
  adjuster.apply(frame);

  EXPECT_NE(frame.press, 1100);
}
```

Use the same style already present in `PressureAdjusterDisabledModeLeavesPressureUnchanged`, but add enough odom/IMU fixture setup to force a real auto-pressure adjustment after expiry.

- [ ] **Step 2: Run the targeted component test and verify it fails**

Run:

```bash
source /opt/ros/humble/setup.bash && colcon test --packages-select safety_emergency_executor --event-handlers console_direct+ --ctest-args -R test_pipeline_components
```

Expected:
- the new hold-window tests fail because `PressureAdjuster` has no external override timer state yet

- [ ] **Step 3: Implement the hold-window behavior in `PressureAdjuster`**

Update `src/safety_emergency_executor/include/safety_emergency_executor/pressure_adjuster.hpp`:

```cpp
class PressureAdjuster
{
public:
  void configure(rclcpp::Node & node);
  void note_external_pressure_override(const rclcpp::Time & stamp);
  void apply(CommandFrame & frame);

private:
  bool external_pressure_hold_active(const rclcpp::Time & now) const;

  rclcpp::Clock::SharedPtr clock_{};
  rclcpp::Time external_pressure_hold_until_{0, 0, RCL_ROS_TIME};
  double external_pressure_hold_s_{30.0};
};
```

Update `src/safety_emergency_executor/src/pressure_adjuster.cpp`:

```cpp
void PressureAdjuster::configure(rclcpp::Node & node)
{
  clock_ = node.get_clock();
  external_pressure_hold_s_ =
    node.declare_parameter<double>("external_pressure_hold_s", 30.0);
  adjuster_.configure(node);
}

void PressureAdjuster::note_external_pressure_override(const rclcpp::Time & stamp)
{
  if (external_pressure_hold_s_ <= 0.0) {
    external_pressure_hold_until_ = stamp;
    return;
  }
  external_pressure_hold_until_ = stamp + rclcpp::Duration::from_seconds(external_pressure_hold_s_);
}

void PressureAdjuster::apply(CommandFrame & frame)
{
  if (frame.press_from_embedded_fields) {
    return;
  }
  if (clock_ && clock_->now() < external_pressure_hold_until_) {
    return;
  }
  // existing auto-pressure logic stays here
}
```

The important invariant is: during the hold window, `frame.press` must remain exactly the externally supplied baseline pressure.

- [ ] **Step 4: Re-run the targeted component test and make it pass**

Run:

```bash
source /opt/ros/humble/setup.bash && colcon test --packages-select safety_emergency_executor --event-handlers console_direct+ --ctest-args -R test_pipeline_components
```

Expected:
- `test_pipeline_components` passes
- hold-window tests prove that automatic adjustment is bypassed before expiry and resumes after expiry

- [ ] **Step 5: Commit the hold-window logic**

```bash
git add src/safety_emergency_executor/include/safety_emergency_executor/pressure_adjuster.hpp src/safety_emergency_executor/src/pressure_adjuster.cpp src/safety_emergency_executor/test/test_pipeline_components.cpp
git commit -m "feat(safety_emergency_executor): hold external pressure overrides"
```

## Chunk 3: Node Wiring, Config, And End-To-End Verification

### Task 3: Wire `/pressure_` As `Int32`, Update Docs, And Verify Node Behavior

**Files:**
- Modify: `src/safety_emergency_executor/include/safety_emergency_executor/safety_emergency_executor_node.hpp`
- Modify: `src/safety_emergency_executor/src/safety_emergency_executor_node.cpp`
- Modify: `src/safety_emergency_executor/test/test_control_source_routing.cpp`
- Modify: `src/safety_emergency_executor/README.md`
- Modify: `src/safety_emergency_executor/config/safety_emergency_executor_params.yaml`
- Modify: `config/Monitor/safety_emergency_executor/safety_emergency_executor_params.yaml`

- [ ] **Step 1: Write the failing node-level and config assertions**

Update `src/safety_emergency_executor/test/test_control_source_routing.cpp` with tests like:

```cpp
TEST_F(SafetyExecutorRoutingTest, PressureTopicOverrideUpdatesOutgoingCommandPressure)
TEST_F(SafetyExecutorRoutingTest, PressureTopicOverrideBypassesAutoPressureDuringHoldWindow)
TEST_F(SafetyExecutorRoutingTest, AutoPressureResumesAfterPressureHoldWindowExpires)
```

Update both YAML files to add the new parameter key expectation in tests only after the runtime tests are in place:

```yaml
external_pressure_hold_s: 30.0
```

- [ ] **Step 2: Run the routing suite and verify it fails**

Run:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test --packages-select safety_emergency_executor --event-handlers console_direct+ --ctest-args -R test_control_source_routing
```

Expected:
- routing tests fail because `/pressure_` is still subscribed as `std_msgs::msg::String`
- the node does not yet refresh the external hold window when pressure messages arrive

- [ ] **Step 3: Implement node wiring and update docs/config**

Update `src/safety_emergency_executor/include/safety_emergency_executor/safety_emergency_executor_node.hpp`:

```cpp
void on_pressure_update(const std_msgs::msg::Int32::SharedPtr msg);
rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pressure_sub_;
```

Update `src/safety_emergency_executor/src/safety_emergency_executor_node.cpp`:

```cpp
pressure_sub_ = this->create_subscription<std_msgs::msg::Int32>(
  pressure_update_topic, rclcpp::QoS(10),
  std::bind(&SafetyEmergencyExecutorNode::on_pressure_update, this, std::placeholders::_1));

void SafetyEmergencyExecutorNode::on_pressure_update(const std_msgs::msg::Int32::SharedPtr msg)
{
  velocity_converter_.update_press_from_topic(msg->data);
  pressure_adjuster_.note_external_pressure_override(this->get_clock()->now());
}
```

Update both parameter files:

```yaml
pressure_update_topic: "/pressure_"  # 外部单值压力覆盖 topic，std_msgs/Int32
external_pressure_hold_s: 30.0  # 外部设压后自动调压冻结时长（秒）
```

Update `src/safety_emergency_executor/README.md` to replace the JSON `/pressure_` description with:

```markdown
- `pressure_update_topic`（`std_msgs/Int32`，只更新基础压力，并在 `external_pressure_hold_s` 内禁止自动调压）
```

- [ ] **Step 4: Run full package verification and make it pass**

Run:

```bash
source /opt/ros/humble/setup.bash && colcon build --packages-select safety_emergency_executor
source install/setup.bash && colcon test --packages-select safety_emergency_executor --event-handlers console_direct+ --ctest-args -R "test_pipeline_components|test_control_source_routing"
```

Expected:
- package builds
- component and routing tests pass
- the outgoing command pressure reflects `/pressure_` immediately
- auto-pressure stays out for 30 seconds, then resumes from the external value as the new baseline

- [ ] **Step 5: Commit the node wiring, config, and docs**

```bash
git add src/safety_emergency_executor/include/safety_emergency_executor/safety_emergency_executor_node.hpp src/safety_emergency_executor/src/safety_emergency_executor_node.cpp src/safety_emergency_executor/test/test_control_source_routing.cpp src/safety_emergency_executor/README.md src/safety_emergency_executor/config/safety_emergency_executor_params.yaml config/Monitor/safety_emergency_executor/safety_emergency_executor_params.yaml
git commit -m "feat(safety_emergency_executor): add external pressure hold window"
```
