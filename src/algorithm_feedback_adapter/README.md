# algorithm_feedback_adapter

Algorithm feedback adapter package for `nav2_monitor/msg/AlgorithmFeedback`.

项目级数据链路见 [项目架构与数据链路](../../docs/project_architecture.html)。

## Modes

- `algorithm_feedback_adapter_node`: primary multi-topic, config-driven adapter
- `algorithm_feedback_adapter_cpp_node`: typed battery feedback adapter

当前仓库内额外提供了 `battery_feedback_bridge` 参考实现，
用于把 `sensor_msgs/msg/BatteryState` 转成统一 `AlgorithmFeedback`
的 `battery_percentage / battery_temperature / battery_voltage` 三个指标。

## Python spec format

The Python node reads a YAML file with a top-level `bridges:` list.
Each entry defines one input topic, message type, and the metrics to publish as `AlgorithmFeedback`.

仓库源文件：`config/Monitor/algorithm_feedback_adapter/algorithm_feedback_adapter_spec.yaml`

See `/opt/ry/config/Monitor/algorithm_feedback_adapter/algorithm_feedback_adapter_spec.yaml`.

## Hot Reload

- `spec_reload_enabled`: enable periodic spec reload polling
- `spec_reload_period_s`: polling period in seconds
- On invalid updated specs, the node keeps the last valid algorithm_feedback_adapter set running
- Removed algorithm_feedback_adapter entries are stopped immediately after a valid reload

## Added Examples

- `battery_feedback_bridge`：电池状态适配实现，提取电量、温度、电压
- `test_battery_feedback_bridge.cpp`：覆盖上述指标提取逻辑的单元测试
- `/drift_status`：`geometry_msgs/msg/PoseStamped` 配置适配示例，输出
  `drift_state / drift_delta_norm / drift_reserved` 三个统一反馈指标，并挂到 `light-lm`
