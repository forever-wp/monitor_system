# bridge

Generic bridge package for `nav2_monitor/msg/AlgorithmFeedback`.

## Modes

- `bridge_py_node`: primary multi-topic, config-driven bridge
- `bridge_cpp_node`: example typed bridge template

当前仓库内额外提供了 `battery_feedback_bridge` 参考实现，
用于把 `sensor_msgs/msg/BatteryState` 转成统一 `AlgorithmFeedback`
的 `battery_percentage / battery_temperature / battery_voltage` 三个指标。

## Python spec format

The Python node reads a YAML file with a top-level `bridges:` list.
Each entry defines one input topic, message type, and the metrics to publish as `AlgorithmFeedback`.

仓库源文件：`config/Monitor/bridge/generic_multi_bridge_spec.yaml`

See `/opt/ry/config/Monitor/bridge/generic_multi_bridge_spec.yaml`.

## Hot Reload

- `spec_reload_enabled`: enable periodic spec reload polling
- `spec_reload_period_s`: polling period in seconds
- On invalid updated specs, the node keeps the last valid bridge set running
- Removed bridge entries are stopped immediately after a valid reload

## Added Examples

- `battery_feedback_bridge`：电池状态桥接参考实现，提取电量、温度、电压
- `test_battery_feedback_bridge.cpp`：覆盖上述指标提取逻辑的单元测试
