# Drift Status Bridge Design

## Goal

将 `/drift_status` (`geometry_msgs/msg/PoseStamped`) 接入现有 `bridge_py_node`，
统一转换为 `nav2_monitor/msg/AlgorithmFeedback`，并挂到 `light-lm` 模块下。

## Chosen Design

- 不新增专用 bridge 节点
- 直接在 `config/Monitor/bridge/generic_multi_bridge_spec.yaml` 中新增一条配置驱动桥接
- `module_name` 固定为 `light-lm`
- `topic_name` 固定为 `/drift_status`

## Metrics

- `drift_state` -> `pose.position.x`
  - `1.0` 表示检测到漂移
  - `2.0` 表示正常
- `drift_delta_norm` -> `pose.position.y`
  - 表示位移差模长
- `drift_reserved` -> `pose.position.z`
  - 保留字段，当前默认 `-1.0`

## Monitor Rules

`nav2_monitor` 中的 `light-lm` 模块增加独立 `feedback_rules`：

- 默认启用 `drift_state`
  - 使用阈值规则区分 `1.0` 和 `2.0`
- 同时补充 `drift_delta_norm` 独立规则模板
  - 保持与 `drift_state` 分开判断
  - 便于后续只启用其中一个

## Affected Files

- `config/Monitor/bridge/generic_multi_bridge_spec.yaml`
- `src/bridge/test/test_monitor_ota_layout.py`
- `config/Monitor/nav2_monitor/fault_detector_config.yaml`
- `config/Monitor/nav2_monitor/profiles/*.yaml`
- `src/nav2_monitor/config/*.yaml`
- `src/nav2_monitor/README.md`
