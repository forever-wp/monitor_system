# nav2_monitor

- 名称：nav2_monitor v2
- 版本：v2
- 时间：2026-03-12
- 作者：ToTo

轻量级 Nav2 监控、故障检测与安全联动系统。

## 设计原则

- 低占用
- 小内存
- 快响应

## 文档入口

- 设计说明：`src/nav2_monitor/docs/architecture.md`
- 流程图 / 数据流图：`src/nav2_monitor/docs/architecture_diagrams.md`

## 功能概览

当前版本已支持：

- 节点在线监控
- `watch_topics` 频率与发布者监控
- `feedback_rules` 统一反馈规则判断
- 底盘异常与久停判断
- 多故障组合与安全动作仲裁
- 自动恢复与 `RESUME`
- 碰撞检测：`LaserScan` / `PointCloud2`
- 碰撞策略：`slowdown zone` / `stop zone` / `approach(TTC)`
- 碰撞区域可视化

## 架构摘要

当前主链路：

`ROS输入 -> Nav2MonitorNode -> MonitorDataStore -> FaultDetector -> FaultStateCoordinator -> SafetyCmd -> safety_emergency_executor`

模块职责：

- `Nav2MonitorNode`：ROS 接线、数据采集、状态发布、事件发布
- `MonitorDataStore`：统一保存最新有效快照
- `FaultDetector`：加载配置并编排 evaluator
- `WatchTopicEvaluator`：直接监控 topic 规则
- `FeedbackRuleEvaluator`：统一反馈规则
- `ChassisEvaluator`：底盘异常 / 久停规则
- `CollisionEvaluator`：碰撞检测规则
- `FaultStateCoordinator`：故障边沿、状态机、安全动作发布
- `safety_emergency_executor`：执行 `SLOW_DOWN / SOFT_STOP / EMERGENCY_STOP / RESUME`

详细职责见 `src/nav2_monitor/docs/architecture.md`。

## 快速启动

```bash
colcon build --packages-select nav2_monitor safety_emergency_executor
source install/setup.bash

ros2 launch nav2_monitor nav2_monitor.launch.py
ros2 launch safety_emergency_executor safety_emergency_executor.launch.py
```

## 关键配置

### 主参数

文件：`src/nav2_monitor/config/nav2_monitor_params.yaml`

关键字段：

- `timeout`
- `scan_rate`
- `check_rate`
- `algorithm_feedback_topic`
- `battery_state_topic`
- `fault_config`
- `target_transforms`

### 故障配置

文件：`src/nav2_monitor/config/fault_detector_config.yaml`

当前主要配置块：

- `collision_detection`
- `chassis_stationary`
- `modules`

### 配置语义

- `watch_topics`
  - 表示 `nav2_monitor` 直接监控的真实 ROS topic
  - 用于发布者状态与频率判断

- `feedback_rules`
  - 表示统一反馈规则
  - `source_topic` 对应 `AlgorithmFeedback.topic_name`
  - `metric_name` 对应 `AlgorithmFeedback.metric_name`

- `collision_detection.zones`
  - `model` 支持：
    - 默认 zone 命中
    - `approach`
  - `actions` 支持：
    - `safety_system`
    - `supervisor`
  - `safety_system` 支持：
    - `0` 不执行
    - `1` 减速
    - `2` 缓停
    - `3` 急停

> 旧字段 `topics`、`feedback_topics`、`topic_name` 已不再接受。

## 输入 / 输出

### 输入

- `/nav2_monitor/algorithm_feedback`
- `/command`
- `/moto_info`
- `/odom`
- `/battery_state`
- `collision_detection.scan_topic`
- `collision_detection.pointcloud_topic`

### 输出

- `/nav2_monitor/status`
- `/nav2_monitor/fault_event`
- `/supervisor/cmd`
- `/safety_system/cmd`
- `collision_detection.zones[*].polygon_pub_topic`

## 安全动作语义

`FaultStateCoordinator` 会聚合所有当前激活的安全故障，统一选择最高安全动作：

- `EMERGENCY_STOP > SOFT_STOP > SLOW_DOWN > NORMAL`

恢复逻辑：

- 同一故障消失时发 `RECOVER` 边沿事件
- 所有安全故障恢复后自动发布 `RESUME`

## 碰撞检测说明

当前碰撞检测是轻量版实现，目标是快速接入而不是完整复刻 Nav2 Collision Monitor。

已支持：

- `LaserScan`
- `PointCloud2`
- polygon 区域
- `slowdown zone`
- `stop zone`
- `approach / TTC`
- 区域可视化 `PolygonStamped`

特点：

- 只保留最新一帧有效点集
- 多 source 统一聚合
- 小计算量几何判断
- 直接进入现有安全链路

## 常用调试命令

```bash
ros2 topic echo /nav2_monitor/status
ros2 topic echo /nav2_monitor/fault_event
ros2 topic echo /safety_system/cmd
ros2 topic echo /supervisor/cmd
```

统一反馈测试示例：

```bash
ros2 topic pub /nav2_monitor/algorithm_feedback nav2_monitor/msg/AlgorithmFeedback \
  "{module_name: navigation, topic_name: /controller/feedback, metric_name: tracking_error, value: 0.95, valid: true}"
```

## 测试

```bash
colcon test --packages-select nav2_monitor --event-handlers console_direct+
```

当前该包核心测试覆盖：

- 节点掉线
- `watch_topics` 频率故障
- `feedback_rules` 越界 / 缺失 / stale / invalid
- 底盘异常与久停
- 故障触发 / 恢复边沿
- 安全状态机仲裁
- 碰撞检测 zone / slowdown / TTC / pointcloud

## 故障排查

### feedback 规则不生效

检查：

- 是否真的有桥接节点在发布 `/nav2_monitor/algorithm_feedback`
- `module_name + topic_name + metric_name` 是否与 `feedback_rules.source_topic + metric_name` 一致
- `fault_config` 路径是否正确

### 底盘判断不生效

检查：

- `chassis_stationary.enabled=1`
- `/command` 是否包含 `speed` 字段
- `/moto_info` 的实际数据结构是否与当前解码假设一致

### 碰撞检测不生效

检查：

- `collision_detection.enabled=1`
- `scan_topic` / `pointcloud_topic` 是否有数据
- 输入 frame 是否能正确变换到 `base_frame_id`
- `zones.points` 是否围住了真实危险区域
- `min_points` 是否过高

## 当前缺项

- `Range / Ultrasonic` 作为碰撞源
- 完整 footprint 前向离散仿真版 TTC
- 多错误组合策略表配置化
- 更彻底的状态组装抽象

## 许可证

Apache-2.0
