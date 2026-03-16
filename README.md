# monitor_system

`monitor_system` 是一个围绕机器人导航安全链路整理出来的功能仓库，当前核心包含两个 ROS2 功能包：

- `src/nav2_monitor`：负责监控、故障检测、事件上报和安全动作决策
- `src/safety_emergency_executor`：负责接收安全动作并把控制链路最终执行到 `/command`

## 功能概览

### `nav2_monitor`

- 节点在线监控
- `watch_topics` 发布者与频率监控
- `feedback_rules` 阈值、stale、invalid 判定
- 底盘异常与久停检测
- `LaserScan` / `PointCloud2` / 超声波碰撞检测
- 安全动作仲裁：`SLOW_DOWN / SOFT_STOP / EMERGENCY_STOP / RESUME`
- 事件 JSON 与心跳 JSON 上报
- 任务配置切换：`default / todoor / elevator / reverse`

### `safety_emergency_executor`

- 订阅 `/safety_system/cmd`
- 执行减速、软停、急停、恢复
- 当前主执行链路为：
  `cmd_vel -> VelocityConverter -> PressureAdjuster -> SafetyPolicyExecutor -> /command`
- 节点内部拆分为三类：
  - `VelocityConverter`
  - `PressureAdjuster`
  - `SafetyPolicyExecutor`

## 推荐版本

如果你希望直接拉取后编译，建议优先使用以下 tag：

- `v2.3.2`：`v2.3.x` 的可编译修复版本
- `v2.4.1`：`v2.4` 安全链路重构后的可编译修复版本

以下 tag 保留历史意义，但不建议直接作为编译版本使用：

- `v2.3`
- `v2.3.1`
- `v2.4`

详细版本说明见 `RELEASE_NOTES.md`。

## 仓库结构

```text
monitor_system/
├── src/
│   ├── nav2_monitor/
│   └── safety_emergency_executor/
└── RELEASE_NOTES.md
```

## 快速使用

```bash
colcon build --packages-select nav2_monitor safety_emergency_executor
source install/setup.bash

ros2 launch nav2_monitor nav2_monitor.launch.py
ros2 launch safety_emergency_executor safety_emergency_executor.launch.py
```

## 文档入口

- `src/nav2_monitor/README.md`
- `src/safety_emergency_executor/README.md`
- `RELEASE_NOTES.md`
