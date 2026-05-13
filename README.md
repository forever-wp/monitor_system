# monitor_system

机器人运行监控、算法反馈适配、碰撞体素融合与安全执行工作区。

本项目不直接负责 MQTT 或云端通信。云端上报由外部模块订阅本项目输出 topic 后实现。本项目负责把机器人本机运行状态、算法反馈、碰撞输入和底盘执行链路组织成可判断、可仲裁、可执行的安全监控系统。

## 文档入口

- [文档索引主界面](index.html)：项目文档总入口，支持分类浏览和搜索。
- [项目架构与数据链路](docs/project_architecture.html)：项目级总览、数据链路、功能图、流程图。
- [监控系统模块独立化与可靠数据联通设计方案](docs/monitor_modular_isolation_design.html)：面向“模块互不影响、数据可靠联通、CPU 满载可降级”的后续架构设计。
- [事件法典仲裁与事件执行层整合计划](docs/event_codex_executor_integration_plan.html)：事件发现器、法典仲裁、事件执行层、上报器的完整实施蓝图。
- [多错误组合策略仲裁层设计页](docs/combined_fault_policy_arbiter_design.html)：组合故障的流程图、数据流、交叉策略和恢复逻辑。
- [接口说明](INTERFACES.html)：topic、message、参数和包间接口。
- [algorithm_feedback_adapter](src/algorithm_feedback_adapter/README.md)：算法 topic 适配为 `AlgorithmFeedback`。
- [nav2_monitor](src/nav2_monitor/README.md)：状态采集、故障判断、事件上报、安全仲裁。
- [collision_voxel_layer](src/collision_voxel_layer/README.md)：scan/depth cloud 融合为体素障碍输入。
- [safety_emergency_executor](src/safety_emergency_executor/README.md)：安全命令执行、速度源选择、底盘 `/command` 输出。

文档规范：`README.md/Readme.md` 保留为入口说明；其他设计、说明、接口、配置类文档统一使用 HTML。README 可互相引用，设计/说明类入口统一指向 HTML 页面。

## 功能边界

| 模块 | 主要职责 | 不负责 |
|---|---|---|
| `algorithm_feedback_adapter` | 订阅算法/业务 topic，拆字段，发布 `/nav2_monitor/algorithm_feedback` | 故障判断、MQTT 上报 |
| `collision_voxel_layer` | 融合 scan/depth cloud，发布 `VoxelGrid` | 安全动作、导航控制器切换 |
| `nav2_monitor` | 数据源检测、规则判断、故障事件、人工介入提醒、安全仲裁 | 直接控制底盘 |
| `safety_emergency_executor` | 执行减速、缓停、急停、恢复，输出 `/command` | 故障判断 |
| `master_interfaces` | 外部接口消息和服务定义 | 业务逻辑 |

## 主数据链路

```text
算法/业务 topic
  -> algorithm_feedback_adapter
  -> /nav2_monitor/algorithm_feedback
  -> nav2_monitor feedback_rules
  -> FaultInfo
  -> FaultStateCoordinator
  -> /safety_system/cmd
  -> safety_emergency_executor
  -> /command
  -> 底盘
```

碰撞链路：

```text
/scan + depth PointCloud2
  -> collision_voxel_layer
  -> /collision_voxel_layer/voxels
  -> nav2_monitor collision_detection
  -> zone / TTC
  -> /safety_system/cmd + /navigation_mode
```

小车状态检测链路：

```text
/command + /livox/imu + /odom + /moto_info
  -> nav2_monitor vehicle_state_judge
  -> /nav2_monitor/fault_event
  -> /nav2_monitor/human_intervention
```

## 常用构建与测试

所有 ROS 相关命令建议显式声明 `ROS_DOMAIN_ID=66`。

```bash
env ROS_DOMAIN_ID=66 colcon build --packages-select algorithm_feedback_adapter nav2_monitor collision_voxel_layer safety_emergency_executor
```

```bash
env ROS_DOMAIN_ID=66 ROS_LOG_DIR=/tmp/ros_logs colcon test --packages-select algorithm_feedback_adapter nav2_monitor collision_voxel_layer safety_emergency_executor --event-handlers console_direct+
```

## 配置约定

仓库内运行配置镜像位于：

```text
config/Monitor/
```

实机运行时配置统一使用：

```text
/opt/ry/config/Monitor/
```

关键配置目录：

```text
config/Monitor/algorithm_feedback_adapter/
config/Monitor/nav2_monitor/
config/Monitor/collision_voxel_layer/
config/Monitor/safety_emergency_executor/
```
