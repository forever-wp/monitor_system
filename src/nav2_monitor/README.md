# nav2_monitor

- 名称：Nav2_Monitor v2.3
- 版本：v2.3
- 时间：2026-03-15
- 作者：ToTo

轻量级 Nav2 监控、故障检测与安全联动系统。

## 设计原则

- 低占用
- 小内存
- 快响应

## 文档入口

- 变更记录：`CHANGELOG.md`
- 设计说明：`src/nav2_monitor/docs/architecture.md`
- 流程图 / 数据流图：`src/nav2_monitor/docs/architecture_diagrams.md`
- 上报器类：`src/nav2_monitor/include/nav2_monitor/monitor_reporter.hpp`

## 功能概览

当前版本已支持：

- 节点在线监控
- `watch_topics` 频率与发布者监控
- `feedback_rules` 统一反馈规则判断
- 底盘异常与久停判断
- 多故障组合与安全动作仲裁
- 自动恢复与 `RESUME`
- 碰撞检测：`LaserScan` / `PointCloud2` / `ultrasonic_eight(JSON)`
- 碰撞策略：`slowdown zone` / `stop zone` / `dynamic ttc`
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

当前 OTA 参数源文件位于 `config/Monitor/nav2_monitor/`，
部署后的运行时路径为 `/opt/ry/config/Monitor/nav2_monitor/`。

## 关键配置

### 主参数

仓库源文件：`config/Monitor/nav2_monitor/nav2_monitor_params.yaml`

运行时路径：`/opt/ry/config/Monitor/nav2_monitor/nav2_monitor_params.yaml`

关键字段：

- `timeout`
- `scan_rate`
- `check_rate`
- `algorithm_feedback_topic`
- `fault_event_topic`
- `supervisor_cmd_topic`
- `safety_cmd_topic`
- `reporter.heartbeat_json_topic`
- `reporter.event_json_topic`
- `battery_state_topic`
- `fault_config`
- `fault_config_reload_enabled`
- `current_nav_task`
- `task_status_topic`
- `task_status_code_mappings.<code>`
- `task_fault_configs.<task_name>`
- `target_transforms`（fallback，仅当 `fault_config` 未配置时生效）

### 故障配置

仓库源文件：`config/Monitor/nav2_monitor/fault_detector_config.yaml`

运行时路径：`/opt/ry/config/Monitor/nav2_monitor/fault_detector_config.yaml`

当前主要配置块：

- `collision_detection`
- `chassis_stationary`
- `modules`

### 配置语义

- `fault_config.target_transforms`
  - 顶层 TF 监控配置，优先级高于 `nav2_monitor_params.yaml` 里的 `target_transforms`
  - 推荐将 TF 监控迁移到 `fault_detector_config.yaml` 统一管理

- `watch_topics`
  - 表示 `nav2_monitor` 直接监控的真实 ROS topic
  - 若配置了 `min_hz`：同时判断发布者与频率
  - 若未配置 `min_hz`：只判断是否有发布者，不做频率阈值判断

- `feedback_rules`
  - 表示统一反馈规则
  - `source_topic` 对应 `AlgorithmFeedback.topic_name`
  - `metric_name` 对应 `AlgorithmFeedback.metric_name`
  - 当前 `light-lm` 已支持 `/drift_status` 的两条独立规则：
    `drift_state` 与 `drift_delta_norm`

- `chassis_stationary.odom_topic`
  - 可选输入
  - 为空时表示不依赖 odom，只根据 `command + moto` 做底盘判断

- `collision_detection.ultrasonic_topic`
  - 单 topic 八路超声波 JSON 输入
- `collision_detection.ultrasonic_widget`
  - 8 个 `0~1` 权重，顺序对应 8 路超声波
  - 当前编号约定：`1号左前，之后顺时针编号`
  - 当前支持从 JSON 中提取 8 路距离数组，并按内置 8 路默认位姿和 `ultrasonic_widget` 权重映射到底盘坐标
- `collision_detection.ultrasonic_blind_distance`
  - 盲区下限，默认 `0.2`；小于该值时按盲区边界处理
- `collision_detection.ultrasonic_out_of_range_value`
  - 超量程值，默认 `1.0`；等于或超过该值时视为“无障碍点”
- `collision_detection.footprint_points`
  - 车体 footprint 多边形，格式与 zone 的 `points` 一致
  - `ttc` 模型要求配置该字段；未配置时该 TTC 规则会被安全跳过
  - 当前 `ttc` 会使用轻量预测轨迹（常速度 / 常角速度离散前推）生成动态 corridor，并基于车体 footprint 计算 TTC
  - 可额外配置 `recover_time_before_collision` 作为退出阈值，以及 `min_hold_time_s` 作为最小保持时间
- `collision_detection.direction_speed_threshold`
  - `zone` 模式前后向判断阈值
  - 当 `|prediction_linear_x|` 小于该值时，保持当前稳定方向；若尚未建立稳定方向，则只匹配 `motion_direction=both`
  - 当预测速度已超时时，方向状态会被清空，重新等待稳定方向建立
- `collision_detection.direction_confirm_count`
  - 前后向切换确认帧数
  - 连续收到相反方向速度达到该次数后，`zone/ttc` 才会切换到新的方向，避免单帧跳变导致方向抖动
- `collision_detection.ttc_visualization_enabled`
  - TTC 预测可视化总开关
  - 打开后发布 `/nav2_monitor/collision_ttc_markers`
- `collision_detection.zones`
  - `motion_direction` 支持：
    - `forward`
    - `reverse`
    - `both`
  - `model` 支持：
    - 默认 zone 命中
    - `ttc`
  - `actions` 支持：
    - `safety_system`
    - `supervisor`
  - `min_points` 对 `scan/pointcloud` 表示点数阈值，对超声波表示加权后的有效点阈值
  - `safety_system` 支持：
    - `0` 不执行
    - `1` 减速
    - `2` 缓停
    - `3` 急停

> 旧字段 `topics`、`feedback_topics`、`topic_name` 已不再接受。

## 发布 Topic 总表

下表按“当前系统涉及的功能模块”整理所有**对外发布**的 topic，包含：`nav2_monitor`、包内 `MonitorReporter`、`safety_emergency_executor`。其中 `/nav2_monitor/fault_event`、`/supervisor/cmd`、`/safety_system/cmd` 以及 reporter JSON topic 均已支持通过参数修改。

| 名称 | 类型 | 作用 | 方式 | 示例 |
|---|---|---|---|---|
| `/nav2_monitor/status` | `nav2_monitor/msg/MonitorStatus` | 周期发布整体监控状态 | 周期发布（`check_rate`） | `all_ok=true`, `cpu_usage=12.3`, `battery_percentage=0.86` |
| `/nav2_monitor/fault_event` | `nav2_monitor/msg/FaultEvent` | 发布故障触发/恢复边沿事件 | 边沿发布（触发 / 恢复） | `module_name=navigation`, `fault_level=ERROR`, `edge=TRIGGER` |
| `/supervisor/cmd` | `std_msgs/msg/String` | 向 supervisor 下发重启命令 | 故障触发后按 cooldown 发布 | `{"module_name":"navigation","nodes_to_restart":[],"reason":"Node inactive"}` |
| `/safety_system/cmd` | `nav2_monitor/msg/SafetyCmd` | 向安全执行链路下发动作 | 安全状态变化时发布 | `action=2`, `slow_down_percentage=0.0`, `reason="Node inactive"` |
| `/nav2_monitor/reporter/heartbeat_json` | `std_msgs/msg/String` | 发布系统心跳 JSON，上报系统资源/电池/导航状态 | 周期发布（随 `/nav2_monitor/status`） | `{"all_ok":true,"system":{"cpu_usage":12.3},"battery":{"percentage":0.86},"navigation":{"active":true}}` |
| `/nav2_monitor/reporter/event_json` | `std_msgs/msg/String` | 发布异常/恢复事件 JSON | 边沿发布（随 `/nav2_monitor/fault_event`） | `{"edge":"TRIGGER","fault_type":"node_inactive","fault_module":"navigation","fault_level":"CRITICAL"}` |
| `collision_detection.zones[*].polygon_pub_topic` | `geometry_msgs/msg/PolygonStamped` | 发布碰撞区可视化轮廓 | 周期发布（随 `check_health()`） | `/nav2_monitor/collision_zone/front_stop` |
| `/command` | `std_msgs/msg/String` | 执行器输出到底盘协议命令 | 动作触发后透传/限速/制动发布 | `{"speed":0.000,"angle":0.0,"press":1000,"acc":1000,"place":-1,"ulock":-1}` |

**说明**

- `/nav2_monitor/reporter/heartbeat_json` 至少包含：系统信息、电池电量、小车导航状态信息。
- `/nav2_monitor/reporter/event_json` 至少包含：错误类型、错误模块、错误等级、错误信息、措施执行；恢复事件同样上报。
- `/command` 由 `safety_emergency_executor` 发布，不是 `nav2_monitor` 主节点直接发布。
- `collision_detection.zones[*].polygon_pub_topic` 是按配置动态创建的 topic，不是固定单一名称。
- `model: "ttc"` 不再发布静态 polygon，可通过 `/nav2_monitor/collision_ttc_markers` 查看动态 corridor。

## JSON 字段说明

### `/supervisor/cmd` JSON 字段

| 字段 | 类型 | 说明 |
|---|---|---|
| `module_name` | string | 触发 supervisor 重启的模块名 |
| `nodes_to_restart` | array | 预留字段，当前通常为空数组 |
| `reason` | string | 触发该次重启的原因说明 |

### `/nav2_monitor/reporter/heartbeat_json` 字段

| 字段 | 类型 | 说明 |
|---|---|---|
| `timestamp` | string | 上报时刻 |
| `all_ok` | bool | 当前总体状态 |
| `system.cpu_usage` | number | CPU 使用率 |
| `system.mem_usage` | number | 内存使用率 |
| `system.disk_usage` | number | 磁盘使用率 |
| `system.cpu_temp` | number | CPU 温度 |
| `system.gpu_usage` | number | GPU 使用率，无 GPU 时可能为 -1 |
| `system.gpu_temp` | number | GPU 温度，无 GPU 时可能为 -1 |
| `system.gpu_mem_usage` | number | GPU 显存使用率，无 GPU 时可能为 -1 |
| `battery.temperature` | number | 电池温度 |
| `battery.percentage` | number | 电池电量百分比 |
| `navigation.status_valid` | bool | 导航状态文件是否有效 |
| `navigation.active` | bool | 导航是否激活 |
| `navigation.succeeded` | bool | 导航是否成功 |
| `navigation.progress_percentage` | number | 导航进度 |
| `navigation.simple_status` | string | 简化导航状态 |
| `navigation.error_message` | string | 导航错误信息 |
| `summary.active_nodes` | number | 当前活跃节点数 |
| `summary.timeout_nodes` | number | 当前超时节点数 |
| `summary.active_topics` | number | 当前活跃监控 topic 数 |
| `summary.inactive_topics` | number | 当前失活监控 topic 数 |

### `/nav2_monitor/reporter/event_json` 字段

| 字段 | 类型 | 说明 |
|---|---|---|
| `timestamp` | string | 故障事件时间戳 |
| `edge` | string | 事件边沿，`TRIGGER` 或 `RECOVER` |
| `fault_type` | string | 错误类型分类，如 `node_inactive` / `feedback_rule` / `collision_detection` |
| `fault_module` | string | 错误所属模块 |
| `fault_level` | string | 错误等级，`NORMAL/WARNING/ERROR/CRITICAL` |
| `fault_message` | string | 详细错误信息 |
| `measure_execution.action_type` | string | 措施类型，`SUPERVISOR / SAFETY_SYSTEM / NONE` |
| `measure_execution.supervisor.matched` | bool | 是否关联到了具体 supervisor 执行 |
| `measure_execution.supervisor.module_name` | string | supervisor 关联模块名，未关联时为空字符串 |
| `measure_execution.supervisor.nodes_to_restart_count` | number | supervisor 关联节点数量，未关联时为 0 |
| `measure_execution.safety.matched` | bool | 是否关联到了具体 safety 执行 |
| `measure_execution.safety.action` | string | safety 动作，未关联时为 `NONE` |
| `measure_execution.safety.slow_down_percentage` | number | safety 减速百分比，未关联时为 0 |
| `measure_execution.safety.reason` | string | safety 触发原因，未关联时为空字符串 |
| `measure_execution.details` | string | 关联状态说明：`matched_by_correlation` / `placeholder_only` |

### `/safety_system/cmd` 字段

| 字段 | 类型 | 说明 |
|---|---|---|
| `action` | uint8 | 动作类型：`1=SLOW_DOWN`，`2=SOFT_STOP`，`3=EMERGENCY_STOP`，`4=RESUME` |
| `slow_down_percentage` | float32 | 减速百分比，仅 `action=1` 时有意义 |
| `reason` | string | 动作触发原因 |

### `/nav2_monitor/fault_event` 字段

| 字段 | 类型 | 说明 |
|---|---|---|
| `stamp` | builtin_interfaces/Time | 故障事件时间戳 |
| `module_name` | string | 故障所属模块名 |
| `fault_level` | uint8 | 故障等级：`0=NORMAL`，`1=WARNING`，`2=ERROR`，`3=CRITICAL` |
| `action` | uint8 | 对应动作：`0=NONE`，`1=SUPERVISOR`，`2=SAFETY_SYSTEM` |
| `edge` | uint8 | 边沿类型：`0=NONE`，`1=TRIGGER`，`2=RECOVER` |
| `reason` | string | 故障或恢复原因 |

### `/nav2_monitor/status` 字段

| 字段 | 类型 | 说明 |
|---|---|---|
| `all_ok` | bool | 当前总体是否正常 |
| `monitored_nodes` | string[] | 监控节点列表 |
| `active_nodes` | string[] | 当前活跃节点 |
| `timeout_nodes` | string[] | 当前超时节点 |
| `monitored_topics` | string[] | 当前监控 topic 列表 |
| `active_topics` | string[] | 当前活跃 topic |
| `inactive_topics` | string[] | 当前失活 topic |
| `topic_frequencies` | float32[] | 与 `monitored_topics` 对齐的频率数组 |
| `cpu_usage` | float32 | CPU 使用率 |
| `mem_usage` | float32 | 内存使用率 |
| `disk_usage` | float32 | 磁盘使用率 |
| `cpu_temp` | float32 | CPU 温度 |
| `gpu_usage` | float32 | GPU 使用率 |
| `gpu_temp` | float32 | GPU 温度 |
| `gpu_mem_usage` | float32 | GPU 显存使用率 |
| `vehicle_status_valid` | bool | 导航状态是否有效 |
| `vehicle_navigation_active` | bool | 导航是否激活 |
| `vehicle_navigation_succeeded` | bool | 导航是否成功 |
| `vehicle_progress_percentage` | float32 | 导航进度 |
| `vehicle_simple_status` | string | 简化导航状态 |
| `vehicle_error_message` | string | 导航错误信息 |
| `battery_temperature` | float32 | 电池温度 |
| `battery_percentage` | float32 | 电池电量百分比 |
| `monitored_transforms` | string[] | 监控的 TF 列表 |
| `available_transforms` | string[] | 当前可用 TF |
| `stale_transforms` | string[] | 当前超时 TF |
| `transform_latencies_ms` | float32[] | TF 延迟 |
| `topic_latencies_ms` | float32[] | 预留字段，当前未重点使用 |

## 输入 / 输出

### 输入

- `/nav2_monitor/algorithm_feedback`
- `/command`
- `/moto_info`
- `/odom`
- `/battery_state`
- `collision_detection.scan_topic`
- `collision_detection.pointcloud_topic`
- `collision_detection.ultrasonic_topic`

### 输出

- `/nav2_monitor/status`
- `/nav2_monitor/fault_event`
- `/supervisor/cmd`（JSON 字符串）
  - 示例：`{"module_name":"navigation","nodes_to_restart":[],"reason":"Node inactive"}`
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
- `ultrasonic_eight(JSON)`
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
- `approach / TTC` 已支持 footprint clearance、轻量预测轨迹方向、恢复滞回和最小保持时间
- `zone` / `ttc` 已支持按当前稳定运动方向筛选前向/后向区域
- 当速度落入 `direction_speed_threshold` 以内时，会保持当前稳定方向；只有连续 `direction_confirm_count` 帧收到相反方向速度后才切换
- 打开 `collision_detection.ttc_visualization_enabled` 后，可在 RViz2 查看 TTC 预测轨迹、预测 footprint、最近碰撞点和 TTC 文本

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

## IMU 频率复现实验

已提供脚本：`src/nav2_monitor/scripts/imu_frequency_repro.py`

用途：

- 自动启动 `nav2_monitor`
- 自动回放指定 rosbag 中的 IMU topic
- 对比实际接收频率与 `/nav2_monitor/status` 中上报的频率

示例：

```bash
python3 src/nav2_monitor/scripts/imu_frequency_repro.py   /home/tokou/claude/rosbag2_2026_03_12-20_15_08/rosbag2_2026_03_12-20_15_08   --topic /livox/imu
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
- footprint TTC / trajectory TTC / recover hysteresis / min_hold_time

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
- `scan_topic` / `pointcloud_topic` / `ultrasonic_topic` 是否有数据
- 输入 frame 是否能正确变换到 `base_frame_id`
- `zones.points` 是否围住了真实危险区域
- `min_points` 是否过高

## 任务驱动配置切换

- `current_nav_task` 用于表示当前导航任务，例如 `default / todoor / elevator / reverse`。
- `/task_status_code` 是外层任务状态输入，消息类型为 `master_interfaces/msg/TaskStatus`，节点会读取其中的 `status_code` 映射为内部任务名，再同步更新 `current_nav_task`。
- `task_status_code_mappings.<code>` 在参数文件中维护状态码到内部任务名的映射，例如 `200~209 -> elevator`、`300~304 -> todoor`。
- `task_fault_configs.<task_name>` 在参数文件中维护任务到安全配置文件的映射。
- 当 `current_nav_task` 变化时，`nav2_monitor` 会自动选择对应 `fault_config` 并复用现有热更新链路完成切换。
- 未知状态码会被忽略并保持当前任务不变；未命中任务时，回退到 `task_fault_configs.default`；若未设置，则回退到 `fault_config`。

## 漂移状态接入

- `/drift_status` 通过 `bridge_py_node` 转换为 `AlgorithmFeedback`
- `module_name` 固定为 `light-lm`
- 当前输出三个指标：
  - `drift_state`：`pose.position.x`，`1.0=漂移`，`2.0=正常`
  - `drift_delta_norm`：`pose.position.y`，表示位移差模长
  - `drift_reserved`：`pose.position.z`，保留字段
- `light-lm` 下已提供两条独立 `feedback_rules`
  - `drift_state`
  - `drift_delta_norm`
- 当前默认阈值：
  - `drift_state` 使用区间判断，`1.0` 触发异常，`2.0` 为正常
  - `drift_delta_norm > 10.0` 触发异常

## 动态配置更新

- `fault_config_reload_enabled=true` 时，节点会在现有定时器周期内轮询 `fault_detector_config.yaml` 的修改时间。
- 当文件内容或路径发生变化时，会自动重载 `FaultDetector` 配置，并重建受影响的监控目标、底盘订阅和碰撞输入。
- 不新增额外线程，保持低占用 / 小内存 / 快响应。

## 任务配置模板

- 默认配置：`config/Monitor/nav2_monitor/fault_detector_config.yaml`
- 到门任务：`config/Monitor/nav2_monitor/profiles/fault_detector_todoor.yaml`
- 电梯任务：`config/Monitor/nav2_monitor/profiles/fault_detector_elevator.yaml`
- 倒车任务：`config/Monitor/nav2_monitor/profiles/fault_detector_reverse.yaml`

当前模板差异：
- `todoor`：更早前向减速，更敏感前视/前侧超声波（1号左前，顺时针编号）
- `elevator`：更依赖超声波，点云关闭，全向更保守
- `reverse`：切到后向碰撞区，后向超声波权重最高

## 当前缺项

- 多超声波更复杂的场景策略（当前已支持单 topic 八路加权输入，推荐直接配 `ultrasonic_widget`）
- 基于 TTC 结果的控制器切换策略配置化（检测侧已支持进入/退出阈值、footprint、trajectory、min_hold_time）
- 多错误组合策略表配置化
- 更彻底的状态组装抽象

## 许可证

Apache-2.0
