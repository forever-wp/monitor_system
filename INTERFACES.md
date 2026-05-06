# Worktree External Interfaces

本文件汇总当前 worktree 中 3 个对外接口面：

- `bridge`
- `nav2_monitor`
- `safety_emergency_executor`

不包含 `master_interfaces` 协议包说明。

## 1. 总体链路

当前对外主链路如下：

`业务 topic -> bridge -> /nav2_monitor/algorithm_feedback -> nav2_monitor -> /safety_system/cmd -> safety_emergency_executor -> /command`

补充链路：

- `nav2_monitor` 同时对外发布状态、故障事件、JSON 上报、碰撞区可视化
- `safety_emergency_executor` 同时接收多路速度源，并通过标准 ROS 2 参数服务切换当前控制源

## 2. Bridge

### 2.1 角色

`bridge` 负责把业务侧原始 ROS 消息统一转换成 `nav2_monitor/msg/AlgorithmFeedback`。

当前有两个对外节点：

- `bridge_py_node`
  - 主实现
  - 多 topic
  - 配置驱动
- `bridge_cpp_node`
  - 示例实现
  - 强类型单桥接模板

### 2.2 启动与配置入口

- 包文档：
  - `src/bridge/README.md`
- Python 节点参数：
  - `src/bridge/config/bridge_py_params.yaml`
- C++ 节点参数：
  - `src/bridge/config/bridge_cpp_params.yaml`
- Python 规格示例：
  - `src/bridge/config/examples/generic_multi_bridge_spec.yaml`

### 2.3 对外接口

#### `bridge_py_node`

节点名：

- `bridge_py_node`

参数：

- `spec_file`
  - 默认值：`config/examples/generic_multi_bridge_spec.yaml`
  - 作用：指定桥接规则 YAML

接口特征：

- 无自定义 service
- 无 action
- 输入 topic 和输出 topic 完全由 `spec_file` 决定

单条桥接规则必须包含：

- `id`
- `message_type`
- `input_topic`
- `output_topic`
- `module_name`
- `topic_name`
- `metrics`

每个 `metrics` 条目必须包含：

- `name`
- `field`

可选字段：

- `valid_field`
- `valid_default`

#### `bridge_cpp_node`

节点名：

- `bridge_cpp_node`

参数：

- `input_topic`
- `output_topic`
- `module_name`
- `topic_name`

默认示例：

- 输入：`/battery_state`
- 输出：`/nav2_monitor/algorithm_feedback`

### 2.4 输出消息

统一输出消息类型：

- `nav2_monitor/msg/AlgorithmFeedback`

字段：

- `stamp`
- `module_name`
- `topic_name`
- `metric_name`
- `value`
- `valid`

含义：

- `module_name`：反馈所属模块
- `topic_name`：原始业务 topic 名
- `metric_name`：单个指标名
- `value`：数值
- `valid`：该指标本次是否有效

### 2.5 示例桥接

当前示例桥接：

- 输入 topic：`/battery_state`
- 输出 topic：`/nav2_monitor/algorithm_feedback`
- 输出指标：
  - `battery_percentage`
  - `battery_temperature`
  - `battery_voltage`

### 2.6 示例

#### bridge 规则 YAML 示例

```yaml
bridges:
  - id: battery
    message_type: "sensor_msgs/msg/BatteryState"
    input_topic: "/battery_state"
    output_topic: "/nav2_monitor/algorithm_feedback"
    module_name: "battery_node"
    topic_name: "/battery_state"
    metrics:
      - name: "battery_percentage"
        field: "percentage"
        valid_field: "present"
      - name: "battery_temperature"
        field: "temperature"
        valid_field: "present"
```

#### bridge 输出示例

当 `/battery_state` 来了一帧：

```text
percentage: 0.86
temperature: 28.5
present: true
```

桥接后会发布两条 `AlgorithmFeedback`，效果类似：

```yaml
stamp:
  sec: 1710000000
  nanosec: 0
module_name: battery_node
topic_name: /battery_state
metric_name: battery_percentage
value: 0.86
valid: true
```

```yaml
stamp:
  sec: 1710000000
  nanosec: 0
module_name: battery_node
topic_name: /battery_state
metric_name: battery_temperature
value: 28.5
valid: true
```

## 3. nav2_monitor

### 3.1 角色

`nav2_monitor` 是轻量监控、故障检测与安全联动节点。

它负责：

- 收集运行状态
- 监控节点、topic、TF
- 消费统一反馈
- 进行故障检测与仲裁
- 发布故障事件
- 下发安全动作
- 发布上报 JSON
- 发布碰撞区域可视化

### 3.2 启动与配置入口

- 启动文件：
  - `src/nav2_monitor/launch/nav2_monitor.launch.py`
- 主参数文件：
  - `src/nav2_monitor/config/nav2_monitor_params.yaml`
- 功能文档：
  - `src/nav2_monitor/README.md`
- 架构文档：
  - `src/nav2_monitor/docs/architecture.md`
  - `src/nav2_monitor/docs/architecture_diagrams.md`

节点名：

- `nav2_monitor`

### 3.3 固定输入接口

固定订阅输入：

| Topic | Type | 说明 |
|---|---|---|
| `/nav2_monitor/algorithm_feedback` | `nav2_monitor/msg/AlgorithmFeedback` | 来自 bridge 的统一指标反馈 |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | 电池状态 |
| `/task_status_code` | `master_interfaces/msg/TaskStatus` | 外层任务状态码输入，用于任务配置切换 |

### 3.4 动态输入接口

以下输入由 `fault_config` 或运行时监控目标决定：

| 接口类型 | 来源 | 说明 |
|---|---|---|
| `watch_topics` | `fault_detector_config.yaml` 或 fallback 参数 | 直接监控的真实 ROS topic |
| `command_topic` | `collision/chassis config` | 被监控的最终控制命令，一般为 `/command` |
| `moto_topic` | `collision/chassis config` | 电机反馈 topic |
| `odom_topic` | `collision/chassis config` | 底盘/定位里程计 |
| `imu_topic` | `collision/chassis config` | IMU 真值输入 |
| `prediction_speed_topic` | 碰撞检测配置 | TTC / approach 预测速度 |
| `scan_topic` | 碰撞检测配置 | `sensor_msgs/msg/LaserScan` |
| `pointcloud_topic` | 碰撞检测配置 | `sensor_msgs/msg/PointCloud2` |
| `ultrasonic_topic` | 碰撞检测配置 | `std_msgs/msg/String`，8 路超声波 JSON |

说明：

- `watch_topics` 会被节点动态发现并按 topic 实际类型建立订阅
- 这些动态输入是否启用取决于当前 `fault_config`

### 3.5 对外输出接口

| Topic | Type | 说明 |
|---|---|---|
| `/nav2_monitor/status` | `nav2_monitor/msg/MonitorStatus` | 周期状态总览 |
| `/nav2_monitor/fault_event` | `nav2_monitor/msg/FaultEvent` | 故障触发/恢复边沿事件 |
| `/supervisor/cmd` | `std_msgs/msg/String` | supervisor 命令 JSON |
| `/safety_system/cmd` | `nav2_monitor/msg/SafetyCmd` | 安全执行命令 |
| `/nav2_monitor/reporter/heartbeat_json` | `std_msgs/msg/String` | 心跳 JSON |
| `/nav2_monitor/reporter/event_json` | `std_msgs/msg/String` | 事件 JSON |
| `collision_detection.zones[*].polygon_pub_topic` | `geometry_msgs/msg/PolygonStamped` | 碰撞区可视化 |

说明：

- `/nav2_monitor/status` 是固定 topic，目前不是参数化输出
- `/supervisor/cmd` 当前真实输出是 JSON 字符串，不是 `SupervisorCmd.msg`
- 碰撞区可视化 topic 名由 zone 配置动态决定

### 3.6 输出消息定义

#### `nav2_monitor/msg/MonitorStatus`

字段分组：

- 总状态：
  - `all_ok`
- 节点监控：
  - `monitored_nodes`
  - `active_nodes`
  - `timeout_nodes`
- topic 监控：
  - `monitored_topics`
  - `active_topics`
  - `inactive_topics`
  - `topic_frequencies`
- 系统资源：
  - `cpu_usage`
  - `mem_usage`
  - `disk_usage`
  - `cpu_temp`
  - `gpu_usage`
  - `gpu_temp`
  - `gpu_mem_usage`
- 车辆导航状态：
  - `vehicle_status_valid`
  - `vehicle_navigation_active`
  - `vehicle_navigation_succeeded`
  - `vehicle_progress_percentage`
  - `vehicle_simple_status`
  - `vehicle_error_message`
- 电池：
  - `battery_temperature`
  - `battery_percentage`
- TF：
  - `monitored_transforms`
  - `available_transforms`
  - `stale_transforms`
  - `transform_latencies_ms`
- 预留：
  - `topic_latencies_ms`

#### `nav2_monitor/msg/FaultEvent`

字段：

- `stamp`
- `module_name`
- `fault_level`
- `action`
- `edge`
- `reason`

常量：

- `fault_level`
  - `NORMAL=0`
  - `WARNING=1`
  - `ERROR=2`
  - `CRITICAL=3`
- `action`
  - `NONE=0`
  - `SUPERVISOR=1`
  - `SAFETY_SYSTEM=2`
- `edge`
  - `EDGE_NONE=0`
  - `EDGE_TRIGGER=1`
  - `EDGE_RECOVER=2`

#### `nav2_monitor/msg/SafetyCmd`

字段：

- `action`
- `slow_down_percentage`
- `reason`

常量：

- `SLOW_DOWN=1`
- `SOFT_STOP=2`
- `EMERGENCY_STOP=3`
- `RESUME=4`

### 3.7 JSON 输出协议

#### `/supervisor/cmd`

类型：

- `std_msgs/msg/String`

JSON 结构：

```json
{
  "module_name": "navigation",
  "nodes_to_restart": [],
  "reason": "Node inactive"
}
```

字段：

- `module_name`
- `nodes_to_restart`
- `reason`

#### `/nav2_monitor/reporter/heartbeat_json`

类型：

- `std_msgs/msg/String`

JSON 顶层字段：

- `timestamp`
- `all_ok`
- `system`
- `battery`
- `navigation`
- `summary`

`system` 包含：

- `cpu_usage`
- `mem_usage`
- `disk_usage`
- `cpu_temp`
- `gpu_usage`
- `gpu_temp`
- `gpu_mem_usage`

`battery` 包含：

- `temperature`
- `percentage`

`navigation` 包含：

- `status_valid`
- `active`
- `succeeded`
- `progress_percentage`
- `simple_status`
- `error_message`

`summary` 包含：

- `active_nodes`
- `timeout_nodes`
- `active_topics`
- `inactive_topics`

#### `/nav2_monitor/reporter/event_json`

类型：

- `std_msgs/msg/String`

JSON 顶层字段：

- `timestamp`
- `edge`
- `fault_type`
- `fault_module`
- `fault_level`
- `fault_message`
- `measure_execution`

`measure_execution` 包含：

- `action_type`
- `supervisor`
- `safety`
- `details`

其中：

- `supervisor.matched`
- `supervisor.module_name`
- `supervisor.nodes_to_restart_count`
- `safety.matched`
- `safety.action`
- `safety.slow_down_percentage`
- `safety.reason`

### 3.8 关键参数接口

常用关键参数：

- `timeout`
- `scan_rate`
- `check_rate`
- `safety_cooldown_s`
- `supervisor_cooldown_s`
- `algorithm_feedback_topic`
- `fault_event_topic`
- `supervisor_cmd_topic`
- `safety_cmd_topic`
- `reporter.heartbeat_json_topic`
- `reporter.event_json_topic`
- `reporter.cmd_correlation_window_s`
- `battery_state_topic`
- `battery_state_timeout_s`
- `fault_config`
- `fault_config_reload_enabled`
- `current_nav_task`
- `task_status_topic`
- `task_status_code_mappings.*`
- `task_fault_configs.*`
- `target_nodes`
- `watch_topics`
- `target_transforms`

### 3.9 运行时可变接口

`nav2_monitor` 支持通过标准 ROS 2 参数机制动态修改以下外部行为：

- `target_nodes`
- `watch_topics`
- `fault_config`
- `current_nav_task`
- `task_status_topic`
- `task_status_code_mappings.*`
- `task_fault_configs.*`
- `fault_config_reload_enabled`
- `target_transforms`
- `timeout`
- `safety_cooldown_s`
- `supervisor_cooldown_s`
- `battery_state_timeout_s`

说明：

- 当监控目标来自 `fault_config modules` 时，`target_nodes` 和 `watch_topics` 更新会被忽略
- `current_nav_task` 更新会触发任务级故障配置切换
- `task_status_topic` 更新会重建订阅

### 3.10 无接口项

当前 `nav2_monitor`：

- 无自定义 service
- 无 action server / action client 对外接口

### 3.11 示例

#### 统一反馈输入示例

向 `nav2_monitor` 注入一条统一反馈：

```bash
ros2 topic pub /nav2_monitor/algorithm_feedback nav2_monitor/msg/AlgorithmFeedback \
  "{module_name: navigation, topic_name: /controller/feedback, metric_name: tracking_error, value: 0.95, valid: true}"
```

#### 监控状态查看示例

```bash
ros2 topic echo /nav2_monitor/status
```

典型输出字段会包含：

```yaml
all_ok: true
active_nodes:
- controller_server
- planner_server
active_topics:
- /battery_state
- /livox/imu
battery_percentage: 0.86
vehicle_navigation_active: true
```

#### 故障事件查看示例

```bash
ros2 topic echo /nav2_monitor/fault_event
```

典型事件：

```yaml
module_name: navigation
fault_level: 2
action: 2
edge: 1
reason: "Node inactive"
```

#### 安全动作输出示例

```bash
ros2 topic echo /safety_system/cmd
```

典型输出：

```yaml
action: 1
slow_down_percentage: 50.0
reason: "Collision risk"
```

#### supervisor JSON 输出示例

```bash
ros2 topic echo /supervisor/cmd
```

典型内容：

```json
{"module_name":"navigation","nodes_to_restart":[],"reason":"Node inactive"}
```

#### 心跳 JSON 输出示例

```bash
ros2 topic echo /nav2_monitor/reporter/heartbeat_json
```

典型内容：

```json
{
  "timestamp":"1710000000.123",
  "all_ok":true,
  "system":{"cpu_usage":12.3,"mem_usage":41.0,"disk_usage":52.1,"cpu_temp":48.0,"gpu_usage":-1,"gpu_temp":-1,"gpu_mem_usage":-1},
  "battery":{"temperature":28.5,"percentage":0.86},
  "navigation":{"status_valid":true,"active":true,"succeeded":false,"progress_percentage":0.42,"simple_status":"RUNNING","error_message":""},
  "summary":{"active_nodes":12,"timeout_nodes":0,"active_topics":18,"inactive_topics":0}
}
```

## 4. safety_emergency_executor

### 4.1 角色

`safety_emergency_executor` 负责：

- 接收安全动作
- 接收多路速度控制源
- 选通当前激活控制源
- 对输出执行减速、缓停、急停、恢复
- 输出底盘 JSON 控制命令

### 4.2 启动与配置入口

- 启动文件：
  - `src/safety_emergency_executor/launch/safety_emergency_executor.launch.py`
- 参数文件：
  - `src/safety_emergency_executor/config/safety_emergency_executor_params.yaml`
- Monitor 目录镜像参数：
  - `config/Monitor/safety_emergency_executor/safety_emergency_executor_params.yaml`
- 包文档：
  - `src/safety_emergency_executor/README.md`

节点名：

- `safety_emergency_executor`

### 4.3 输入接口

| Topic | Type | 说明 |
|---|---|---|
| `/safety_system/cmd` | `nav2_monitor/msg/SafetyCmd` | 安全动作输入 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 导航速度 |
| `/cmd_vel_miniapp` | `geometry_msgs/msg/Twist` | 小程序速度 |
| `/cmd_vel_remote` | `geometry_msgs/msg/Twist` | 远程驾驶速度 |
| `/cmd_vel_other` | `geometry_msgs/msg/Twist` | 其他来源速度 |
| `/pressure_` | `std_msgs/msg/String` | 动态更新 `acc/press/place/ulock` 的 JSON |
| `/acc_` | `std_msgs/msg/Int32` | 动态覆盖 `acc` |
| `/odom_base` | `nav_msgs/msg/Odometry` | 轮速里程计 |
| `/odom` | `nav_msgs/msg/Odometry` | 定位里程计 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | IMU |

说明：

- 同一时间只允许一个速度源进入输出链路
- 非激活源直接丢弃
- 激活源停发时不补零、不保留最后一帧
- 小程序、远程驾驶、其他外部源可选启用 `Twist` 扩展字段复用
  - `linear.x -> speed`
  - `angular.z -> angle`
  - `linear.y -> press`
  - `linear.z -> acc`
  - `angular.x -> place`
  - `angular.y -> ulock`
  - 仅当至少一个辅助轴非零时，才会把辅助轴解释为扩展控制字段；否则继续使用默认参数或 `/pressure_`、`/acc_` 动态值

### 4.4 输出接口

| Topic | Type | 说明 |
|---|---|---|
| `/command` | `std_msgs/msg/String` | 最终输出到底盘的 JSON 命令 |
| `/control_source_state` | `std_msgs/msg/String` | 当前激活控制源状态 |

说明：

- `/control_source_state` 用于被动发布当前模式
- 该 topic 采用 `transient_local + reliable`，后加入订阅者也可拿到最近状态

### 4.5 控制源切换接口

当前不使用自定义 service。

控制源切换方式：

- 使用 ROS 2 标准参数服务设置 `active_control_source`
- 使用 ROS 2 标准参数服务查询 `active_control_source`

可选值：

- `navigation`
- `miniapp`
- `remote`
- `other`

默认值：

- `navigation`

说明：

- `control_source_auto_preempt_enabled` 目前仅为预留开关，自动抢占未启用
- 扩展字段开关默认值：
  - `navigation=false`
  - `miniapp=true`
  - `remote=true`
  - `other=true`

控制源切换示例：

1. 使用 `ros2 param` 查询当前控制源

```bash
ros2 param get /safety_emergency_executor active_control_source
```

示例输出：

```text
String value is: navigation
```

2. 使用 `ros2 param` 切换到 `remote`

```bash
ros2 param set /safety_emergency_executor active_control_source remote
```

示例输出：

```text
Set parameter successful: ok
```

3. 直接调用标准参数服务查询

```bash
ros2 service call /safety_emergency_executor/get_parameters \
  rcl_interfaces/srv/GetParameters \
  "{names: ['active_control_source']}"
```

示例输出：

```yaml
values:
- type: 4
  string_value: remote
```

4. 直接调用标准参数服务切换

```bash
ros2 service call /safety_emergency_executor/set_parameters \
  rcl_interfaces/srv/SetParameters \
  "{parameters: [{name: 'active_control_source', value: {type: 4, string_value: 'miniapp'}}]}"
```

示例输出：

```yaml
results:
- successful: true
  reason: ok
```

### 4.6 `/command` JSON 协议

类型：

- `std_msgs/msg/String`

JSON 字段：

- `speed`
- `angle`
- `acc`
- `press`
- `place`
- `ulock`

示例：

```json
{
  "speed": 0.5,
  "angle": 0.2,
  "acc": 2000,
  "press": 1400,
  "place": -1,
  "ulock": -1
}
```

说明：

- `speed` 由 `Twist.linear.x` 转换
- `angle` 由 `Twist.angular.z` 转换
- `speed` 和 `angle` 按两位小数四舍五入后输出
- `acc`、`press`、`place`、`ulock` 默认来自静态参数或动态更新
- 若当前激活源启用了扩展字段解析，且收到的 `Twist` 至少一个辅助轴非零，则：
  - `press` 取 `linear.y`
  - `acc` 取 `linear.z`
  - `place` 取 `angular.x`
  - `ulock` 取 `angular.y`
  - 此时 `press` 直接按上层输入透传，不再经过自动调压

### 4.7 `/pressure_` JSON 协议

类型：

- `std_msgs/msg/String`

支持字段：

- `acc`
- `press`
- `place`
- `ulock`

示例：

```json
{
  "acc": 1500,
  "press": 950,
  "place": 2,
  "ulock": 3
}
```

### 4.8 安全动作语义

输入消息类型：

- `nav2_monitor/msg/SafetyCmd`

动作语义：

- `SLOW_DOWN`
  - 优先使用消息内 `slow_down_percentage`
  - 未给出时回退到本地参数 `slow_down_percentage`
- `SOFT_STOP`
  - 速度与转角都截断到 `0`
- `EMERGENCY_STOP`
  - 禁止继续透传
  - 直接发布制动序列到 `/command`
- `RESUME`
  - 清除安全限制
  - 恢复正常透传

### 4.9 关键参数接口

关键参数：

- `safety_cmd_topic`
- `command_output_topic`
- `cmd_vel_navigation_topic`
- `cmd_vel_miniapp_topic`
- `cmd_vel_remote_topic`
- `cmd_vel_other_topic`
- `control_source_state_topic`
- `active_control_source`
- `control_source_auto_preempt_enabled`
- `cmd_vel_navigation_extended_fields_enabled`
- `cmd_vel_miniapp_extended_fields_enabled`
- `cmd_vel_remote_extended_fields_enabled`
- `cmd_vel_other_extended_fields_enabled`
- `pressure_update_topic`
- `acc_update_topic`
- `wheel_odom_topic`
- `loc_odom_topic`
- `imu_topic`
- `acc_`
- `press_`
- `place_`
- `ulock_`
- `auto_pressure.*`
- `slow_down_percentage`
- `brake_*`

### 4.10 运行时可变接口

当前支持通过标准参数服务动态切换：

- `active_control_source`

### 4.11 无接口项

当前 `safety_emergency_executor`：

- 无自定义 service
- 无 action server / action client 对外接口

### 4.12 示例

#### 查询当前控制源

```bash
ros2 param get /safety_emergency_executor active_control_source
```

示例输出：

```text
String value is: navigation
```

#### 切换控制源到 remote

```bash
ros2 param set /safety_emergency_executor active_control_source remote
```

示例输出：

```text
Set parameter successful: ok
```

#### 查看当前控制源状态 topic

```bash
ros2 topic echo /control_source_state
```

示例输出：

```text
data: remote
```

#### 下发 remote 速度指令

```bash
ros2 topic pub /cmd_vel_remote geometry_msgs/msg/Twist \
  "{linear: {x: 1.0}, angular: {z: 0.4}}"
```

#### 查看最终 `/command`

```bash
ros2 topic echo /command
```

未叠加安全动作时，典型输出：

```json
{"speed":1.0,"angle":0.4,"acc":2000,"press":1400,"place":-1,"ulock":-1}
```

#### 下发带扩展控制字段的 remote 速度指令

```bash
ros2 topic pub /cmd_vel_remote geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 950.0, z: 1500.0}, angular: {x: 2.0, y: 0.0, z: 0.4}}"
```

典型输出：

```json
{"speed":1.0,"angle":0.4,"acc":1500,"press":950,"place":2,"ulock":0}
```

#### 下发减速安全动作

```bash
ros2 topic pub /safety_system/cmd nav2_monitor/msg/SafetyCmd \
  "{action: 1, slow_down_percentage: 50.0, reason: 'smoke'}"
```

随后再发送同样的 remote 速度，`/command` 典型输出会变成：

```json
{"speed":0.5,"angle":0.2,"acc":2000,"press":1400,"place":-1,"ulock":-1}
```

#### 动态更新底盘参数字段

```bash
ros2 topic pub /pressure_ std_msgs/msg/String \
  '{data: "{\"press\":950,\"place\":2,\"ulock\":3}"}'
```

随后输出示例：

```json
{"speed":0.5,"angle":0.2,"acc":2000,"press":950,"place":2,"ulock":3}
```

## 5. 包间调用关系

### 5.1 bridge -> nav2_monitor

桥接输出：

- `nav2_monitor/msg/AlgorithmFeedback`

关键字段关联：

- `AlgorithmFeedback.topic_name` 对应 `feedback_rules.source_topic`
- `AlgorithmFeedback.metric_name` 对应 `feedback_rules.metric_name`

### 5.2 nav2_monitor -> safety_emergency_executor

安全动作输出：

- `/safety_system/cmd`

消息类型：

- `nav2_monitor/msg/SafetyCmd`

### 5.3 safety_emergency_executor -> chassis

最终底盘输出：

- `/command`

消息类型：

- `std_msgs/msg/String`

载荷：

- 底盘 JSON 控制命令

## 6. 当前接口注意事项

- `nav2_monitor` 中定义了 `SupervisorCmd.msg`，但当前真实对外输出仍是 `/supervisor/cmd` 的 JSON 字符串
- `safety_emergency_executor` 当前控制源切换完全依赖标准参数服务，不提供自定义 service
- `bridge_py_node` 的对外 topic 集合不是固定的，最终以 `spec_file` 为准
- `nav2_monitor` 的动态监控 topic、碰撞输入和碰撞区发布 topic 最终以当前 `fault_config` 为准
