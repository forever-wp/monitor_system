# nav2_monitor

轻量级 Nav2 节点健康监控与故障检测系统

## 设计原则

**低占用 + 响应快**

## 功能特点

- **低占用监控** - 节点监控使用 ROS2 Graph API，反馈数据走统一总线
- **节点监控** - 检测 Nav2 节点存活状态
- **话题监控** - 检测话题发布者状态和频率
- **算法反馈监控** - 支持按模块配置反馈 topic 指标阈值判定
- **底盘停留监控** - 基于 `/command`、`/moto_info`、`/odom` 组合判定停留过久/反馈异常
- **系统监控** - CPU、内存、磁盘、温度、电池状态
- **GPU 监控** - GPU 使用率、温度、显存（可选）
- **故障检测** - 模块级健康判断与动作下发（重启/应急）
- **小车状态监控** - 读取导航状态文件，判断行驶状态
- **双通道上报** - 周期状态 `/nav2_monitor/status` + 事件 `/nav2_monitor/fault_event`

## 核心功能详解

### 1. 监控模块 (Nav2MonitorNode)

**功能**：实时监控 ROS2 系统状态

**监控内容**：
- **节点监控**：通过 ROS2 Graph API 检测目标节点是否在线，超时判断
- **话题监控**：
  - 检测话题发布者状态
  - 计算话题发布频率（基于最近10条消息）
  - 检测空消息
  - 动态订阅话题获取实时数据
- **TF 变换监控**：
  - 检查坐标变换可用性
  - 计算变换延迟（基于时间戳）
- **系统资源监控**：
  - CPU 使用率（通过 /proc/stat）
  - 内存使用率（通过 /proc/meminfo）
  - 磁盘使用率（通过 statvfs）
  - CPU 温度（通过 /sys/class/thermal）
  - GPU 监控（通过 NVML 库）

**特点**：
- 零订阅开销：使用 Graph API 而非订阅节点状态话题
- 低频扫描：可配置扫描频率，默认 0.5Hz
- 线程安全：使用 mutex 保护共享数据

### 2. 故障检测模块 (FaultDetector)

**功能**：模块级健康判断与故障分级

**检测逻辑**：
1. **节点存活检测**（优先级最高）
   - 检查模块所属节点是否全部在线
   - 故障级别：CRITICAL
   - 触发措施：supervisor + safety_system

2. **反馈 topic 阈值检测**
   - 按 `feedback_topics` 配置检查指标值区间、有效位、频率、超时
   - 每条规则可独立配置故障级别和动作列表
   - 支持动作顺序（例如：`[safety_system, supervisor]`）
   - 支持连续计数防误报（默认 `2触发/2恢复`）

3. **兼容话题频率检测（legacy）**
   - 检查话题频率是否低于配置阈值
   - 故障级别：ERROR
   - 触发措施：supervisor
   - 支持连续计数防误报（默认 `2触发/2恢复`）

4. **底盘停留原地/反馈异常检测（全局）**
   - 数据源：`/command`（目标速度）、`/moto_info`（双轮速度）、`/odom`（辅助判断）
   - 四种分支：
     - `command有 + moto无`：判定底盘反馈异常/小车可能未动（结合 odom 区分原因文案）
     - `command无 + moto有`：判定底盘反馈异常
     - `command无 + moto无` 且持续超过阈值：提醒停留原地过久
     - `command有 + moto有`：不报此类故障
   - 支持连续计数防误报（默认 `2触发/2恢复`）

> 说明：节点存活判断保持快速路径，不使用连续计数。

**故障分级**：
- `NORMAL` (0) - 正常
- `WARNING` (1) - 警告（预留）
- `ERROR` (2) - 错误
- `CRITICAL` (3) - 严重

**执行措施**：
- `SUPERVISOR` - 发布软重启命令到 `/supervisor/cmd`
- `SAFETY_SYSTEM` - 发布应急措施到 `/safety_system/cmd`

**特点**：
- 模块化配置：支持多模块独立配置
- 规则级动作：反馈规则可独立配置动作列表
- 可扩展：适配多个算法模块统一反馈消息
- 低开销底盘判断：只做阈值/时效判断，不做复杂模型推断

### 3. 小车状态监控 (VehicleStatusMonitor)

**功能**：读取导航状态文件，获取小车行驶状态

**监控数据**：
- `navigation_active` - 导航是否激活
- `navigation_succeeded` - 导航是否成功
- `progress_percentage` - 进度百分比
- `current_waypoint_index` - 当前路点索引
- `total_waypoints` - 总路点数
- `simple_status` - 简单状态
- `error_message` - 错误信息
- `timestamp` - 时间戳

**特点**：
- 轻量级 JSON 解析：无第三方库依赖
- 文件读取：直接读取状态文件，无网络开销
- 容错处理：解析失败返回 invalid 状态

### 4. 系统资源监控 (SystemMonitor)

**功能**：监控系统硬件资源使用情况

**监控指标**：
- **CPU 使用率**：通过 /proc/stat 计算增量
- **内存使用率**：通过 /proc/meminfo 计算
- **磁盘使用率**：通过 statvfs 系统调用
- **CPU 温度**：读取 thermal_zone0
- **GPU 使用率**：通过 NVML API
- **GPU 温度**：通过 NVML API
- **GPU 显存**：通过 NVML API

**特点**：
- 无外部依赖：直接读取系统文件
- GPU 可选：无 GPU 时返回 -1
- 高效计算：增量计算 CPU 使用率

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│              Nav2MonitorNode (主节点)                    │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐ │
│  │ SystemMonitor│  │VehicleStatus │  │FaultDetector │ │
│  │              │  │   Monitor    │  │              │ │
│  └──────────────┘  └──────────────┘  └──────────────┘ │
│         │                 │                  │          │
│         │                 │                  │          │
│         ▼                 ▼                  ▼          │
│  ┌─────────────────────────────────────────────────┐  │
│  │          数据聚合 (内存直接交互)                 │  │
│  └─────────────────────────────────────────────────┘  │
│                          │                              │
└──────────────────────────┼──────────────────────────────┘
                           │
                           ▼
              ┌────────────────────────┐
              │   发布话题              │
              ├────────────────────────┤
              │ /nav2_monitor/status   │
              │ /nav2_monitor/fault_event │
              │ /supervisor/cmd        │
              │ /safety_system/cmd     │
              └────────────────────────┘
```

## 工作流程

### 监控流程
1. **扫描阶段** (scan_topology, 默认 0.5Hz)
   - 扫描 ROS2 节点列表
   - 更新节点最后见到时间
   - 检查话题发布者状态
   - 查询 TF 变换
   - 动态订阅新话题

2. **检查阶段** (check_health, 默认 1Hz)
   - 判断节点超时
   - 统计话题频率
   - 读取系统资源
   - 读取小车状态
   - 发布监控状态消息

### 故障检测流程
1. **数据更新**
   - 监控目标从 `fault_config.modules` 自动汇总（节点+话题）
   - 从监控模块获取节点状态/话题频率
   - 从统一反馈话题 `/nav2_monitor/algorithm_feedback` 获取算法指标
   - 从 `/command`、`/moto_info`、`/odom` 获取底盘停留判定输入

2. **故障判断**
   - 遍历所有配置的模块
   - 检查节点存活（优先级高）
   - 检查反馈 topic 阈值规则
   - 检查 legacy 话题频率（优先级低）
   - 检查底盘停留原地/反馈异常（全局）

3. **措施执行**
   - 根据配置决定执行措施
   - 发布 supervisor 命令（软重启）
   - 发布 safety_system 命令（应急措施）
   - 记录日志

## 依赖

### 系统依赖
```bash
# ROS2 (Humble/Iron/Jazzy)
sudo apt install ros-${ROS_DISTRO}-rclcpp ros-${ROS_DISTRO}-std-msgs

# yaml-cpp
sudo apt install libyaml-cpp-dev

# NVIDIA GPU 支持（可选）
sudo apt install libnvidia-ml-dev
```

### 编译依赖
- ament_cmake
- rclcpp
- std_msgs
- nav_msgs
- tf2_ros
- yaml-cpp
- nvidia-ml (可选)

## 安装

```bash
cd ~/your_workspace/src
# 将 nav2_monitor 放入 src 目录

cd ~/your_workspace
colcon build --packages-select nav2_monitor
source install/setup.bash
```

## 配置

### 监控配置
编辑 `config/nav2_monitor_params.yaml`：

默认监控对象来源为 `fault_config.modules`；`target_nodes/target_topics` 仅作兼容回退。

```yaml
nav2_monitor:
  ros__parameters:
    timeout: 5.0              # 节点超时时间（秒）
    scan_rate: 0.5            # 拓扑扫描频率（Hz）
    check_rate: 1.0           # 健康检查频率（Hz）
    safety_cooldown_s: 2.0    # safety动作冷却时间
    supervisor_cooldown_s: 5.0 # 重启动作冷却时间
    algorithm_feedback_topic: "/nav2_monitor/algorithm_feedback" # 统一反馈话题
    battery_state_topic: "/battery_state" # 电池状态话题
    battery_state_timeout_s: 90.0 # 电池状态有效超时(秒, 适配1分钟/次上报)
    fault_config: "path/to/fault_detector_config.yaml"  # 故障检测配置
    vehicle_status_file: "/home/ry/.ros/navigate_status/navigate_todoor_status.json"  # 小车状态文件
    # 兼容回退：仅在fault_config缺失或无有效modules时使用
    target_nodes:
      - "controller"
      - "planner"
      - "bt_navigator"
    target_topics:
      - "/cmd_vel"
      - "/plan"
```

### 故障检测配置
编辑 `config/fault_detector_config.yaml`：

```yaml
multi_value_judge:
  trigger_count: 2
  recover_count: 2

chassis_stationary:            # 底盘停留原地判断（新增）
  enabled: 1
  module_name: "chassis_stationary"
  command_topic: "/command"
  moto_topic: "/moto_info"
  odom_topic: "/odom"
  source_timeout_s: 1.0
  idle_timeout_s: 30.0
  command_speed_threshold: 0.05
  moto_speed_threshold: 0.05
  odom_speed_threshold: 0.03
  anomaly_level: "ERROR"
  idle_level: "WARNING"
  safety_system: 2            # 0=不执行 1=减速 2=缓停 3=急停
  safety_slow_down_percentage: 50.0
  anomaly_actions: ["supervisor"]
  idle_actions: ["none"]

modules:
  - name: "navigation"        # 模块名
    supervisor: 1             # 启用软重启 (0/1)
    safety_system: 2          # 0=不执行 1=减速 2=缓停 3=急停
    safety_slow_down_percentage: 50.0  # 仅 safety_system=1 时生效
    nodes:                    # 模块所属节点
      - "controller_server"
      - "planner_server"
    topics:                   # 话题频率阈值
      - name: "/cmd_vel"
        min_hz: 5.0
      - name: "/plan"
        min_hz: 1.0
    feedback_topics:          # 算法反馈规则（新增）
      - topic_name: "/controller/feedback"
        metric_name: "tracking_error"
        min_value: 0.0
        max_value: 0.8
        min_hz: 5.0
        max_stale_s: 1.0
        level: "CRITICAL"
        safety_system: 3      # 可选，规则级覆盖：0=不执行 1=减速 2=缓停 3=急停
        safety_slow_down_percentage: 30.0  # 仅 safety_system=1 时生效
        actions: ["safety_system", "supervisor"]
      - topic_name: "/planner/feedback"
        metric_name: "path_quality"
        min_value: 0.3
        max_value: 1.0
        level: "ERROR"
        actions: ["supervisor"]

  - name: "localization"
    supervisor: 1
    safety_system: 0
    nodes:
      - "amcl"
    topics:
      - name: "/amcl_pose"
        min_hz: 10.0
```

## 使用说明

### 启动监控
```bash
ros2 launch nav2_monitor nav2_monitor.launch.py
```

### 查看状态
```bash
# 查看监控状态
ros2 topic echo /nav2_monitor/status

# 查看故障事件
ros2 topic echo /nav2_monitor/fault_event

# 查看日志
ros2 run nav2_monitor nav2_monitor_node
```

### 发布反馈样例（桥接输出）
```bash
ros2 topic pub /nav2_monitor/algorithm_feedback nav2_monitor/msg/AlgorithmFeedback \
  "{module_name: navigation, topic_name: /controller/feedback, metric_name: tracking_error, value: 0.92, valid: true}"
```

## 数据字段说明

### 输出话题

#### `/nav2_monitor/status`
- **类型**: `nav2_monitor/msg/MonitorStatus`
- **频率**: 1Hz (默认)
- **字段**:

| 字段 | 类型 | 说明 |
|------|------|------|
| all_ok | bool | 总体状态 |
| monitored_nodes | string[] | 监控的节点列表 |
| active_nodes | string[] | 活跃的节点 |
| timeout_nodes | string[] | 超时的节点 |
| monitored_topics | string[] | 监控的话题列表 |
| active_topics | string[] | 有发布者的话题 |
| inactive_topics | string[] | 无发布者的话题 |
| topic_frequencies | float32[] | 话题频率(Hz) |
| cpu_usage | float32 | CPU使用率(%) |
| mem_usage | float32 | 内存使用率(%) |
| disk_usage | float32 | 磁盘使用率(%) |
| cpu_temp | float32 | CPU温度(°C) |
| gpu_usage | float32 | GPU使用率(%, -1无GPU) |
| gpu_temp | float32 | GPU温度(°C, -1无GPU) |
| gpu_mem_usage | float32 | GPU显存(%, -1无GPU) |
| vehicle_status_valid | bool | 小车状态文件是否有效 |
| vehicle_navigation_active | bool | 导航是否激活 |
| vehicle_navigation_succeeded | bool | 导航是否成功 |
| vehicle_progress_percentage | float32 | 导航进度 |
| vehicle_simple_status | string | 小车简化状态 |
| vehicle_error_message | string | 小车错误信息 |
| battery_temperature | float32 | 电池温度 |
| battery_percentage | float32 | 电池百分比（0~1） |

#### `/nav2_monitor/fault_event`
- **类型**: `nav2_monitor/msg/FaultEvent`
- **说明**: 故障边沿事件上报，包含模块、级别、动作、边沿和原因
- **字段**:
  - `fault_level`: 故障等级
  - `action`: 对应动作类型
  - `edge`: 边沿标志 (`1=触发`, `2=恢复`)
  - `reason`: 触发边沿时为原始故障原因；恢复边沿时为标准化文案 `RECOVER fault_key=...; previous_reason=...`

#### `/nav2_monitor/algorithm_feedback`（输入）
- **类型**: `nav2_monitor/msg/AlgorithmFeedback`
- **说明**: 算法模块桥接后的统一反馈输入
- **字段**:
  - `module_name`
  - `topic_name`
  - `metric_name`
  - `value`
  - `valid`

#### `/command`（输入）
- **类型**: `std_msgs/msg/String`
- **说明**: JSON 字符串，监控器从中提取 `speed` 字段作为目标速度
- **示例**:
  - `{"acc":500,"angle":0.0,"place":-1,"press":800,"speed":0.0,"ulock":-1}`

#### `/moto_info`（输入）
- **类型**: 通常为 `chassis_drive/msg/MotoInfo`
- **说明**: 使用左右轮速度（`left_speed_rad`/`right_speed_rad`）判断底盘是否有运动反馈
- **备注**:
  - 当前实现为低耦合，使用 generic subscription + 二进制解码，不强依赖 `chassis_drive` 编译依赖
  - 如果现场消息定义与默认假设不一致，需要提供桥接/适配

#### `/battery_state`（输入）
- **类型**: `sensor_msgs/msg/BatteryState`
- **说明**: 系统监测会缓存最新电池数据并写入 `/nav2_monitor/status`
- **默认节奏**: 1分钟/次也可正常工作（`battery_state_timeout_s` 默认 90 秒）

#### `/odom`（输入）
- **类型**: `nav_msgs/msg/Odometry`
- **说明**: 使用线速度模长作为辅助证据，帮助区分“moto反馈异常”与“小车实际在动/未动”

#### `/supervisor/cmd`
- **类型**: `nav2_monitor/msg/SupervisorCmd`
- **说明**: 软重启命令
- **字段**:
  - `module_name`: 模块名称
  - `nodes_to_restart`: 需要重启的节点列表

#### `/safety_system/cmd`
- **类型**: `nav2_monitor/msg/SafetyCmd`
- **说明**: 应急措施命令
- **字段**:
  - `action`: 动作类型 (1=减速, 2=缓停, 3=急停, 4=恢复)
  - `slow_down_percentage`: 减速百分比，仅 `action=1` 时使用
  - `reason`: 原因描述

### 安全动作恢复
- `nav2_monitor` 会聚合当前所有激活的安全故障，统一下发最严格的安全动作
- 当所有安全故障都恢复后，会自动发布一次 `RESUME`
- 因此 `safety_system: 3`（急停）现在也支持自动恢复

### `feedback_topics` 规则级安全动作覆盖
- `actions` 里包含 `"safety_system"` 时，默认继承模块级 `safety_system`
- 可在单条 `feedback_topics` 规则里额外配置 `safety_system: 0/1/2/3`
- 可在单条 `feedback_topics` 规则里额外配置 `safety_slow_down_percentage`
- 这样可以实现“同一模块不同指标触发不同应急措施”



## 测试方法

### 1. 基础功能测试
```bash
# 启动监控节点
ros2 run nav2_monitor nav2_monitor_node

# 新终端查看输出
ros2 topic echo /nav2_monitor/status
```

### 2. 节点监控测试
```bash
# 启动一个测试节点
ros2 run demo_nodes_cpp talker --ros-args -r __node:=controller

# 观察监控日志，应显示节点正常
# 停止节点后，监控应报告超时
```

### 3. 系统监控测试
```bash
# 查看系统监控日志（每10秒输出）
ros2 run nav2_monitor nav2_monitor_node

# 输出示例：
# CPU=25.3 MEM=45.2 DISK=60.1 TEMP=55.0 GPU=30.5
```

### 4. 压力测试
```bash
# 监控大量节点
# 修改 fault_detector_config.yaml 中 modules 的 nodes/topics
# 观察 CPU 和内存占用（应保持在 1% 以下）
```

### 5. 反馈规则测试
```bash
# 启动监控
ros2 run nav2_monitor nav2_monitor_node --ros-args --params-file src/nav2_monitor/config/nav2_monitor_params.yaml -p fault_config:=/home/tokou/claude/ry_work/src/nav2_monitor/config/fault_detector_config.yaml

# 发布异常反馈（应触发fault_event和动作）
ros2 topic pub /nav2_monitor/algorithm_feedback nav2_monitor/msg/AlgorithmFeedback \
  "{module_name: navigation, topic_name: /controller/feedback, metric_name: tracking_error, value: 0.95, valid: true}"
```

### 6. 底盘停留判断测试
```bash
# 发布command有、moto无场景（应触发底盘异常）
ros2 topic pub /command std_msgs/msg/String \
  "{data: '{\"acc\":500,\"angle\":0.0,\"place\":-1,\"press\":800,\"speed\":0.5,\"ulock\":-1}'}"

# odom保持接近0可验证“可能未动”分支
ros2 topic pub /odom nav_msgs/msg/Odometry \
  "{twist: {twist: {linear: {x: 0.0, y: 0.0, z: 0.0}}}}"
```

## 性能指标

- **内存占用**: ~5KB (10节点) / ~20KB (100节点)
- **CPU 占用**: ~0.07% (10节点) / ~0.55% (100节点)
- **监控延迟**: < 2秒（取决于 scan_rate）

## 故障排除

### GPU 监控失败
如果没有 NVIDIA GPU，GPU 相关指标显示 -1.0，这是正常的。

### 节点检测失败
确保目标节点名称正确，使用 `ros2 node list` 查看实际节点名。

### feedback规则不生效
- 确认桥接节点是否在发布 `/nav2_monitor/algorithm_feedback`
- 确认 `module_name + topic_name + metric_name` 与 `feedback_topics` 配置完全一致
- 确认 `fault_config` 使用了绝对路径或正确可访问路径
- 若 `fault_config` 读取失败/格式异常，系统会保留上一份有效配置并继续运行（不会因配置异常崩溃）

### 底盘停留判断不生效
- 确认 `chassis_stationary.enabled=1`
- 确认 `/command` 能解析到 `speed` 字段（JSON 字段名必须是 `speed`）
- 确认 `/moto_info` 真实消息结构与当前解码假设一致；不一致时需加桥接适配

## 当前缺项（记录）

1. **桥接适配节点未内置**
   - 本包只定义统一反馈消息 `AlgorithmFeedback` 与消费逻辑
   - 各算法模块到统一消息的桥接节点需要外部实现

2. **多错误组合策略表未配置化（未来优化点）**
   - 当前组合状态类已支持多错误聚合与最高安全机制选择
   - 但优先级仍是代码内置规则，后续可扩展为策略表/优先级表

3. **反馈规则的动态热更新未实现**
   - 目前需要重载 `fault_config`（重启节点或扩展热加载逻辑）

4. **`/moto_info` 强类型解码未内置**
   - 当前为了降低编译耦合，未直接依赖 `chassis_drive/msg/MotoInfo`
   - 现场若 CDR 布局变化，建议补一层桥接转为统一反馈消息后再接入判断器

## 许可证

Apache-2.0

## 扩展监控功能

### TF 变换监控
- 监控关键坐标系变换的可用性
- 检测变换延迟（从时间戳计算）
- 超时检测（默认5秒）

### 网络延迟监控
- 测量话题消息的传输延迟
- 基于消息时间戳和接收时间计算
- 10个样本的滚动平均值

## 配置示例

```yaml
nav2_monitor:
  ros__parameters:
    target_transforms:
      - "map->odom"
      - "odom->base_link"
```

## 新增数据字段

| 字段 | 类型 | 说明 |
|------|------|------|
| monitored_transforms | string[] | 监控的变换列表 |
| available_transforms | string[] | 可用的变换 |
| stale_transforms | string[] | 过期的变换 |
| transform_latencies_ms | float32[] | 变换延迟(ms) |
| topic_latencies_ms | float32[] | 话题延迟(ms) |
