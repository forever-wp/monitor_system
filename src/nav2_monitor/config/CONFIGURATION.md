# nav2_monitor 配置说明

本目录是包内默认配置，运行时配置通常同步到 `/opt/ry/config/Monitor/nav2_monitor/`。仓库根目录的 `config/Monitor/nav2_monitor/` 是部署配置镜像，两边的配置层级应保持一致。

配置分为两层：节点参数层和业务规则层。节点参数层负责“节点怎么启动、订阅哪个状态、输出到哪里”；业务规则层负责“监控什么、怎样判故障、触发什么动作”。任务切换只切换业务规则文件，不需要为每个任务复制整套节点参数。

## 1. 节点参数层

这些文件都是 ROS 2 参数文件，格式必须是：

```yaml
节点名:
  ros__parameters:
    参数名: 参数值
```

顶层节点名必须和 launch 中的 `name=` 完全一致，否则 ROS 2 不会把参数加载到对应节点。

| 文件 | 顶层节点名 | 作用 |
|---|---|---|
| `nav2_monitor_params.yaml` | `nav2_monitor_aggregator` | 聚合器参数，消费 `/monitor/*_state`，发布状态、故障事件、节点管理器命令和安全命令。 |
| `topic_frequency_monitor_params.yaml` | `topic_frequency_monitor` | topic 数据源、频率、新鲜度监测参数。 |
| `vehicle_state_judge_params.yaml` | `vehicle_state_judge` | 小车状态判断节点参数，检测有指令不动、无指令仍运动、数据源缺失等。 |
| `node_tf_monitor_params.yaml` | `node_tf_monitor` | ROS 节点在线状态和 TF 可用性监测参数。 |
| `battery_monitor_params.yaml` | `battery_monitor` | 电池 topic 订阅和状态输出参数。 |
| `algorithm_feedback_monitor_params.yaml` | `algorithm_feedback_monitor` | 算法反馈状态监测参数。 |
| `collision_monitor_params.yaml` | `collision_monitor` | 碰撞/TTC 原始输入、导航模式切换和可视化相关参数。 |

聚合器只订阅标准状态总线：

```text
/monitor/topic_states
/monitor/vehicle_state
/monitor/node_tf_state
/monitor/battery_state
/monitor/feedback_state
/monitor/collision_state
```

聚合器不再直接订阅高频传感器、底盘反馈、算法反馈原始输入或碰撞原始输入。

## 2. 业务规则层

这些文件不是 ROS 2 参数文件，不包含 `ros__parameters`。

| 文件 | 作用 |
|---|---|
| `fault_detector_config.yaml` | 默认业务规则，定义模块、topic、TF、碰撞、小车状态、组合故障等规则。 |
| `profiles/fault_detector_todoor.yaml` | 到门任务规则。 |
| `profiles/fault_detector_elevator.yaml` | 电梯任务规则。 |
| `profiles/fault_detector_reverse.yaml` | 倒车任务规则。 |

业务规则顶层结构：

```yaml
multi_value_judge:
combined_fault_rules:
target_transforms:
collision_detection:
vehicle_state_judge:
modules:
```

`modules` 中每个模块描述节点、topic 和反馈规则。当前格式只使用 `nodemanager`，不使用旧 `supervisor` 字段。

## 3. 任务切换

任务切换入口在 `nav2_monitor_params.yaml`：

```yaml
current_nav_task: "default"
task_status_topic: "/task_status_code"
task_fault_configs.default: "/opt/ry/config/Monitor/nav2_monitor/fault_detector_config.yaml"
task_fault_configs.todoor: "/opt/ry/config/Monitor/nav2_monitor/profiles/fault_detector_todoor.yaml"
task_fault_configs.elevator: "/opt/ry/config/Monitor/nav2_monitor/profiles/fault_detector_elevator.yaml"
task_fault_configs.reverse: "/opt/ry/config/Monitor/nav2_monitor/profiles/fault_detector_reverse.yaml"
```

切换任务时，只切换当前使用的 `fault_detector_config` 路径。节点参数文件仍保持同一套，避免每个任务都复制一组启动参数。

聚合器会把当前任务和 `fault_config` 通过 `/monitor/config_profile` 广播出去，QoS 为 `reliable + transient_local`。以下独立模块会订阅该 topic，并在 profile 变化时自动重载业务规则：

- `vehicle_state_judge`
- `node_tf_monitor`
- `algorithm_feedback_monitor`
- `collision_monitor`
- `topic_frequency_monitor`

`topic_frequency_monitor` 从当前业务规则的 `modules[].watch_topics` 生成监控清单。只有配置了 `min_hz > 0` 的 topic 才会通过 ROS graph 自动发现消息类型并建立订阅计算频率；未配置 `min_hz` 的 topic 只检查 publisher 是否存在，不订阅数据流。任务 profile 切换后会自动重建清单；如果某个需要频率统计的 topic 暂时没有发布者，频率节点会先输出 `has_publisher=false`、`has_data=false`，等发布者出现后自动补建订阅。

## 4. 修改建议

- 改 topic 名、输出名、超时时间：优先修改对应 `*_params.yaml`。
- 改监控哪些模块、哪些节点、哪些 topic、故障动作：修改 `fault_detector_config.yaml` 或对应 profile。
- 改 TTC、碰撞源、导航 FAST/SAFE 切换：修改 `collision_detection` 段，必要时同步 `collision_monitor_params.yaml` 中的基础参数。
- 改小车状态判断阈值：修改 `vehicle_state_judge` 段。
- 新增任务模式：新增 `profiles/fault_detector_xxx.yaml`，并在 `nav2_monitor_params.yaml` 中增加 `task_fault_configs.xxx` 和任务状态码映射。
- 如果新增的任务 profile 改了碰撞、小车状态、节点/TF、算法反馈或 watch topic 规则，独立模块会通过 `/monitor/config_profile` 自动跟随。

## 5. 校验要点

- 所有 `*_params.yaml` 顶层节点名必须和 launch 节点名一致。
- `fault_detector_config.yaml` 和 profiles 不要加 `ros__parameters`。
- 配置中不要使用旧字段 `supervisor`、`topics`、`feedback_topics`、`topic_name`。
- 部署路径统一使用 `/opt/ry/config/Monitor/nav2_monitor/`。
