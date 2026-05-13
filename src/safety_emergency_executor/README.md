# safety_emergency_executor

从 `strategy_state_machine` 中抽取的应急措施执行功能包。

项目级数据链路见 [项目架构与数据链路](../../docs/project_architecture.html)。

## 功能

- 订阅 `/safety_system/cmd` (`nav2_monitor/msg/SafetyCmd`)
- 订阅 `/cmd_vel`、`/cmd_vel_miniapp`、`/cmd_vel_remote`、`/cmd_vel_other`
- 订阅 `/control_source_cmd` (`std_msgs/String`) 切换当前控制源
  - 可直接通过 `ros2 topic pub --once /control_source_cmd std_msgs/msg/String '{data: remote}'`
- 提供 `/safety_emergency_executor/query_control_source` (`std_srvs/Trigger`) 主动查询当前控制源
  - 可直接通过 `ros2 service call /safety_emergency_executor/query_control_source std_srvs/srv/Trigger '{}'`
- 持续发布 `/control_source_state` (`std_msgs/String`) 被动反馈当前控制源
  - 可直接通过 `ros2 topic echo /control_source_state` 持续观察当前控制源
  - 若只想取一帧，也可用 `ros2 topic echo --once /control_source_state`
- `active_control_source` 仍保留为内部镜像参数，收到切换 topic 后同步更新；外部不再建议用 `ros2 param get` 做主动查询
- 任一时刻只有一个控制源能进入 `/command`，非激活源直接丢弃
- 控制源切换或激活源停发时不补零、不保留最后一帧
- 四种控制源各自有独立安全开关，默认都启用
  - `control_source_navigation_safety_enabled`
  - `control_source_miniapp_safety_enabled`
  - `control_source_remote_safety_enabled`
  - `control_source_other_safety_enabled`
  - 切换控制源时日志会打印 `safety_enabled=true/false`
  - 当当前激活源安全关闭时，`/safety_system/cmd` 仍会更新内部安全状态，但不会减速、拦截转发，也不会下发急停制动序列
- 订阅 `/pressure_` (`std_msgs/Int32`) 作为外部单值压力覆盖入口
  - 收到后立即更新基础 `press`
  - `external_pressure_hold_s` 时间窗内自动调压完全不介入
  - 时间窗结束后，以最近一次外部值为新基线继续自动调压
- 小程序、远程驾驶、其他外部源可在 `Twist` 空闲轴里直接携带 `press/acc/place/ulock`
  - 映射为 `linear.y -> press`、`linear.z -> acc`、`angular.x -> place`、`angular.y -> ulock`
  - 仅当至少一个辅助轴非零时启用该扩展解析；否则继续沿用现有默认值和动态更新链路
  - 当该扩展解析生效时，`press` 不再经过自动调压，按上层输入原样下发
- 执行应急措施：
  - `SLOW_DOWN`：优先使用消息中的减速百分比，否则回退到本地 `slow_down_percentage`（默认 50%）
  - `SOFT_STOP`：限速到 0%
  - `EMERGENCY_STOP`：禁用转发并发布制动序列到 `/command`
  - `RESUME`：恢复转发并清除全部安全限制
- 安全动作始终会更新内部安全状态；是否真正作用到输出链路，取决于当前激活控制源的安全开关
- 主执行链路：`{active cmd_vel source}` + 传感器 + `/safety_system/cmd` -> `/command`
- 节点内部拆分为 `VelocityConverter`、`PressureAdjuster`、`SafetyPolicyExecutor`

## 启动

```bash
ros2 launch safety_emergency_executor safety_emergency_executor.launch.py
```

## 关键参数

- `safety_cmd_topic`（默认 `/safety_system/cmd`）
- `command_output_topic`（默认 `/command`）
- `cmd_vel_navigation_topic`（默认 `/cmd_vel`）
- `cmd_vel_miniapp_topic`（默认 `/cmd_vel_miniapp`）
- `cmd_vel_remote_topic`（默认 `/cmd_vel_remote`）
- `cmd_vel_other_topic`（默认 `/cmd_vel_other`）
- `control_source_command_topic`（默认 `/control_source_cmd`，`std_msgs/String`）
- 主动查询服务固定为 `/safety_emergency_executor/query_control_source`（`std_srvs/Trigger`，返回值在 `message`）
- `control_source_state_topic`（默认 `/control_source_state`）
- `control_source_state_publish_period_ms`（默认 `1000`，被动状态 topic 的持续发布周期，单位毫秒）
- `active_control_source`（默认 `navigation`，内部镜像参数，可选 `navigation` / `miniapp` / `remote` / `other`）
- `control_source_auto_preempt_enabled`（默认 `false`，仅预留自动抢占能力）
- `control_source_navigation_safety_enabled` / `control_source_miniapp_safety_enabled` / `control_source_remote_safety_enabled` / `control_source_other_safety_enabled`（默认都为 `true`，分别控制四种控制源是否启用安全链路）
- `cmd_vel_navigation_extended_fields_enabled`（默认 `false`）
- `cmd_vel_miniapp_extended_fields_enabled`（默认 `true`）
- `cmd_vel_remote_extended_fields_enabled`（默认 `true`）
- `cmd_vel_other_extended_fields_enabled`（默认 `true`）
- `wheel_odom_topic` / `loc_odom_topic` / `imu_topic`
- `pressure_update_topic`（`std_msgs/Int32`，只更新基础压力）
- `acc_update_topic`（直接覆盖当前 `acc` 值；未收到消息时保留默认 `acc_`）
- `external_pressure_hold_s`（默认 `30.0`，外部设压后自动调压冻结时长）
- `auto_pressure.*`（打滑检测与压力调节参数）
- `slow_down_percentage`（默认 `50.0`）
- `brake_*`（急停制动序列）
