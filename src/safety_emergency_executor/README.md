# safety_emergency_executor

从 `strategy_state_machine` 中抽取的应急措施执行功能包。

## 功能

- 订阅 `/safety_system/cmd` (`nav2_monitor/msg/SafetyCmd`)
- 订阅 `/cmd_vel`、`/cmd_vel_miniapp`、`/cmd_vel_remote`、`/cmd_vel_other`
- 使用 ROS 2 标准参数服务切换 `active_control_source`
  - 可直接通过 `ros2 param set /safety_emergency_executor active_control_source remote`
- 使用 ROS 2 标准参数服务主动查询当前 `active_control_source`
  - 可直接通过 `ros2 param get /safety_emergency_executor active_control_source`
- 发布 `/control_source_state` (`std_msgs/String`) 被动反馈当前控制源
- 任一时刻只有一个控制源能进入 `/command`，非激活源直接丢弃
- 控制源切换或激活源停发时不补零、不保留最后一帧
- 小程序、远程驾驶、其他外部源可在 `Twist` 空闲轴里直接携带 `press/acc/place/ulock`
  - 映射为 `linear.y -> press`、`linear.z -> acc`、`angular.x -> place`、`angular.y -> ulock`
  - 仅当至少一个辅助轴非零时启用该扩展解析；否则继续沿用现有默认值和动态更新链路
  - 当该扩展解析生效时，`press` 不再经过自动调压，按上层输入原样下发
- 执行应急措施：
  - `SLOW_DOWN`：优先使用消息中的减速百分比，否则回退到本地 `slow_down_percentage`（默认 50%）
  - `SOFT_STOP`：限速到 0%
  - `EMERGENCY_STOP`：禁用转发并发布制动序列到 `/command`
  - `RESUME`：恢复转发并清除全部安全限制
- 安全动作独立于控制源选择，始终作用在当前激活源的输出链路上
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
- `active_control_source`（默认 `navigation`，可选 `navigation` / `miniapp` / `remote` / `other`）
- `control_source_state_topic`（默认 `/control_source_state`）
- `control_source_auto_preempt_enabled`（默认 `false`，仅预留自动抢占能力）
- `cmd_vel_navigation_extended_fields_enabled`（默认 `false`）
- `cmd_vel_miniapp_extended_fields_enabled`（默认 `true`）
- `cmd_vel_remote_extended_fields_enabled`（默认 `true`）
- `cmd_vel_other_extended_fields_enabled`（默认 `true`）
- `wheel_odom_topic` / `loc_odom_topic` / `imu_topic`
- `pressure_update_topic`（动态更新 `acc_/press_/place_/ulock_`）
- `acc_update_topic`（直接覆盖当前 `acc` 值；未收到消息时保留默认 `acc_`）
- `auto_pressure.*`（打滑检测与压力调节参数）
- `slow_down_percentage`（默认 `50.0`）
- `brake_*`（急停制动序列）
