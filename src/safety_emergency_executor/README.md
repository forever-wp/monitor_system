# safety_emergency_executor

从 `strategy_state_machine` 中抽取的应急措施执行功能包。

## 功能

- 订阅 `/safety_system/cmd` (`nav2_monitor/msg/SafetyCmd`)
- 提供 `/set_manual_override` (`std_srvs/SetBool`) 外部手动接管接口，优先级高于安全执行
- 发布 `/manual_override_active` (`std_msgs/Bool`) 反馈当前是否处于手动接管
- 执行应急措施：
  - `SLOW_DOWN`：优先使用消息中的减速百分比，否则回退到本地 `slow_down_percentage`（默认 50%）
  - `SOFT_STOP`：限速到 0%
  - `EMERGENCY_STOP`：禁用转发并发布制动序列到 `/command`
  - `RESUME`：恢复转发并清除全部安全限制
- 主执行链路：`/cmd_vel` + 传感器 + `/safety_system/cmd` -> `/command`
- 节点内部拆分为 `VelocityConverter`、`PressureAdjuster`、`SafetyPolicyExecutor`

## 启动

```bash
ros2 launch safety_emergency_executor safety_emergency_executor.launch.py
```

## 关键参数

- `safety_cmd_topic`（默认 `/safety_system/cmd`）
- `command_output_topic`（默认 `/command`）
- `cmd_vel_topic` / `wheel_odom_topic` / `loc_odom_topic` / `imu_topic`
- `pressure_update_topic`（动态更新 `acc_/press_/place_/ulock_`）
- `acc_update_topic`（直接覆盖当前 `acc` 值；未收到消息时保留默认 `acc_`）
- `manual_override_service`（默认 `/set_manual_override`，`true=手动接管`、`false=恢复自动`）
- `manual_override_state_topic`（默认 `/manual_override_active`）
- `auto_pressure.*`（打滑检测与压力调节参数）
- `slow_down_percentage`（默认 `50.0`）
- `brake_*`（急停制动序列）
