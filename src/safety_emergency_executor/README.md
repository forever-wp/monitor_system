# safety_emergency_executor

从 `strategy_state_machine` 中抽取的应急措施执行功能包。

## 功能

- 订阅 `/safety_system/cmd` (`nav2_monitor/msg/SafetyCmd`)
- 执行应急措施：
  - `SLOW_DOWN`：优先使用消息中的减速百分比，否则回退到本地 `slow_down_percentage`（默认 50%）
  - `SOFT_STOP`：限速到 0%
  - `EMERGENCY_STOP`：禁用转发并发布制动序列到 `/command`
  - `RESUME`：恢复转发并清除全部安全限制
- 保留控制链路：`/command_safety` -> `/command`

## 启动

```bash
ros2 launch safety_emergency_executor safety_emergency_executor.launch.py
```

## 关键参数

- `safety_cmd_topic`（默认 `/safety_system/cmd`）
- `command_input_topic`（默认 `/command_safety`）
- `command_output_topic`（默认 `/command`）
- `slow_down_percentage`（默认 `50.0`）
- `brake_*`（急停制动序列）
