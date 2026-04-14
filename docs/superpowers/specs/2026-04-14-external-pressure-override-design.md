# External Pressure Override Design

## Background

当前 `safety_emergency_executor` 提供两条与底盘控制字段相关的外部更新链路：

- `/pressure_`
  - 类型为 `std_msgs/String`
  - 通过 JSON 一次更新 `acc / press / place / ulock`
- `/acc_`
  - 类型为 `std_msgs/Int32`
  - 只更新 `acc`

这套接口有两个问题：

- `/pressure_` 的输入形式和 `/acc_` 不一致，外部接入复杂，且对“只调压力”这个需求来说过重
- 外部更新 `press` 后，自动调压链路仍可能立刻继续改写输出压力，导致“外部设压”在运行时并不可靠

用户明确要求：

- 将外部压力更新改成和 `/acc_` 一样的单值 topic
- 输入类型改成整数
- 只控制单一压力值，不再复用 JSON 批量更新
- 外部设置压力后，自动调压必须暂时完全不介入
- 本期锁定窗口固定为 `30s`
- 锁定窗口结束后，自动调压恢复，但要以外部设置的压力值作为新的基线继续工作

## Goal

把 `safety_emergency_executor` 的外部设压能力收敛成一个明确、可预期的运行时覆盖机制：

- `/pressure_` 改为 `std_msgs/Int32`
- 只接收单个 `press` 值
- 收到外部压力后立即生效
- 自动调压在 `30s` 内完全旁路
- `30s` 结束后，自动调压以该外部压力值为新的基础压力继续调整

## Scope

In scope:

- 将 `pressure_update_topic` 的消息类型从 `std_msgs/String` 改为 `std_msgs/Int32`
- 删除通过 `/pressure_` 更新 `acc / place / ulock` 的 JSON 语义
- 新增“外部压力覆盖保持窗口”参数
- 调整 `VelocityConverter` / `PressureAdjuster` 边界，保证窗口期内自动调压不改写外部压力
- 更新测试、参数说明和 README

Out of scope:

- 不修改 `/acc_` 的现有行为
- 不修改 `cmd_vel_remote` / `cmd_vel_miniapp` / `cmd_vel_other` 的辅助轴嵌入字段语义
- 不新增“手动清除外部压力锁定”的接口
- 不新增“恢复默认 `press_`”的显式控制接口

## Chosen Design

采用“外部压力覆盖 + 定时保护窗口”的方案：

- `pressure_update_topic` 保持原参数名不变，默认 topic 仍是 `/pressure_`
- 订阅消息类型改为 `std_msgs/msg/Int32`
- 节点收到新压力值后，立即把它记为当前外部覆盖压力
- 同时开启一个持续 `30s` 的保护窗口
- 保护窗口存在期间：
  - 输出压力固定使用该外部值
  - `PressureAdjuster` 不得对该压力做任何自动调节
- 保护窗口结束后：
  - 外部压力值继续保留为新的基础压力
  - 自动调压从这个新基础压力继续增减，不回退到 YAML 中原始 `press_`

这个方案满足两个关键目标：

- 外部设压一定真实生效，不会被自动调压马上抢回去
- 自动调压不会被永久关闭，超时后仍可继续参与防打滑调节

## Runtime Model

### 1. Canonical pressure sources

压力值在运行时有 3 类来源：

- 默认配置压力 `press_`
- 外部压力覆盖 topic `/pressure_`
- 外部速度消息嵌入字段 `linear.y -> press`

它们的优先级为：

1. `cmd_vel` 辅助轴嵌入压力
2. 外部压力覆盖窗口
3. 自动调压基于当前基础压力工作
4. 默认配置压力 `press_`

### 2. External pressure override semantics

收到 `/pressure_` 的 `Int32.data = N` 后：

- 当前基础压力立即更新为 `N`
- 记录一次外部覆盖生效时间
- 启动或刷新保护窗口：
  - `expires_at = now + external_pressure_hold_s`

本期默认：

- `external_pressure_hold_s = 30.0`

语义约束：

- 如果窗口期内再次收到新的外部压力值：
  - 新值立即替换旧值
  - 窗口重新计时 `30s`
- 如果窗口期内没有新的外部压力消息：
  - 到期后自动解除“禁止自动调压”状态

### 3. Relationship with automatic pressure adjustment

窗口期内：

- `PressureAdjuster` 必须完全旁路
- 不允许根据打滑状态增压
- 不允许根据恢复状态回落压力
- 输出压力保持外部指定值

窗口结束后：

- 自动调压重新恢复
- 恢复后的 `base_press` 使用“最近一次外部设置值”
- 自动调压对该基础压力做正常增减

例子：

- YAML `press_ = 800`
- 外部发 `/pressure_ = 1100`
- 未来 `30s` 内输出压力固定为 `1100`
- `30s` 后若检测到打滑，可从 `1100` 向上调
- 若无打滑恢复，则可从更高值回落到 `1100`
- 不允许自动回落到 `800`

### 4. Relationship with embedded Twist fields

现有多来源速度输入已经支持在辅助轴中携带：

- `linear.y -> press`
- `linear.z -> acc`
- `angular.x -> place`
- `angular.y -> ulock`

该路径保持不变，并继续拥有最高优先级。

当辅助轴嵌入压力生效时：

- 该帧命令继续直接绕过自动调压
- 不修改“外部压力覆盖 topic”的持有窗口状态
- 不刷新 `/pressure_` 维护的基础压力

也就是说：

- `/pressure_` 作用于“节点基础压力状态”
- `cmd_vel` 辅助轴作用于“该条速度命令自身”

两者语义独立，不互相覆盖状态。

## Interface Changes

### 1. Pressure update topic

保留参数名：

- `pressure_update_topic`

默认值保持：

- `/pressure_`

消息类型调整为：

- `std_msgs/msg/Int32`

消息内容：

- `data = pressure_value`

示例：

```bash
ros2 topic pub --once /pressure_ std_msgs/msg/Int32 "{data: 1100}"
```

### 2. New hold-window parameter

新增参数：

- `external_pressure_hold_s`

默认值：

- `30.0`

语义：

- 表示收到 `/pressure_` 后，自动调压完全旁路的持续时间（秒）

本期约束：

- 值必须 `>= 0.0`
- 若设为 `0.0`，则外部压力只更新基础压力，不保留保护窗口

## Component Design

### 1. `VelocityConverter`

职责调整：

- 删除 `update_params_from_json()`
- 新增单独的压力更新接口，例如：
  - `update_press_from_topic(int press_value)`
- 保留：
  - `update_acc_from_topic(int acc_value)`
  - 嵌入字段解析
  - `template_frame()`

状态要求：

- 最近一次 `/pressure_` 的值要写入内部基础参数状态
- `template_frame()` 返回的基础 `press` 必须反映这个最新值

### 2. `PressureAdjuster`

职责扩展：

- 保留现有自动调压逻辑
- 新增“外部压力锁定是否仍在有效期内”的判断
- 在窗口期内直接返回，不改写 `frame.press`

实现要求：

- 不把“时间窗口逻辑”散落在节点主类里
- 由 `PressureAdjuster` 或其下层保持最小必要状态：
  - 最近一次外部压力时间
  - 持有时长

判断逻辑：

- 若 `frame.press_from_embedded_fields == true`
  - 继续直接返回
- 否则若外部压力锁定窗口仍有效
  - 直接返回
- 否则再进入现有自动调压分支

### 3. `SafetyEmergencyExecutorNode`

节点层改动保持最小化：

- `pressure_sub_` 的订阅类型改为 `std_msgs::msg::Int32`
- `on_pressure_update()` 改为读取整数压力值
- 收到消息后：
  - 更新 `VelocityConverter` 内的基础压力
  - 通知 `PressureAdjuster` 刷新外部压力保护窗口

节点不负责：

- 手动计算过期时间
- 直接修改自动调压内部状态

这样可以继续保持：

- `VelocityConverter` 管理控制字段模板
- `PressureAdjuster` 管理“是否可自动改压”
- Node 仅做 topic 接入和编排

## Edge Cases

### 1. Repeated external pressure updates

若在窗口期内连续收到多个 `/pressure_`：

- 每次都立即更新基础压力
- 每次都重新刷新 `30s` 保护窗口

### 2. Hold window disabled

若 `external_pressure_hold_s = 0.0`：

- 外部压力仍然更新基础压力
- 自动调压不再被时间窗口阻止
- 自动调压会立刻以新基础压力参与调节

### 3. Invalid pressure values

本期不额外增加复杂业务约束，但至少要求：

- topic 类型必须是 `Int32`
- 不做 JSON 解析兼容

是否对负值或超范围值做业务裁剪，继续沿用底盘既有语义；本期不在此设计中额外引入新限制，避免和现有 `press_` 参数口径不一致。

## Testing

### 1. Component tests

补充或改写以下测试：

- `/pressure_` 改成 `Int32` 后可以更新基础压力模板
- 收到外部压力后，`template_frame().press` 立即更新
- 持有窗口内 `PressureAdjuster::apply()` 不改写该压力
- 持有窗口过期后，自动调压重新生效
- 过期后自动调压以外部值为基线，而不是旧 `press_`

### 2. Routing / integration tests

补充节点级测试：

- 发布 `/pressure_ = 1100` 后，下一条输出命令 `press == 1100`
- 在滑移数据存在的情况下，窗口期内输出压力仍保持 `1100`
- 等待窗口过期后，再次发命令时自动调压恢复工作

### 3. Docs / config verification

同步更新：

- `README.md`
- `config/safety_emergency_executor_params.yaml`
- `config/Monitor/safety_emergency_executor/safety_emergency_executor_params.yaml`

并明确说明：

- `/pressure_` 现在是 `std_msgs/Int32`
- `external_pressure_hold_s` 默认值为 `30.0`
- 窗口结束后以最近外部值为基线继续自动调压
