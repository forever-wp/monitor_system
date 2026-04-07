# Control Source Routing Design

## Background

当前 `safety_emergency_executor` 只有一条速度输入链路：

- 订阅单一 `cmd_vel_topic`
- 通过 `/set_manual_override` / `/get_manual_override` / `/manual_override_active`
  表示是否进入“外部手动接管”
- 再经过 `VelocityConverter -> PressureAdjuster -> SafetyPolicyExecutor`
  输出到底盘 `/command`

这套结构已经能完成“自动控制 + 安全动作执行”，但无法满足以下新需求：

- 同时接入多类速度来源：
  - `navigation`
  - `miniapp`
  - `remote`
  - `other`
- 由外部显式指定当前唯一有效的速度来源
- 同一时间必须有且仅有一种速度指令允许进入底盘执行链路
- 非当前来源的速度消息直接截断丢弃，不做融合，不做排队
- 保持安全动作链路独立存在，不因控制来源切换而失效

用户明确要求：

- 不做“百分比限速式分级”
- 做成“按来源分级 + 非激活来源截断”
- 启动默认来源为 `navigation`
- 当前来源停发速度时，不补发 0 速，不保持上一帧，不做额外动作
- 现有 `manual_override` 语义直接升级并并入新的多来源控制方案
- 外部切换为主，未来可预留自动抢占扩展

## Goal

把 `safety_emergency_executor` 升级为“多来源速度仲裁 + 安全执行”节点：

- 接入多路 `Twist` 输入
- 由外部显式切换当前控制来源
- 在任意时刻只放行当前来源的速度消息
- 保持 `SLOW_DOWN / SOFT_STOP / EMERGENCY_STOP / RESUME` 安全语义不变

## Scope

In scope:

- 将 `manual_override` 升级为多来源 `control_source`
- 为 4 类来源分别增加独立速度 topic
- 增加设置、查询、状态发布接口
- 重构节点内部路由逻辑，保证单时刻单来源放行
- 更新配置、测试、README、launch 说明

Out of scope:

- 本期不实现自动抢占策略
- 不增加速度来源间的优先级抢占规则
- 不做多来源速度融合
- 不在来源停发时主动补发停车命令
- 不修改 `nav2_monitor` 的安全故障判定语义

## Chosen Design

采用“来源仲裁内聚到 `safety_emergency_executor`”的方案：

- 在当前执行器节点内部新增 `ControlSourceController`
- 由它统一维护当前激活来源、来源合法性校验和模式切换
- `SafetyEmergencyExecutorNode` 改为订阅 4 路速度 topic
- 每条速度消息都先经过“来源是否激活”的门禁判断
- 只有当前来源的速度消息才能继续进入原有执行链路

这样可以最小化改动范围：

- 保留当前 `/safety_system/cmd -> SafetyPolicyExecutor -> /command` 主链路
- 不额外增加新的 mux 节点
- 让“控制来源”和“安全策略”在同一个执行节点内协同，但仍保持职责边界清晰

## Control Model

### 1. Canonical control sources

第一期固定支持 4 类来源：

- `navigation`
- `miniapp`
- `remote`
- `other`

运行时任何时刻都只有一个 `active_source`。

默认值：

- `active_source = navigation`

### 2. Gate semantics

速度消息的处理规则固定为：

- 若消息来源 `!= active_source`
  - 直接丢弃
  - 不进入 `VelocityConverter`
  - 不进入 `PressureAdjuster`
  - 不进入 `SafetyPolicyExecutor`
  - 不发布 `/command`
- 若消息来源 `== active_source`
  - 按现有链路继续处理

这意味着系统保证：

- 同一时刻只有一个来源能真正驱动底盘
- 非当前来源的输入不会混入执行链路

### 3. Source silence behavior

若当前 `active_source` 在一段时间内没有继续发布速度：

- 节点不补发 0 速
- 节点不保持上一帧速度
- 节点不主动发任何额外控制

底盘后续行为继续由现有底盘侧机制负责。

### 4. Relationship with safety policy

控制来源与安全策略是两条独立语义：

- `control_source`
  - 决定“谁的速度消息能进入执行链路”
- `safety_system/cmd`
  - 决定“当前被放行的速度链路还能否继续运行，以及是否需要降速/停车”

因此即使当前来源是 `miniapp` 或 `remote`：

- `SLOW_DOWN` 仍然生效
- `SOFT_STOP` 仍然生效
- `EMERGENCY_STOP` 仍然生效
- `RESUME` 仍然恢复安全策略

控制来源切换不会绕过安全，也不会替代安全动作。

## Interfaces

## 1. Speed input topics

将当前单一 `cmd_vel_topic` 拆分为 4 个参数化输入：

- `cmd_vel_navigation_topic`
  - 默认 `/cmd_vel`
- `cmd_vel_miniapp_topic`
  - 默认 `/cmd_vel_miniapp`
- `cmd_vel_remote_topic`
  - 默认 `/cmd_vel_remote`
- `cmd_vel_other_topic`
  - 默认 `/cmd_vel_other`

约束：

- 每个 topic 对应唯一来源
- 若某个 topic 参数为空字符串，则不创建该来源订阅
- 来源可以被切到一个未配置 topic 的 source，但此时不会有速度消息进入执行链路，节点需打印明确日志

## 2. Set interface

新增 service：

- `/set_control_source`

建议定义：

```srv
string source
---
bool success
string active_source
string message
```

语义：

- 请求中的 `source` 必须是 4 个 canonical source 之一
- 合法时：
  - 更新 `active_source`
  - 发布状态 topic
  - 返回 `success=true`
- 非法时：
  - 保持当前来源不变
  - 返回 `success=false`

切换行为只更新控制门禁，不主动发布 0 速。

## 3. Query interface

新增 service：

- `/get_control_source`

建议定义：

```srv
---
bool success
string active_source
string message
```

语义：

- 总是返回当前 `active_source`
- `message` 用于携带附加说明，例如 `active source is remote`

## 4. Passive state topic

新增状态 topic：

- `/control_source_state`

建议类型：

- `std_msgs/msg/String`

内容：

- `data = active_source`

QoS：

- `transient_local`
- `reliable`

发布时机：

- 节点启动完成后立即发布一次
- 每次控制来源切换后再次发布

## 5. Automatic preemption reservation

本期不实现自动抢占，但保留扩展边界：

- `ControlSourceController` 内部 API 设计为可在未来接入“候选来源请求”
- 可预留参数 `control_source_auto_preempt_enabled`
- 第一期开关默认 `false`
- 第一期开启后也不自动切换来源；仅用于保留配置位和内部状态位，避免未来改接口

这样可以满足“外部显式切换为主，自动抢占后续再加”的要求，同时不在本期引入未定义的抢占规则。

## Runtime Architecture

### 1. New unit: `ControlSourceController`

新增独立组件，职责收敛为：

- 保存当前 `active_source`
- 校验来源是否合法
- 处理切换请求
- 生成查询结果
- 提供状态 topic 输出内容
- 为未来自动抢占保留扩展点

该组件不负责：

- 速度转换
- 压力调整
- 安全动作执行
- 底盘命令发布

### 2. Existing units remain focused

- `VelocityConverter`
  - 继续只负责 `Twist -> CommandFrame`
- `PressureAdjuster`
  - 继续只负责压力调节
- `SafetyPolicyExecutor`
  - 继续只负责安全动作约束与急停制动序列
- `SafetyEmergencyExecutorNode`
  - 负责 ROS 接线、来源路由、状态发布和组件编排

## Runtime Flow

### 1. Startup

节点启动时：

1. 读取 `default_control_source`
2. 初始化 `ControlSourceController`
3. 创建 4 路来源订阅
4. 创建 `set/get/state` 控制来源接口
5. 发布一次 `/control_source_state`
6. 继续创建现有安全相关订阅与发布器

### 2. On velocity message

每路 `Twist` 回调统一使用相同逻辑：

1. 回调已知消息所属来源
2. 向 `ControlSourceController` 查询当前 `active_source`
3. 若来源不匹配：
   - 节流日志提示
   - 直接返回
4. 若来源匹配：
   - 执行 `VelocityConverter::convert`
   - 执行 `PressureAdjuster::apply`
   - 执行 `SafetyPolicyExecutor::apply`
   - 若安全策略允许，则发布 `/command`

### 3. On control source switch

收到 `/set_control_source` 请求时：

1. 校验 source 是否合法
2. 若非法：
   - 返回失败
   - 当前来源保持不变
3. 若合法：
   - 更新 `active_source`
   - 发布 `/control_source_state`
   - 打印切换日志

明确不做：

- 不发 0 速
- 不清空最近命令
- 不重置 `PressureAdjuster`
- 不重置 `SafetyPolicyExecutor`

### 4. On safety command

`/safety_system/cmd` 处理流程保持现有语义：

- 不依赖 `active_source`
- 继续更新 `SafetyPolicyExecutor`
- `EMERGENCY_STOP` 继续直接发布制动序列

这样即使当前来源切到 `remote`，安全链路仍可强制减速或停车。

## Configuration Changes

参数文件 `config/Monitor/safety_emergency_executor/safety_emergency_executor_params.yaml`
需要从“单 topic + manual override”迁移到“多 topic + control source”。

新增参数：

- `cmd_vel_navigation_topic`
- `cmd_vel_miniapp_topic`
- `cmd_vel_remote_topic`
- `cmd_vel_other_topic`
- `set_control_source_service`
- `get_control_source_service`
- `control_source_state_topic`
- `default_control_source`
- `control_source_auto_preempt_enabled`

删除参数：

- `cmd_vel_topic`
- `manual_override_service`
- `manual_override_query_service`
- `manual_override_state_topic`

默认值建议：

```yaml
cmd_vel_navigation_topic: "/cmd_vel"
cmd_vel_miniapp_topic: "/cmd_vel_miniapp"
cmd_vel_remote_topic: "/cmd_vel_remote"
cmd_vel_other_topic: "/cmd_vel_other"
set_control_source_service: "/set_control_source"
get_control_source_service: "/get_control_source"
control_source_state_topic: "/control_source_state"
default_control_source: "navigation"
control_source_auto_preempt_enabled: false
```

## Interface Packaging

由于 `std_srvs` 无法表达带字符串枚举的切换请求，
本期需要为 `safety_emergency_executor` 增加自定义 service 接口。

建议新增：

- `srv/SetControlSource.srv`
- `srv/GetControlSource.srv`

这会带来构建层修改：

- `CMakeLists.txt`
  - 增加 `rosidl_default_generators`
  - 增加 `rosidl_generate_interfaces(...)`
  - 为 `safety_emergency_executor_node` 和 `test_pipeline_components`
    增加对生成接口目标的构建依赖，确保同包内 service 头文件先生成后编译
- `package.xml`
  - 增加 `rosidl_default_generators`
  - 增加 `rosidl_default_runtime`
  - 增加 `member_of_group>rosidl_interface_packages</member_of_group>`

这是本次设计中唯一新增的接口类型扩展；状态 topic 继续使用 `std_msgs/msg/String`，
避免在第一期引入额外 message 类型。

## Migration Plan

本次升级属于语义替换，不保留旧 `manual_override` 接口。

迁移规则：

- `/set_manual_override` -> `/set_control_source`
- `/get_manual_override` -> `/get_control_source`
- `/manual_override_active` -> `/control_source_state`

外部调用方需要显式迁移到新的来源语义：

- 过去的“手动接管=true”
  - 现在应切换到 `miniapp` 或 `remote`
- 过去的“恢复自动=false”
  - 现在应切换回 `navigation`

README 和参数说明必须同步删除“manual override”旧措辞，统一使用 `control_source`。

## Affected Files

- `src/safety_emergency_executor/include/safety_emergency_executor/safety_emergency_executor_node.hpp`
- `src/safety_emergency_executor/src/safety_emergency_executor_node.cpp`
- `src/safety_emergency_executor/include/safety_emergency_executor/external_override_controller.hpp`
- `src/safety_emergency_executor/src/external_override_controller.cpp`
- `src/safety_emergency_executor/include/safety_emergency_executor/control_source_controller.hpp`
- `src/safety_emergency_executor/src/control_source_controller.cpp`
- `src/safety_emergency_executor/CMakeLists.txt`
- `src/safety_emergency_executor/package.xml`
- `src/safety_emergency_executor/README.md`
- `src/safety_emergency_executor/test/test_pipeline_components.cpp`
- `config/Monitor/safety_emergency_executor/safety_emergency_executor_params.yaml`
- `src/safety_emergency_executor/config/safety_emergency_executor_params.yaml`
- `src/safety_emergency_executor/srv/SetControlSource.srv`
- `src/safety_emergency_executor/srv/GetControlSource.srv`

## Testing Strategy

### 1. Controller unit coverage

新增测试覆盖：

- 默认来源初始化为 `navigation`
- 合法来源切换成功
- 非法来源切换失败且保持原值
- 查询接口返回当前来源

### 2. Node component coverage

调整现有组件测试，覆盖：

- 默认 `navigation` 时，仅导航 topic 可放行
- 切到 `remote` 后：
  - `navigation` 消息被丢弃
  - `remote` 消息被放行
- 切到 `miniapp` 后：
  - `miniapp` 消息被放行
  - 其余来源被丢弃
- 当前来源停发时：
  - 不额外产生新的 `/command`
- `/control_source_state` 启动即发布
- 切换来源后 `/control_source_state` 正确更新

### 3. Safety interaction coverage

必须验证：

- 任意 `active_source` 下，`SLOW_DOWN` 仍会限制当前被放行来源的速度
- 任意 `active_source` 下，`SOFT_STOP` 仍会把当前被放行速度截到 0
- 任意 `active_source` 下，`EMERGENCY_STOP` 仍会直接发送制动序列
- `RESUME` 只恢复安全策略，不修改当前 `active_source`

### 4. Config and documentation coverage

增加静态验证：

- 默认参数文件不再出现 `manual_override_*`
- 默认参数文件包含四路 `cmd_vel_*_topic`
- README 中所有外部接口文档切换为 `control_source`

## Risks And Mitigations

- 风险：旧客户端仍调用 `manual_override` 接口，升级后直接失效
  - 缓解：README、参数、变更说明中明确声明该接口族被替换

- 风险：切换到一个未配置 topic 的来源后，外部误以为“切换成功但车不动是故障”
  - 缓解：service 成功响应中附带当前来源；启动和切换日志中明确提示该来源是否配置了输入 topic

- 风险：多路回调逻辑散落在主节点中，后续自动抢占难扩展
  - 缓解：把来源状态和校验逻辑收敛到 `ControlSourceController`

- 风险：把安全控制和来源控制混成一个状态机，后续排障困难
  - 缓解：保持两条状态独立；`control_source` 不控制安全状态，`safety_cmd` 不修改来源

## Verification Criteria

满足以下条件即可认为设计落地成功：

1. `safety_emergency_executor` 可同时接入四路速度来源
2. 运行时始终只有当前 `active_source` 的速度消息能进入 `/command` 链路
3. 非当前来源消息被截断，不做融合、不补偿、不保留
4. 默认来源为 `navigation`
5. 提供 `set/get/state` 三件套接口管理当前来源
6. 旧 `manual_override` 接口已被移除并完成文档迁移
7. 安全动作在任意控制来源下仍保持原有语义
