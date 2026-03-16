# monitor_system 版本说明

本文档汇总当前仓库主要版本标签、功能变化和使用建议。

## 使用建议

- 需要稳定编译：优先使用 `v2.3.2` 或 `v2.4.1`
- 需要查看历史过程：可参考 `v2.3`、`v2.3.1`、`v2.4`
- `nav2_monitor_v2*` 与 `nav2_monitor_safety_refactor_v1` 保留为历史阶段标签

## v2

- 标签：`v2` / `nav2_monitor_v2`
- 说明：`nav2_monitor` v2 基线版本，完成碰撞能力扩展与文档更新
- 关键提交：`1254b39`

## v2.1

- 标签：`v2.1` / `nav2_monitor_v2.1`
- 说明：完成 `nav2_monitor` 与执行链路的集成增强，补充 reporter 上报能力
- 关键提交：`988a45a`

## v2.2

- 标签：`v2.2` / `nav2_monitor_v2.2`
- 说明：完善超声波加权与监控细化，提升碰撞检测与监控稳定性
- 关键提交：`38bece9`

## v2.3

- 标签：`v2.3` / `nav2_monitor_v2.3`
- 说明：引入任务驱动故障配置切换与 profile 热切换能力
- 已知问题：该 tag 的 `nav2_monitor` 源码存在接口不一致，直接编译可能失败
- 使用建议：不建议直接用于构建，改用 `v2.3.2`

## v2.3.1

- 标签：`v2.3.1`
- 说明：补齐 `nav2_monitor` 主配置与 `todoor / elevator / reverse` 三套 task profile
- 已知问题：这是配置补充版本，不包含完整源码编译修复
- 使用建议：如需可编译版本，改用 `v2.3.2`

## v2.3.2

- 标签：`v2.3.2`
- 说明：`v2.3.x` 可编译修复版本
- 修复内容：
  - 补齐 `CollisionDetectionConfig` 缺失字段
  - 补齐 `FaultDetector::get_monitored_transforms()` 及相关解析逻辑
- 使用建议：作为 `v2.3` / `v2.3.1` 的正式可编译替代版本

## v2.4

- 标签：`v2.4`
- 说明：安全执行链路重构为 `cmd_vel -> command`
- 功能变化：
  - `safety_emergency_executor` 内部拆分为 `VelocityConverter`、`PressureAdjuster`、`SafetyPolicyExecutor`
  - `nav2_monitor` 底盘命令监控源切换到 `/command`
- 已知问题：该 tag 未包含 `nav2_monitor` 的完整编译补丁
- 使用建议：不建议直接编译，改用 `v2.4.1`

## v2.4.1

- 标签：`v2.4.1`
- 说明：`v2.4` 的可编译补丁版本
- 修复内容：
  - 保留 `v2.4` 的安全链路重构
  - 补齐 `nav2_monitor` 缺失接口与字段，修复编译失败
- 使用建议：作为 `v2.4` 的正式可编译版本使用

## 历史阶段标签

- `nav2_monitor_safety_refactor_v1`
  - 安全状态协调与执行链路重构的中间阶段标签
