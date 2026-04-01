# TTC Visualization Design

## Background

当前 `nav2_monitor` 已支持 `approach / TTC` 碰撞检测，包括：

- footprint clearance
- 轻量预测轨迹
- hysteresis / min hold

但目前在 RViz2 中只能看到静态碰撞区 polygon，
无法直接看到 TTC 计算过程中的预测轨迹、预测 footprint、最近碰撞点或 TTC 数值。

## Goal

为 `approach / TTC` 增加 RViz2 可视化，并通过单一配置开关控制开闭。

## Scope

In scope:

- 为 `approach / TTC` 增加 `MarkerArray` 可视化
- 增加一个总开关配置
- 默认关闭，避免影响现网开销
- 增加相应测试和文档

Out of scope:

- 不改现有 `PolygonStamped` zone 可视化逻辑
- 不增加复杂的可视化参数矩阵
- 不改变 TTC 判定本身

## Chosen Design

采用最小实现：

- 在 `collision_detection` 下增加一个总开关：

```yaml
ttc_visualization_enabled: 0
```

- 当开关关闭时，不发布 TTC 可视化 marker
- 当开关打开时，发布一个固定 topic：

```text
/nav2_monitor/collision_ttc_markers
```

- 使用 `visualization_msgs/msg/MarkerArray`

## Visualized Items

仅针对 `approach / TTC` 发布以下内容：

1. 预测轨迹 `LINE_STRIP`
2. 预测 footprint 轮廓
3. 最近碰撞点 `SPHERE`
4. TTC 文本 `TEXT_VIEW_FACING`

这些 marker 只作为调试辅助，不参与控制。

## Runtime Behavior

- 开关关闭：
  - 不发布 TTC marker
  - 不影响现有 zone polygon 可视化

- 开关打开：
  - 在 `approach / TTC` 路径计算后构造 marker
  - 每次 `check_health()` 或碰撞评估周期更新
  - 无有效 TTC 时清空或发布空 marker 集

## Why This Design

- 单开关最简单，符合“只需要设置开关”
- 默认关闭，避免给现网增加额外负担
- 用固定 topic，便于 RViz2 直接配置
- 不把 zone polygon 和 TTC 轨迹混到一套消息里，调试更清晰

## Affected Files

- `src/nav2_monitor/include/nav2_monitor/fault_detector.hpp`
- `src/nav2_monitor/src/fault_detector.cpp`
- `src/nav2_monitor/include/nav2_monitor/collision_evaluator.hpp`
- `src/nav2_monitor/src/collision_evaluator.cpp`
- `src/nav2_monitor/include/nav2_monitor/nav2_monitor_node.hpp`
- `src/nav2_monitor/src/nav2_monitor_node.cpp`
- `src/nav2_monitor/test/test_fault_detector.cpp`
- `config/Monitor/nav2_monitor/fault_detector_config.yaml`
- `config/Monitor/nav2_monitor/profiles/*.yaml`
- `src/nav2_monitor/config/*.yaml`
- `src/nav2_monitor/README.md`
- `src/nav2_monitor/docs/architecture.md`

## Testing Strategy

新增/调整验证：

- 默认关闭时，现有测试全部保持通过
- 打开开关时，能创建并发布 TTC marker topic
- `approach / TTC` 逻辑不回归

## Risks And Mitigations

- 风险：marker 发布增加少量运行时开销
  - 缓解：默认关闭，仅调试时打开

- 风险：轨迹/footprint marker 与现有 polygon 可视化混淆
  - 缓解：单独使用 `/nav2_monitor/collision_ttc_markers`

- 风险：无有效 TTC 时显示残留 marker
  - 缓解：无有效命中时主动清空 marker

## Verification Criteria

满足以下条件即可认为设计落地成功：

1. 配置中仅通过一个开关控制 TTC 可视化
2. 默认关闭时系统行为与当前一致
3. 打开后 RViz2 可看到 TTC 相关 marker
4. 现有碰撞检测测试不回归
