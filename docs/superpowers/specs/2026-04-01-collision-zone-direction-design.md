# Collision Zone Direction Design

## Background

当前 `nav2_monitor` 的碰撞检测里，`approach / TTC` 在配置了 `footprint_points` 后会使用
`prediction_motion` 的带符号速度做轻量轨迹估算，因此对前进/倒车方向有一定感知。

但 `slow_down / stop` 这类 `zone` 模式仍然只做“点是否进入 polygon”的判断，
不区分车辆此刻是在前进还是倒车。这会导致：

- 前进时，后向缓停/急停区也可能参与判断
- 倒车时，如果仍使用前向 zone，实际运动方向与碰撞检测方向不一致

## Goal

让 `zone` 模式根据当前速度方向自动筛选前向/后向碰撞区，而不是依赖整份配置文件切换。

## Scope

In scope:

- 仅增强 `zone` 模式的方向筛选
- 为 `CollisionZoneConfig` 增加显式 `motion_direction` 配置
- 增加一个速度方向判断阈值
- 为前向/后向缓停区、急停区补方向字段
- 增加相应测试与文档

Out of scope:

- 不修改 `approach / TTC` 的方向逻辑
- 不引入整份碰撞配置文件的自动切换
- 不改变现有 zone 多边形几何算法

## Chosen Design

采用“同一配置文件内、运行时按速度方向筛选 zone”的方案：

- 不切换整份 `fault_detector_config.yaml`
- 在同一配置里同时保留 `front_*` 与 `rear_*` zone
- 运行时根据当前 `prediction_linear_x` 判断车辆是前进还是倒车
- 只让与当前运动方向匹配的 `zone` 生效

## Configuration Changes

### 1. `collision_detection.direction_speed_threshold`

在 `collision_detection` 下新增全局阈值：

```yaml
direction_speed_threshold: 0.05
```

含义：

- `prediction_linear_x > threshold` 认为前进
- `prediction_linear_x < -threshold` 认为倒车
- 绝对值在阈值内认为方向不确定

### 2. `zones[*].motion_direction`

在 `zone` 配置下新增显式字段：

```yaml
motion_direction: "forward"
```

取值：

- `forward`
- `reverse`
- `both`

设计原则：

- 不根据 zone 名字猜方向
- 用显式配置表达语义
- 现有 `front_*` zone 补成 `forward`
- 现有 `rear_*` zone 补成 `reverse`
- 未指定时默认 `both`，保持向后兼容

## Runtime Behavior

在 `CollisionEvaluator::evaluate()` 的 `zone` 分支中新增方向筛选：

1. 先根据 `prediction_linear_x` 和 `direction_speed_threshold` 计算当前方向
2. 若 zone 的 `motion_direction` 与当前方向不匹配，则跳过该 zone
3. 若当前方向不确定：
   - 若有上一次有效方向，则保持上一次方向
   - 若没有历史方向，则按 `both` 处理

## Why This Design

相比“根据任务切换整个 reverse profile”，这个方案更适合缓停区/急停区：

- 直接依赖实时速度方向，不必等待任务状态切换
- 配置文件结构更稳定
- 前后向安全区可以同时配置在一份文件里
- 更适合即时安全逻辑

## Testing Strategy

新增/调整测试覆盖：

- 前进时，仅前向 `zone` 会触发
- 倒车时，仅后向 `zone` 会触发
- 方向不确定时，保持上一次方向
- 未配置 `motion_direction` 时，默认行为与旧版一致
- `approach / TTC` 现有测试保持通过

## Documentation Updates

需要更新：

- `src/nav2_monitor/README.md`
- `src/nav2_monitor/docs/architecture.md`

重点写明：

- `zone` 模式现已支持按当前运动方向自动筛选
- 该逻辑与 `approach / TTC` 是分开的
- 推荐给 `front_*` / `rear_*` zone 显式配置 `motion_direction`

## Risks And Mitigations

- 风险：速度接近 0 时方向抖动，导致 zone 在前后之间来回切换
  - 缓解：引入 `direction_speed_threshold` 和“保持上一次方向”的逻辑

- 风险：旧配置未补 `motion_direction`
  - 缓解：默认 `both`，保证兼容

- 风险：用户误以为会自动切换整份 reverse 配置
  - 缓解：文档明确这是“同一配置内运行时筛选 zone”

## Verification Criteria

满足以下条件即视为设计落地成功：

1. `slow_down / stop` zone 可按速度方向自动筛选前后向
2. `approach / TTC` 现有行为不回归
3. 未配置 `motion_direction` 的旧配置仍能工作
4. 文档清楚说明“方向筛选”与“配置文件切换”的区别
