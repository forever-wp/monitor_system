# Dynamic TTC Zone Design

## Background

当前 `nav2_monitor` 的 `approach / TTC` 是“静态 polygon 粗筛 + 预测 footprint 精算”的混合模式：

- 外层先用固定 `approach.points` 过滤候选点
- 内层再用 `footprint_points + prediction_motion` 估算轨迹碰撞时间

这套实现已经能工作，但固定 `approach` 框带来两个问题：

- 它把 TTC 的业务语义绑死在静态区域上，不适合后续把 TTC 区域直接作为减速、控制器切换、全局规划降速避障的入口
- 它让后续开发者误以为“碰撞提前区”是人工画出来的静态区域，而不是由车辆实际预测运动生成的动态区域

用户希望直接替换 `approach`，不再让静态框影响 TTC 主逻辑，同时要求实现保持轻量。

## Goal

用轻量的动态 TTC corridor 直接替换当前的 `approach` 主逻辑，并让该区域直接承担安全动作触发职责。

## Scope

In scope:

- 用动态 TTC corridor 替换 `approach` 的主判定逻辑
- 将 TTC 规则直接作为安全动作触发入口
- 保留 `front_slow/front_stop/rear_slow/rear_stop` 作为近场硬防线
- 为动态 TTC 增加轻量候选筛选和降采样策略
- 更新 RViz2 可视化，使其展示动态 TTC 区域而不是静态 `approach` 框
- 更新配置、测试、文档

Out of scope:

- 暂不输出单独的 TTC 状态信号给外部模块
- 暂不实现控制器切换或全局规划降速逻辑
- 不修改近场 `zone` 模式的硬防线语义

## Chosen Design

采用“双层碰撞链路”：

- `dynamic ttc`
  - 负责提前介入
  - 运行时根据 `footprint_points + prediction_motion + ttc_horizon` 生成动态 corridor
  - 直接触发减速或软停等安全动作
  - 未来可自然扩展为控制器切换、全局规划降速入口

- `near-field hard zone`
  - 继续保留现有 `front_slow/front_stop/rear_slow/rear_stop`
  - 只负责近场硬防线和最终兜底

这意味着当前 `approach` 不再作为 TTC 主逻辑存在。

## Configuration Changes

### 1. Canonical TTC model

TTC 规则的规范写法改为：

```yaml
zones:
  - name: "front_ttc"
    enabled: 1
    model: "ttc"
    motion_direction: "forward"
    level: "WARNING"
    safety_system: 1
    safety_slow_down_percentage: 50.0
    time_before_collision: 3.0
    recover_time_before_collision: 3.5
    min_hold_time_s: 0.2
    ttc_horizon_s: 3.5
    simulation_time_step: 0.1
    corridor_margin: 0.10
    candidate_downsample_resolution: 0.08
    actions: ["safety_system"]
```

设计原则：

- `ttc` 规则不再依赖静态 `points`
- TTC 区域由运行时动态生成
- `motion_direction` 保留，用于前进/倒车 TTC 分流
- `time_before_collision` 继续作为触发阈值
- `recover_time_before_collision`、`min_hold_time_s` 继续保留现有滞回/保持机制

### 2. New TTC-specific fields

为 `model: "ttc"` 新增字段：

- `ttc_horizon_s`
  - TTC 预测时域
  - 默认取 `max(time_before_collision, recover_time_before_collision)`

- `corridor_margin`
  - 在车辆外接半径之外额外扩出的 corridor 余量
  - 用于候选点粗筛

- `candidate_downsample_resolution`
  - 候选点降采样分辨率
  - 用于降低 TTC 精算点数

### 3. Legacy `approach` handling

仓库内配置、文档、命名全部切换到 `ttc`。

运行时对旧写法采用短期兼容策略：

- 允许解析 `model: "approach"`
- 但其语义直接等价为 `model: "ttc"`
- 原有 `points` 不再参与 TTC 主判定
- 启动时打印明确 warning，提示迁移到 `model: "ttc"`

这样可避免 OTA 切换瞬间因历史配置而直接启动失败，但未来主语义只有 `ttc`。

### 4. Footprint requirement

`ttc` 规则要求存在 `collision_detection.footprint_points`。

若未配置 footprint：

- 该 `ttc` 规则跳过并打印错误
- 不回退到旧的静态 `approach` polygon TTC 模式

这样可以确保 TTC 的业务语义始终基于真实车体，而不是退化回“点到原点”的逻辑。

## Runtime Behavior

`CollisionEvaluator::evaluate()` 中 `ttc` 分支改为以下流程：

1. 根据 `prediction_linear_x` 和 `motion_direction` 判断该 TTC 规则是否参与本周期计算
2. 计算本周期有效预测时域：
   - `active_horizon = max(ttc_horizon_s, time_before_collision, recover_time_before_collision)`
3. 基于 `prediction_motion` 采样未来若干个中心位姿
4. 用“中心轨迹 + 外接半径 + corridor_margin”构造轻量 corridor 粗筛
5. 仅保留落入 corridor 的碰撞点作为 TTC 候选点
6. 对候选点按 `candidate_downsample_resolution` 做空间降采样
7. 对降采样后的候选点，继续沿用现有 footprint trajectory TTC 精算
8. 若最小 TTC 小于等于阈值，则触发该 TTC 规则的安全动作

这样实现后：

- TTC 的业务区域是动态生成的
- 精算仍使用现有可靠的 footprint trajectory 逻辑
- 开销控制在“动态粗筛 + 少量候选点精算”

## Lightweight Strategy

为了满足“更轻量”，采用三级控制：

### 1. Dynamic corridor coarse filter

不对全量点直接跑 footprint TTC。

先使用未来轨迹中心线构造 corridor：

- corridor radius = 车体外接半径 + `corridor_margin`
- 只有进入 corridor 的点才进入下一阶段

这一层比静态 `approach` 更贴近真实运动，但仍明显轻于“对所有点精算 TTC”。

### 2. Candidate downsampling

对进入 corridor 的候选点做空间降采样：

- 同一小网格只保留一个代表点
- 优先保留更接近当前轨迹或更小 clearance 的点

这一步减少点云密集时的重复计算。

### 3. Early exit

在 TTC 精算阶段加入早停条件：

- 一旦找到 `ttc <= time_before_collision` 且 `clearance` 已接近碰撞，可提前停止后续无意义精算
- 至少保留“最坏 TTC”搜索的正确性

## Visualization Changes

当前 RViz2 的 TTC 可视化要跟随设计一起调整：

- 不再把静态 `approach` polygon 作为 TTC 主可视化
- `ttc_visualization_enabled = 1` 时，优先显示：
  - 动态 trajectory
  - 预测 footprint
  - 动态 corridor 边界
  - 命中点与 TTC 文本

近场 `front_slow/front_stop/rear_slow/rear_stop` 的 polygon 可视化继续保留，
因为它们仍然是静态硬防线区。

## Configuration Migration

仓库内默认配置直接迁移：

- `front_approach` 重命名为 `front_ttc`
- `rear_approach` 重命名为 `rear_ttc`
- 删除 TTC 规则中的静态 `points`
- 补充 `ttc_horizon_s`
- 补充 `corridor_margin`
- 补充 `candidate_downsample_resolution`

`front_slow/front_stop/rear_slow/rear_stop` 保持不变。

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

新增或调整测试覆盖：

- `ttc` 规则在无 `points` 配置下仍能正常计算并触发
- 前进时仅前向 `ttc` 规则参与
- 倒车时仅后向 `ttc` 规则参与
- 无 `footprint_points` 时，`ttc` 规则被安全跳过并记录错误
- corridor 粗筛后仍能命中真正的碰撞点
- 候选点降采样不会破坏已有 TTC 触发结果
- 近场 `front_slow/front_stop/rear_slow/rear_stop` 行为不回归
- RViz2 TTC 可视化能展示动态 corridor，而不是静态 `approach` polygon

## Risks And Mitigations

- 风险：完全移除静态 `approach` 主逻辑后，旧配置理解会发生变化
  - 缓解：仓库配置全部迁移到 `ttc`，运行时对旧 `approach` 给出强 warning

- 风险：动态 corridor 若做得过宽，会重新变成“另一个隐形静态框”
  - 缓解：corridor 由预测轨迹实时生成，宽度只由车体外接半径和小余量决定

- 风险：去掉静态框后点数过多，CPU 升高
  - 缓解：引入 corridor 粗筛、候选点降采样和早停

- 风险：调试时用户看不到 TTC 区域范围
  - 缓解：TTC 可视化中新增动态 corridor 显示

## Verification Criteria

满足以下条件即视为设计落地成功：

1. TTC 主逻辑不再依赖静态 `approach.points`
2. TTC 规则可直接触发安全动作
3. 近场硬防线区保持独立存在
4. 运行时开销不明显高于当前实现
5. RViz2 能看到动态 TTC corridor 与预测轨迹
6. 仓库默认配置中不再使用静态 `approach` 业务语义
