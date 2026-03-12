# Changelog

## v2 - 2026-03-12

作者：ToTo

### 新增

- 将安全链路拆分为：`Nav2MonitorNode -> MonitorDataStore -> FaultDetector -> FaultStateCoordinator -> safety_emergency_executor`
- 新增 `MonitorDataStore`，统一保存最新输入快照
- 新增 evaluator 拆分：
  - `WatchTopicEvaluator`
  - `FeedbackRuleEvaluator`
  - `ChassisEvaluator`
  - `CollisionEvaluator`
- 新增碰撞检测能力：
  - `LaserScan`
  - `PointCloud2`
  - polygon zone
  - slowdown zone
  - stop zone
  - approach / TTC
- 新增碰撞区域可视化 `PolygonStamped`
- 新增 `FaultEvent.edge` 触发/恢复边沿语义
- 新增 `SafetyCmd.RESUME`
- 新增正式架构文档：`src/nav2_monitor/docs/architecture.md`

### 重构

- `FaultDetector` 从“大而全判断器”收敛为配置加载 + evaluator 编排器
- `FaultStateCoordinator` 负责故障标志位、边沿识别、多故障组合与安全动作发布
- `README` 精简重写，文档入口更清晰
- 配置语义重构：
  - `topics -> watch_topics`
  - `feedback_topics -> feedback_rules`
  - `topic_name -> source_topic`

### 行为变化

- 安全动作支持自动恢复
- 急停也支持在所有安全故障恢复后自动 `RESUME`
- 碰撞检测已并入统一安全链路

### 兼容说明

- 旧配置字段 `topics`、`feedback_topics`、`topic_name` 已不再接受
- 旧 `FaultDetector` 输入接口暂时保留为兼容包装，但内部已统一写入 `MonitorDataStore`
