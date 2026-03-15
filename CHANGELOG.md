# Changelog

## v2.2 - 2026-03-15

作者：ToTo

### 新增

- 新增单 topic 八路超声波碰撞输入：`/ultrasonic_eight` JSON
- 新增 `collision_detection.ultrasonic_widget` 8 路权重配置，默认偏向前视与前侧
- 新增超声波权重与内置安装位姿映射逻辑，并统一进入现有碰撞检测链路

### 优化

- `watch_topics` 未配置 `min_hz` 时，仅检测有无，不做频率阈值判断
- `chassis_stationary.odom_topic` 允许为空，底盘异常判断可不依赖 odom
- `event_json` 固定 schema，`safety` / `supervisor` 字段始终存在
- 修复碰撞点聚合路径中的死锁问题，降低碰撞开启时卡住的风险

### 说明

- 现在配置超声波权重时，推荐只写：`ultrasonic_widget: [0-1, ..., 0-1]`
- 完整 8 路位姿使用内置默认布局，避免配置文件过长

## v2.1 - 2026-03-13

作者：ToTo

### 新增

- 新增包内 `MonitorReporter` 类，负责系统心跳 JSON 与异常/恢复 JSON 上报
- `/supervisor/cmd` 改为 JSON 字符串发布
- 新增 `LaserScan` / `PointCloud2` 碰撞检测输入
- 新增 `slowdown zone`、`stop zone`、`approach / TTC`
- 新增碰撞区域可视化 `PolygonStamped`
- 新增 IMU 频率复现实验脚本

### 优化

- `MonitorDataStore` 加锁与重复缓存收敛
- 高频 topic 订阅改为更轻量的 QoS 适配策略
- `watch_topics` 对高频常见消息优先使用强类型订阅
- README 增加 topic 总表与 JSON 字段说明

### 说明

- 当前碰撞检测与上报器均已并入 `nav2_monitor` 包内部
- 保持低占用 / 小内存 / 快响应原则

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
