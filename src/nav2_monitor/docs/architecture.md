# nav2_monitor 架构设计说明

- 名称：nav2_monitor v2.3
- 版本：v2.3
- 时间：2026-03-15
- 作者：ToTo

## 1. 版本目标

当前版本的 `nav2_monitor` 已形成一条完整的轻量安全链路，目标是：

- 低占用
- 小内存
- 快响应
- 配置可扩展
- 故障检测与安全执行解耦

它不是单一的“状态上报节点”，而是一个由**监控采集、规则判断、故障组合、安全执行**组成的轻量安全监控系统。

---

## 2. 总体架构

当前实现按职责分为 6 层：

1. **输入采集层**：`Nav2MonitorNode`
2. **数据快照层**：`MonitorDataStore`
3. **规则判断层**：`WatchTopicEvaluator` / `FeedbackRuleEvaluator` / `ChassisEvaluator` / `CollisionEvaluator`
4. **故障编排层**：`FaultDetector`
5. **安全状态层**：`FaultStateCoordinator`
6. **动作执行层**：`safety_emergency_executor`

数据主链路如下：

`ROS 输入 -> Nav2MonitorNode -> MonitorDataStore -> FaultDetector -> FaultStateCoordinator -> SafetyCmd -> safety_emergency_executor`

同时还有两条输出链路：

- 状态输出链：`/nav2_monitor/status`
- 事件输出链：`/nav2_monitor/fault_event`

---

## 3. 各模块职责

### 3.1 Nav2MonitorNode

`Nav2MonitorNode` 是系统入口，负责：

- 加载参数与故障配置
- 建立所有 ROS 订阅/发布/定时器
- 周期扫描节点、话题发布者、TF
- 接收并预处理外部输入
- 将最新输入写入 `MonitorDataStore`
- 周期组装 `MonitorStatus`
- 触发故障检测与安全仲裁
- 发布 `FaultEvent` 与 `SupervisorCmd`
- 发布碰撞区域可视化

它负责“接线”和“驱动”，不负责具体规则判断。

### 3.2 MonitorDataStore

`MonitorDataStore` 是统一快照层，保存**最新有效状态**，不做业务判断。

当前保存的数据包括：

- 节点最后 seen 时间
- `watch_topics` 的发布者状态、有效位、频率、小窗口时间戳
- 统一反馈 `feedback` 的最近值、合法性、时间戳、小窗口时间戳
- 底盘输入状态：`/command`、`/moto_info`、`/odom`
- 电池状态
- 碰撞点集（按 source 分类缓存，并按超时聚合）

设计原则：

- 只保留最新值或很小的滑窗
- 不保留长历史
- 不做规则逻辑
- 尽量避免重复缓存

### 3.3 WatchTopicEvaluator

负责直接监控型 topic 的规则判断。

判断内容：

- 发布频率是否低于配置阈值
- 连续计数防误报
- 输出 legacy topic 类故障

适用对象：

- `watch_topics`

### 3.4 FeedbackRuleEvaluator

负责统一反馈规则判断。

判断内容：

- missing
- stale
- invalid
- 数值越界
- 频率不足
- 规则级安全动作覆盖
- 连续计数防误报

适用对象：

- `feedback_rules`
- 配置中的 `source_topic + metric_name`

### 3.5 ChassisEvaluator

负责底盘异常与久停判断。

判断内容：

- `command有 / moto无`
- `command无 / moto有`
- `command无 / moto无` 且持续超时
- 连续计数防误报

适用对象：

- `/command`
- `/moto_info`
- `/odom`

### 3.6 CollisionEvaluator

负责轻量碰撞检测。

当前支持：

- `LaserScan`
- `PointCloud2`
- `ultrasonic_eight(JSON)`
- polygon zone
- slowdown zone
- stop zone
- approach / TTC

判断方式：

- 将 scan / pointcloud 转换到 `base_frame` 下的 2D 点集
- 聚合当前有效 source 点集
- 针对每个 zone 做几何判定
- 输出统一 `FaultInfo`

当前 `approach / TTC` 是轻量版：

- 使用当前线速度近似
- 对 zone 内点估算 `distance / speed`
- 当最短 TTC 小于阈值时触发

### 3.7 FaultDetector

`FaultDetector` 现在是故障编排器，不再承担所有具体判断细节。

职责：

- 解析 `fault_detector_config.yaml`
- 保存配置对象
- 初始化各 evaluator
- 调用 evaluator，汇总当前激活故障 `FaultInfo[]`
- 生成稳定 `fault_key`

### 3.8 FaultStateCoordinator

`FaultStateCoordinator` 是安全状态机与边沿管理中心。

职责：

- 维护当前激活故障标志位
- 根据 `fault_key` 识别触发/恢复边沿
- 聚合多个安全故障
- 选择当前最高安全动作
- 发布 `/safety_system/cmd`
- 在所有安全故障恢复后自动发布 `RESUME`

当前安全动作优先级：

`EMERGENCY_STOP > SOFT_STOP > SLOW_DOWN > NORMAL`

### 3.9 safety_emergency_executor

执行端只负责动作执行：

- `SLOW_DOWN`
- `SOFT_STOP`
- `EMERGENCY_STOP`
- `RESUME`

它不参与故障判断，也不参与多故障组合。

---

## 4. 数据流与执行流

### 4.1 状态采集流

- 拓扑扫描阶段：
  - 节点在线状态
  - topic 发布者状态
  - TF 可用性
- 输入回调阶段：
  - feedback
  - command / moto / odom
  - battery
  - collision scan / pointcloud
- 所有输入统一写入 `MonitorDataStore`

### 4.2 故障检测流

- `check_health()` 时读取 `MonitorDataStore`
- `FaultDetector` 调用各 evaluator 输出 `FaultInfo[]`
- 每条 `FaultInfo` 带稳定 `fault_key`

### 4.3 安全仲裁流

- `FaultStateCoordinator` 接收 `FaultInfo[]`
- 识别：
  - 新出现故障 -> `TRIGGER`
  - 消失故障 -> `RECOVER`
- 聚合所有激活安全故障
- 选最高安全动作
- 发布 `/safety_system/cmd`

### 4.4 事件与状态输出流

- `MonitorStatus`：周期发布系统当前总体状态
- `FaultEvent`：只在触发/恢复边沿发布
- `SupervisorCmd`：按 cooldown 控制发布

---

## 5. 配置语义

### 5.1 `watch_topics`

表示 `nav2_monitor` 直接监控的真实 ROS topic。

用途：

- 检查发布者是否存在
- 检查频率是否达标

### 5.2 `feedback_rules`

表示统一反馈规则，而不是 `nav2_monitor` 直接订阅的真实 topic。

其中：

- `source_topic` 对应 `AlgorithmFeedback.topic_name`
- `metric_name` 对应 `AlgorithmFeedback.metric_name`

### 5.2 `target_transforms`

- 顶层 TF 监控列表，格式为 `frame1->frame2`
- 若 `fault_config` 中存在该字段，则优先使用；否则回退到 `nav2_monitor_params.yaml` 的 `target_transforms`

### 5.3 `collision_detection`

表示碰撞检测配置。

当前字段包括：

- `scan_topic`
- `pointcloud_topic`
- `ultrasonic_topic`
- `ultrasonic_widget`
  - 8 路权重，默认编号为 `1号左前，之后顺时针`
- `ultrasonic_blind_distance`
- `ultrasonic_out_of_range_value`
- `pointcloud_min_height`
- `pointcloud_max_height`
- `source_timeout_s`
- `zones`

### 5.4 `zones`

每个碰撞区支持：

- `name`
- `model`
  - 默认：普通命中区
  - `approach`
- `points`
- `min_points`
- `level`
- `safety_system`
- `safety_slow_down_percentage`
- `actions`
- `visualize`
- `polygon_pub_topic`
- `time_before_collision`

---

## 6. 当前已实现能力

### 已实现

- 节点在线监控
- `watch_topics` 频率监控
- `feedback_rules` 阈值/频率/超时/合法性监控
- 底盘异常与久停监控
- 多故障组合仲裁
- 自动恢复与 `RESUME`
- `LaserScan` 碰撞检测
- `PointCloud2` 碰撞检测
- 单 topic 八路超声波 JSON 加权碰撞检测
- polygon 区域可视化
- slowdown / stop / approach(TTC) 三类碰撞策略

### 尚未实现 / 未来优化点

- 更复杂的超声波场景策略（当前已支持单 topic 八路加权输入，推荐直接配 `ultrasonic_widget`）
- 完整 footprint 前向离散仿真版 TTC
- 碰撞 source 抽象层进一步统一
- 多错误组合策略表配置化
- 状态组装进一步独立成 `StatusAssembler`

---

## 7. 性能设计原则

本版本始终遵守以下原则：

- 低占用
- 小内存
- 快响应

具体体现：

- 最新快照驱动，而不是长历史驱动
- 小窗口时间戳缓存
- evaluator 做局部轻量判断
- `FaultStateCoordinator` 统一安全仲裁，避免多处重复逻辑
- 碰撞检测使用轻量几何判定，而不是重型地图/规划推演

---

## 8. 推荐阅读顺序

建议从以下顺序理解代码：

1. `Nav2MonitorNode`
2. `MonitorDataStore`
3. `FaultDetector`
4. 各 evaluator
5. `FaultStateCoordinator`
6. `safety_emergency_executor`

推荐入口文件：

- `include/nav2_monitor/nav2_monitor_node.hpp`
- `include/nav2_monitor/monitor_data_store.hpp`
- `include/nav2_monitor/fault_detector.hpp`
- `include/nav2_monitor/fault_state_coordinator.hpp`
- `include/nav2_monitor/collision_evaluator.hpp`
- `src/safety_emergency_executor/src/safety_emergency_executor_node.cpp`
