# nav2_monitor 架构图（流程图 / 数据流图 / 功能图）

## 1) 流程图（运行流程）

```mermaid
flowchart TD
    A[节点启动 Nav2MonitorNode] --> B[加载参数]
    B --> C[加载 fault_config 到 FaultDetector]
    C --> D{modules 有效?}
    D -- 是 --> E[监控目标=modules 汇总 nodes/topics]
    D -- 否 --> F[回退 target_nodes/target_topics]
    E --> G[初始化发布器/订阅器/定时器]
    F --> G
    G --> H[scan_topology 定时循环]
    G --> I[check_health 定时循环]

    H --> H1[扫描节点在线状态]
    H --> H2[更新话题发布者状态]
    H --> H3[查询 TF 可用性与延迟]
    H --> H4[动态订阅监控话题]
    H --> H5[尝试订阅 /moto_info]

    I --> I1[读取 VehicleStatus 文件]
    I --> I2[采集系统资源指标]
    I --> I3[聚合节点/话题/TF 状态]
    I3 --> I4[更新 FaultDetector 输入]
    I4 --> I5[detect_faults]
    I5 --> I6[按 module+action 执行 cooldown]
    I6 --> I7[发布 /nav2_monitor/status]
    I6 --> I8[发布 /nav2_monitor/fault_event]
    I6 --> I9[发布 /supervisor/cmd 或 /safety_system/cmd]
```

## 2) 数据流图（Data Flow）

```mermaid
flowchart LR
    subgraph Inputs[外部输入]
      FC[fault_detector_config.yaml]
      AF[/nav2_monitor/algorithm_feedback]
      CMD[/command]
      MOTO[/moto_info]
      ODOM[/odom]
      VS[vehicle_status_file]
      SYS[/proc + /sys + NVML]
      ROSG[ROS Graph + TF]
    end

    subgraph Core[nav2_monitor 核心]
      N[Nav2MonitorNode]
      FD[FaultDetector]
      VM[VehicleStatusMonitor]
      SM[SystemMonitor]
      MV[MultiValueJudge<br/>trigger/recover]
      CD[Action Cooldown<br/>module+action]
    end

    subgraph Outputs[输出]
      ST[/nav2_monitor/status]
      FE[/nav2_monitor/fault_event]
      SP[/supervisor/cmd]
      SA[/safety_system/cmd]
    end

    FC --> FD
    ROSG --> N
    AF --> N
    CMD --> N
    MOTO --> N
    ODOM --> N
    VS --> VM --> N
    SYS --> SM --> N

    N -->|node_status/topic_freq + feedback + chassis state| FD
    FD --> MV --> N
    N --> CD --> SP
    N --> CD --> SA
    N --> FE
    N --> ST
```

## 3) 功能图（模块功能分解）

```mermaid
flowchart TB
    ROOT[nav2_monitor]

    ROOT --> M1[Nav2MonitorNode]
    ROOT --> M2[FaultDetector]
    ROOT --> M3[VehicleStatusMonitor]
    ROOT --> M4[SystemMonitor]

    M1 --> M11[拓扑扫描<br/>节点/话题/TF]
    M1 --> M12[状态发布<br/>/nav2_monitor/status]
    M1 --> M13[事件与动作执行<br/>fault_event/supervisor/safety]
    M1 --> M14[动作冷却抑制<br/>safety_cooldown / supervisor_cooldown]
    M1 --> M15[输入接入<br/>algorithm_feedback + command/moto/odom]

    M2 --> M21[节点存活判断<br/>快速路径]
    M2 --> M22[feedback_topics 阈值判断]
    M2 --> M23[legacy topics 频率判断]
    M2 --> M24[chassis_stationary 判断]
    M2 --> M25[连续计数防误报<br/>trigger_count / recover_count]
    M2 --> M26[监控目标导出<br/>聚合 nodes/topics]

    M3 --> M31[导航状态文件解析]
    M3 --> M32[小车状态字段输出]

    M4 --> M41[CPU/内存/磁盘/温度]
    M4 --> M42[GPU 使用率/温度/显存]
```

