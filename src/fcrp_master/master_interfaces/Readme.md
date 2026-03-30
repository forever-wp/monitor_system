# CRP Interfaces

---

## 目录结构概览
```
master_interfaces/
├── msg/    # 消息定义 (6)
├── srv/    # 服务定义 (4)
├── test/   # GTest 单元测试
└── *       # 构建/元数据文件
```
> **提示**：在工作区根目录执行 `ros2 interface list | grep master_interfaces` 可查看实时接口列表。

---

## 主要接口速查

### 消息（Msg）
| 文件                     | 作用           | 关键字段 / 常量                                               |
| ------------------------ | -------------- | ------------------------------------------------------------- |
| **ErrorCode.msg**        | 标准错误码     | `code` `description`  <br> 0 SUCCESS / 1001 NETWORK_ERROR ... |
| **RobotState.msg**       | 机器人整体状态 | 位姿、电池、电机、IO 状态                                     |
| **SystemState.msg**      | 系统宏观状态   | `current_state` `previous_state` `state_description`          |
| **NavigationStatus.msg** | 导航任务状态   | `task_id` `status` `progress`                                 |
| **StateTransition.msg**  | 状态转换记录   | from → to + 时间戳                                            |
| **StandardResponse.msg** | 通用响应封装   | `error_code` `stamp`                                          |

### 服务（Srv）
| 文件                        | 功能         | 请求 → 响应                                                 |
| --------------------------- | ------------ | ----------------------------------------------------------- |
| **Ping.srv**                | 连通性测试   | 无 → `StandardResponse`                                     |
| **NavigateToPose.srv**      | 导航到位姿   | 目标 `PoseStamped` → `task_id` + `StandardResponse`         |
| **CancelNavigation.srv**    | 取消导航     | `task_id` `reason` → `StandardResponse`                     |
| **GetNavigationStatus.srv** | 查询导航状态 | `task_id`(可空) → `NavigationStatus[]` + `StandardResponse` |

---

## 快速开始
```bash
# 1) 编译接口包
cd robot_fc_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select master_interfaces
source install/setup.bash

# 2) 查看接口
ros2 interface show master_interfaces/msg/SystemState
ros2 interface show master_interfaces/srv/NavigateToPose
```

---

## 常用调试/测试命令
```bash
# 列出所有接口
ros2 interface list | grep master_interfaces

# 调用导航服务示例
ros2 service call /navigate_to_pose master_interfaces/srv/NavigateToPose "{target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}, planner_id: 'NavfnPlanner', controller_id: 'FollowPath', timeout: 60.0}"

# 查询导航状态
ros2 service call /get_navigation_status master_interfaces/srv/GetNavigationStatus "{task_id: ''}"

# 发布系统状态
ros2 topic pub /system_state master_interfaces/msg/SystemState "{current_state: 3, previous_state: 2, state_description: '系统激活'}"
```

---

## 单元测试
```bash
colcon test --packages-select master_interfaces
colcon test-result --all --verbose
```

---