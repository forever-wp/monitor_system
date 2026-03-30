# NavigationStatus.msg - 导航状态消息
#
# 描述导航任务的详细状态信息
# 用于导航状态查询和监控

# 导航状态常量定义
uint8 PENDING = 0           # 等待中 - 任务已创建但未开始
uint8 PLANNING = 1          # 规划中 - 正在计算路径
uint8 RUNNING = 2           # 执行中 - 正在导航
uint8 SUCCEEDED = 3         # 成功 - 导航任务成功完成
uint8 FAILED = 4            # 失败 - 导航任务失败
uint8 CANCELLED = 5         # 已取消 - 导航任务被取消
uint8 PAUSED = 6            # 暂停中 - 导航任务暂停

# 任务ID
string task_id

# 任务状态
uint8 status

# 目标位姿
geometry_msgs/PoseStamped target_pose

# 当前位姿
geometry_msgs/PoseStamped current_pose

# 剩余距离 (米)
float64 remaining_distance

# 预计剩余时间 (秒)
float64 estimated_time_remaining

# 导航进度 (0.0 - 1.0)
float64 progress

# 当前使用的规划器ID
string current_planner_id

# 当前使用的控制器ID
string current_controller_id

# 任务创建时间
builtin_interfaces/Time created_time

# 任务开始时间
builtin_interfaces/Time started_time

# 任务完成时间 (如果已完成)
builtin_interfaces/Time completed_time

# 错误信息 (如果失败)
string error_message

# 航点类型 (用于业务逻辑识别)
string waypoint_type

# 导航相关的额外数据 (JSON格式)
string navigation_data
