# RobotState.msg - 机器人综合状态信息
#
# 提供机器人在某一时刻的完整运行状态，用于监控与调试。
# 该消息与 master_SDK_server/RobotStateServer 实现保持一致。

# 标准头部（时间戳 + 坐标系）
std_msgs/Header header

# 机器人唯一标识（一般使用节点名或序列号）
string robot_id

# 机器人位姿（包含协方差）
geometry_msgs/PoseWithCovarianceStamped pose

# 机器人速度（包含协方差）
geometry_msgs/TwistWithCovariance twist

# 电池电量百分比 (0-100)
float64 battery_percentage

# 电池电压 (V)
float64 battery_voltage

# 当前是否在充电
bool is_charging

# 系统状态 (参考 master_interfaces/msg/SystemState 常量)
uint8 system_state

# 系统状态文字描述
string state_description

# 机器人已运行时间 (秒)
float64 uptime

# 健康状态文字描述，如 "健康"、"警告: 电池电量低"
string health_status

# 与平台（MQTT / ROS2 等）的连接状态
bool is_connected

# 错误信息（非空表示出现异常）
string error_message

# ==== 保留字段，便于向后兼容扩展 ====
# string reserved_1
# string reserved_2
# ==================================== 