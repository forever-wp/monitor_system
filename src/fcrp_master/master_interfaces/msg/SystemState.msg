# SystemState.msg - 系统状态消息
#
# 定义机器人系统的宏观状态，用于状态机管理
# 该消息描述了系统当前所处的主要状态

# 状态常量定义
uint8 UNKNOWN = 0           # 未知状态
uint8 INITIALIZING = 1      # 初始化中
uint8 IDLE = 2              # 空闲状态 - 系统已就绪但未执行任务
uint8 ACTIVE = 3            # 激活状态 - 系统已激活，可以接收任务
uint8 NAVIGATING = 4        # 导航中 - 正在执行导航任务
uint8 MAPPING = 5           # 建图中 - 正在执行SLAM建图
uint8 LOCALIZING = 6        # 定位中 - 正在执行重定位
uint8 CHARGING = 7          # 充电中 - 机器人正在充电
uint8 ERROR = 8             # 错误状态 - 系统出现错误需要处理
uint8 EMERGENCY_STOP = 9    # 紧急停止 - 系统紧急停止
uint8 MAINTENANCE = 10      # 维护模式 - 系统处于维护状态
uint8 SHUTDOWN = 11         # 关机中 - 系统正在关闭

# 当前系统状态
uint8 current_state

# 前一个系统状态
uint8 previous_state

# 状态描述信息
string state_description

# 状态进入时间
builtin_interfaces/Time state_entered_time

# 状态持续时间 (秒)
float64 state_duration

# 状态相关的额外信息 (JSON格式)
string state_data
