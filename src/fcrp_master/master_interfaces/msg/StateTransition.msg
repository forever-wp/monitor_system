# StateTransition.msg - 状态转换消息
#
# 描述系统状态转换的详细信息
# 用于状态机的状态转换记录和调试

# 转换类型常量
uint8 AUTOMATIC = 0         # 自动转换
uint8 MANUAL = 1            # 手动触发转换
uint8 EVENT_TRIGGERED = 2   # 事件触发转换
uint8 ERROR_TRIGGERED = 3   # 错误触发转换
uint8 TIMEOUT_TRIGGERED = 4 # 超时触发转换

# 转换结果常量
uint8 SUCCESS = 0           # 转换成功
uint8 FAILED = 1            # 转换失败
uint8 REJECTED = 2          # 转换被拒绝
uint8 PENDING = 3           # 转换进行中

# 转换ID (唯一标识符)
string transition_id

# 源状态
uint8 from_state

# 目标状态
uint8 to_state

# 转换类型
uint8 transition_type

# 转换结果
uint8 transition_result

# 触发转换的事件或条件
string trigger_event

# 转换开始时间
builtin_interfaces/Time transition_start_time

# 转换完成时间
builtin_interfaces/Time transition_end_time

# 转换持续时间 (毫秒)
int64 transition_duration_ms

# 转换原因或描述
string reason

# 转换失败时的错误信息
string error_message

# 转换相关的额外数据 (JSON格式)
string transition_data
