# 故障事件
builtin_interfaces/Time stamp
string module_name

# 故障级别
uint8 NORMAL = 0
uint8 WARNING = 1
uint8 ERROR = 2
uint8 CRITICAL = 3
uint8 fault_level

# 动作类型
uint8 NONE = 0
uint8 SUPERVISOR = 1
uint8 SAFETY_SYSTEM = 2
uint8 action

# 边沿类型
uint8 EDGE_NONE = 0
uint8 EDGE_TRIGGER = 1
uint8 EDGE_RECOVER = 2
uint8 edge

string reason
