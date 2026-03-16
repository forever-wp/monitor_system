# 监控状态消息
bool all_ok                    # 总体状态

# 节点监控
string[] monitored_nodes       # 监控的节点列表
string[] active_nodes          # 活跃的节点
string[] timeout_nodes         # 超时的节点

# 话题监控
string[] monitored_topics      # 监控的话题列表
string[] active_topics         # 有发布者的话题
string[] inactive_topics       # 无发布者的话题
float32[] topic_frequencies    # 话题频率(Hz)，与monitored_topics对应

# 系统资源
float32 cpu_usage              # CPU使用率 (%)
float32 mem_usage              # 内存使用率 (%)
float32 disk_usage             # 磁盘使用率 (%)
float32 cpu_temp               # CPU温度 (°C)
float32 gpu_usage              # GPU使用率 (%, -1表示无GPU)
float32 gpu_temp               # GPU温度 (°C, -1表示无GPU)
float32 gpu_mem_usage          # GPU显存使用率 (%, -1表示无GPU)

# 小车状态
bool vehicle_status_valid
bool vehicle_navigation_active
bool vehicle_navigation_succeeded
float32 vehicle_progress_percentage
string vehicle_simple_status
string vehicle_error_message

# 电池状态 (/battery_state)
float32 battery_temperature
float32 battery_percentage

# TF变换监控
string[] monitored_transforms      # 监控的变换列表 (格式: "frame1->frame2")
string[] available_transforms      # 可用的变换
string[] stale_transforms          # 过期的变换
float32[] transform_latencies_ms   # 变换延迟(ms)

# 网络延迟
float32[] topic_latencies_ms       # 话题延迟(ms)
