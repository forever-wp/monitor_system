# 标准错误码定义
int32 SUCCESS = 0                # 操作成功
int32 UNKNOWN_ERROR = 1000       # 未知错误
int32 NETWORK_ERROR = 1001       # 网络通信错误
int32 INVALID_PARAMETER = 1002   # 无效参数
int32 SERVICE_UNAVAILABLE = 1003 # 服务不可用
int32 TIMEOUT = 1004             # 操作超时
int32 RESOURCE_NOT_FOUND = 1005  # 资源不存在
int32 PERMISSION_DENIED = 1006   # 权限不足
int32 HARDWARE_ERROR = 2000      # 硬件错误
int32 NAVIGATION_ERROR = 3000    # 导航相关错误

# 当前错误码
int32 code
# 错误详细描述
string description 