# 标准响应消息格式

# 操作是否成功
bool success

# 错误信息（success=false时使用）
ErrorCode error_code

# 响应时间戳
builtin_interfaces/Time timestamp

# 可选的额外数据
string data 

# 可选的额外消息
string message