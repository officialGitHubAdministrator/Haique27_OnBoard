# 海雀机载系统 - 状态与安全监控消息

std_msgs/Header header

# 整体系统健康状态 (true: 允许策略控制; false: 强制降级或接管)
bool is_safe

# 是否处于强化学习 Offboard 控制激活状态
bool offboard_enabled

# 是否触发紧急停车/返航/抢权
bool emergency_triggered

# 状态码或诊断提示 (如 "NORMAL", "RC_OVERRIDE", "OBSERVATION_TIMEOUT", "EXCESSIVE_TILT")
string status_code
string message
