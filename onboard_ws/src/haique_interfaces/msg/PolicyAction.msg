# 海雀机载系统 - 强化学习策略 4 维动作输出消息
# 坐标系说明: FLU 机体系

std_msgs/Header header

# 1. 期望三轴机体角速度 (单位: rad/s, FLU系: roll_rate, pitch_rate, yaw_rate)
geometry_msgs/Vector3 desired_body_rates

# 2. 标一化期望总推力 [0.0 ~ 1.0] (对应 PX4 collective thrust)
float32 collective_thrust

# 3. 原始网络未裁剪输出 (便于日志追溯与真机调参诊断)
float32[4] raw_action

# 4. 单步推理耗时 (毫秒 ms)
float32 inference_latency_ms
