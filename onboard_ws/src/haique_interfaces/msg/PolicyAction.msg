# 海雀机载系统 - 强化学习策略 4 维动作输出消息
# 坐标系说明: FLU 机体系

std_msgs/Header header

# 1. 期望三轴机体角速度 (单位: rad/s, FLU系: roll_rate, pitch_rate, yaw_rate)
geometry_msgs/Vector3 desired_body_rates

# 2. 归一化期望总推力 [0.0 ~ 1.0] (对应 PX4 collective thrust)
float32 collective_thrust

# 3. 原始网络未裁剪输出 (便于日志追溯与真机调参诊断)
# 兜底/降级帧里填的是实际下发的安全动作 (不含 NaN)，不是网络原始输出。
float32[4] raw_action

# 4. 推理回调内部耗时 (仅用于评估推理引擎，不含通信)
# 看门狗超时下发安全动作时为 -1.0 (没有对应观测)。
float32 inference_latency_ms

# 5. 观测 header.stamp 到 publish 调用前的延迟 (真正该盯的指标，不含发布耗时)
# 负值表示无法测量 (观测时间戳为 0、时钟回跳、超出 1000ms 上界，
# 或该帧由看门狗超时兜底下发)。
float32 end_to_end_latency_ms
