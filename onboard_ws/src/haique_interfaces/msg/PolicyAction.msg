# 海雀机载系统 - 强化学习策略 4 维动作输出消息
# 坐标系说明: FLU 机体系

std_msgs/Header header

# 1. 期望三轴机体角速度 (单位: rad/s, FLU系: roll_rate, pitch_rate, yaw_rate)
geometry_msgs/Vector3 desired_body_rates

# 2. 归一化期望总推力 [0.0 ~ 1.0] (对应 PX4 collective thrust)
float32 collective_thrust

# 3. 原始网络未裁剪输出 (便于日志追溯与真机调参诊断)
# 安全兜底帧（非有限观测/动作、看门狗超时、管线异常）里填的是实际下发的安全动作
# (不含 NaN)；内置控制器模式（BASELINE_NO_MODEL/BASELINE_FALLBACK）里填的是
# 限幅前的内置控制器输出，不是安全动作。
float32[4] raw_action

# 4. 策略回调内处理耗时 (从进入 observation_callback 到构造动作消息前，
#    含观测校验、拼装、推理、限幅；不含消息构造与 DDS 通信)
# 看门狗超时下发安全动作时为 -1.0 (没有对应观测)。
float32 inference_latency_ms

# 5. 观测 header.stamp 到 publish 调用前的延迟 (真正该盯的指标，不含发布耗时)
# 负值表示无法测量 (观测时间戳为 0、时钟回跳、超出 1000ms 上界，
# 或该帧由看门狗超时兜底下发)。
float32 end_to_end_latency_ms
