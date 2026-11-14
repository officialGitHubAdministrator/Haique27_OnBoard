# 17-Dimensional Observation Vector for Haique27 RL Policy (FLU Frame)
# 对应接口契约 INTERFACE_CONTRACT.md

# 1. 当前姿态四元数 (Hamilton 约定: [w, x, y, z])
float32[4] q_body

# 2. 机体系线速度 (FLU 体系: [vx, vy, vz], 单位 m/s)
float32[3] v_body

# 3. 机体系角速度 (FLU 体系: [wx, wy, wz], 单位 rad/s)
float32[3] w_body

# 4. 相对目标路点位置偏差 (机体系 FLU: [dx, dy, dz], 单位 m)
float32[3] pos_err_body

# 5. 上一帧动作 (上一时刻输出的 [roll_rate, pitch_rate, yaw_rate, thrust])
float32[4] last_act
