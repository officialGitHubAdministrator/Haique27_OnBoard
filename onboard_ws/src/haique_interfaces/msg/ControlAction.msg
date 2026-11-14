# 4-Dimensional Control Action Vector for Haique27 Quadcopter
# 对应接口契约 INTERFACE_CONTRACT.md

# 1. 期望横滚角速度 (FLU 体系, 单位 rad/s)
float32 roll_rate_cmd

# 2. 期望俯仰角速度 (FLU 体系, 单位 rad/s)
float32 pitch_rate_cmd

# 3. 期望偏航角速度 (FLU 体系, 单位 rad/s)
float32 yaw_rate_cmd

# 4. 归一化总推力 (范围 [0.0, 1.0], 0.28 对应悬停推力)
float32 collective_thrust
