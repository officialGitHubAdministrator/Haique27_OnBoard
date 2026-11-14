# 海雀机载系统 17 维观测消息（50Hz）
# 所有物理量均在 FLU 机体系（前-左-上）下表达。
# 17 维扁平向量的拼装顺序见 docs/DESIGN.md，由 policy_runner_node 统一负责。

# header.frame_id 当前不做校验，仅透传；契约以 FLU 机体系为准。
std_msgs/Header header

# 姿态四元数 [w, x, y, z]，机体坐标系到世界坐标系
geometry_msgs/Quaternion orientation

# 机体系线速度 (m/s)
geometry_msgs/Vector3 body_linear_velocity

# 机体系角速度 (rad/s)
geometry_msgs/Vector3 body_angular_velocity

# 目标点在机体系下的相对位置误差 (m)：目标点 - 当前点
geometry_msgs/Point rel_position_error

# 上一步实际下发的动作：[roll_rate, pitch_rate, yaw_rate, thrust]
float32[4] last_action
