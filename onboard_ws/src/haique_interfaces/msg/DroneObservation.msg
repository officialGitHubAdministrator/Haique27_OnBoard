# 海雀机载系统 - 17维统一观测向量消息定义
# 坐标系说明: 所有机体物理量均转换至 FLU (前-左-上, Front-Left-Up) 机体系

std_msgs/Header header

# 1. 目标点在机体系下的相对位置误差 (单位: m) [3维]
geometry_msgs/Point rel_position_error

# 2. 机体当前姿态四元数 (FLU系机体相对于地面参考系) [4维, w, x, y, z]
geometry_msgs/Quaternion orientation

# 3. 机体线速度 (FLU系, 单位: m/s) [3维]
geometry_msgs/Vector3 body_linear_velocity

# 4. 机体角速度 (FLU系, 单位: rad/s) [3维]
geometry_msgs/Vector3 body_angular_velocity

# 5. 上一时刻策略动作 (3轴角速度 + 标一化推力) [4维]
float32[4] last_action

# 6. 扁平化 17 维连续数组 (方便直接转换张量输入 ONNX/TensorRT 模型)
# 对应拼接顺序: [rel_pos(3), quat(4), lin_vel(3), ang_vel(3), last_action(4)]
float32[17] flat_obs
