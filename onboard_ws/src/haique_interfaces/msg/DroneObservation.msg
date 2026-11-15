# 海雀机载系统 17 维策略输入消息（50Hz）
#
# 坐标系契约：
# - 世界系：默认 ENU（ROS REP-103，x 东、y 北、z 上）；训练侧若使用其他约定，
#   必须在 INTERFACE_CONTRACT.md 中显式对齐，不能混用。
# - 机体系：FLU（前-左-上）；header.frame_id 必须是 base_link_flu。
# - 上游若来自 PX4（NED 世界系 / FRD 机体系），必须先转换成 ENU / FLU 再填本消息。
#
# 时间契约：
# - header.stamp 必须是观测采样时刻，不是 publish 时刻。
# - (0, 0) 表示未打戳：消费方退化为按到达时刻判超时，端到端延迟填 -1。
#
# 17 维策略输入由本消息字段按固定顺序拼装，顺序见 docs/DESIGN.md；
# 拼装唯一入口是 policy_runner_node，别处不得重复拼装。
std_msgs/Header header

# 姿态四元数：body(FLU) -> world(ENU)，Hamilton 约定。
# geometry_msgs/Quaternion 的字段定义顺序是 x, y, z, w；
# 拼进 17 维时按 [w, x, y, z]，不要按消息下标取。
# 模长必须为 1；policy_runner_node 以 1e-2 容差校验，超出报 INVALID_OBSERVATION。
geometry_msgs/Quaternion orientation

# 机体系线速度 (m/s, FLU)
geometry_msgs/Vector3 body_linear_velocity

# 机体系角速度 (rad/s, FLU)
geometry_msgs/Vector3 body_angular_velocity

# 目标点在机体系下的相对位置误差 (m, FLU)：目标点 - 当前点。
# 若目标点/当前点在世界系给出，等价于 R_wb^T * (p_target_w - p_current_w)。
geometry_msgs/Point rel_position_error

# 上一帧策略节点发布的动作（commanded action，已限幅，FLU）：
# [roll_rate, pitch_rate, yaw_rate, collective_thrust]
# 前三个单位 rad/s；thrust 为归一化推力，范围 [min_thrust, max_thrust]。
# 注意：这是指令值，不是飞控回读的实际执行值；bridge 接通前动作不会到达 PX4。
# 首帧没有历史动作时由发布方给出初始猜测（mock 用 hover_thrust），不是真实动作。
float32[4] last_action

# TODO(@wang): 以下属于 schema 变更，暂缓；先与训练仓库 docs/INTERFACE_CONTRACT.md 对齐：
# 1. rel_position_error 用 Point 表示位移，语义上更适合 geometry_msgs/Vector3；
# 2. last_action 若要改名为 last_commanded_action，需同步修改训练侧；
# 3. 增加 schema_version 字段，用于跨仓库契约版本化。
