# 海雀机载系统 - 状态与安全监控消息
#
# 本话题的唯一发布者是 policy_runner_node。为防止 DDS 双发布者互相覆盖，
# 未来真正发送 setpoint 的节点若要上报 offboard_enabled，不得与 policy 同时
# 发布本话题，应改用独立话题或服务。

std_msgs/Header header

# 整体系统健康状态 (true: 允许策略控制; false: 强制降级或接管)
bool is_safe

# 是否处于强化学习 Offboard 控制激活状态
# 语义上属于 px4_rates_bridge: 只有真正向 PX4 发送 setpoint 时才可置 true。
# 但本话题的唯一发布者是 policy_runner_node，且 bridge 目前不发送 setpoint，
# 因此当前恒为 false。接入 PX4 前需给 bridge 单独开状态话题/服务，或由 policy
# 订阅 bridge 状态后再填写，不能出现两个发布者。
bool offboard_enabled

# 是否检测到需要上层介入的紧急故障（急停/返航/抢权由上层决定并执行；
# 当前 policy_runner_node 只下发悬停安全动作，不执行这三种动作）。
# 仅 "OBS_TIMEOUT" 与 "PIPELINE_EXCEPTION" 两类真故障置 true;
# 非有限输入与降级模式已输出安全动作，不算紧急。
# OBS_TIMEOUT 期间 policy_runner_node 会以 20Hz 主动向 /haique/policy_action
# 下发悬停安全动作 (零角速度 + 限幅后的悬停推力)，让看门狗执行动作。
# 仅在此前收到过观测后生效，纯上电阶段不下发，避免向飞控灌 setpoint。
bool emergency_triggered

# 状态码或诊断提示 (如 "NORMAL", "WAITING_OBSERVATION", "BASELINE_FALLBACK",
# "NONFINITE_OBSERVATION", "INVALID_OBSERVATION", "NONFINITE_ACTION",
# "OBS_TIMEOUT", "PIPELINE_EXCEPTION")
# INVALID_OBSERVATION 表示 frame_id 不是 base_link_flu，或四元数模长偏离 1。
string status_code
string message
