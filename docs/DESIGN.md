# 海雀机载系统设计说明

本仓库只做机载外层控制：50Hz、非实时，17 维观测进，4 维动作出。
跨仓库接口以训练仓库的 `docs/INTERFACE_CONTRACT.md` 为基准；训练仓库单独维护，不在本仓库内，
两边改动必须同步。该 contract 已按 v4 与本仓库实现对齐（2026-11-15）；再改接口时先更新 contract。
代码导读见 [CODE_GUIDE.md](CODE_GUIDE.md)。

## 1. 系统边界

多旋翼控制分两层。内层在 PX4 飞控板上，频率远高于外层，负责陀螺仪滤波、角速度 PID、
电机混控和失控保护。外层在机载 Linux 上，50Hz，决定期望角速度和归一化总推力。

本仓库不输出电机 PWM：

- 机载 Linux 不是实时系统，Python GC 和调度抖动无法保证 1kHz 级周期。
- 电机混控依赖电池电压、桨叶差异和电机安装方向，这些由飞控侧处理。
- 输出角速度时，外层异常仍要经过飞控的角速度环；直接给 PWM 没有中间层兜底。

代价是外层不能直接指定姿态角，只能给角速度；推力是相对最大推力的归一化比例，换桨、换电池后需要重新标定。

## 2. 分层架构与数据流

```text
[外层] 机载 Linux，本仓库，50Hz，非实时
  mock_telemetry_publisher --/haique/observation (50Hz, FLU, 拼成 17 维)--> policy_runner_node
  policy_runner_node       --/haique/policy_action (正常 50Hz；超时 20Hz 安全动作)--> px4_rates_bridge
  policy_runner_node       --/haique/policy_action--> mock_telemetry_publisher（只回填 last_action）
  policy_runner_node       --/haique/safety_status (20Hz)--> 当前无订阅者
  px4_rates_bridge         --FLU 转 FRD--> 换算结果丢弃，未接 PX4
        │ 目标链路，未实现，需要 px4_msgs 与 Micro-XRCE-DDS
        ▼
[内层] PX4 飞控，数百 Hz 以上，由飞控保证实时性
  输入：三轴期望角速度 (FRD) + 归一化推力 (thrust_body.z = -τ)
  输出：四路电机控制信号（PWM/DShot 等）；反馈：/fmu/out/vehicle_odometry (FRD/NED，未接入)
```

| 话题 | 消息类型 | 频率 | 发布者 | 订阅者 | 状态 |
|---|---|---|---|---|---|
| `/haique/observation` | `DroneObservation` | 50Hz（mock 默认） | `mock_telemetry_publisher`；真实状态估计未实现 | `policy_runner_node` | 已实现，内容是三角函数模拟 |
| `/haique/policy_action` | `PolicyAction` | 默认 50Hz；超时期间 20Hz 安全动作 | `policy_runner_node` | `px4_rates_bridge`、`mock_telemetry_publisher` | 已实现 |
| `/haique/safety_status` | `SafetyStatus` | 20Hz | `policy_runner_node` | 无 | 已实现，`offboard_enabled` 恒为 false |
| `/fmu/out/vehicle_odometry` | `px4_msgs` 消息 | 未实现 | 无 | 无 | 未实现，仓库内没有 `px4_msgs` |
| `/fmu/in/vehicle_rates_setpoint` | `px4_msgs` 消息 | 未实现 | 无 | 无 | 未实现，bridge 只换算不发送 |

`mock_telemetry_publisher` 是节点名，对应的可执行文件是 `mock_telemetry_pub`。
它订阅动作只为回填 `last_action`，不按动作更新姿态或速度，链路是开环。
`/haique/safety_status` 的唯一发布者是 `policy_runner_node`。`offboard_enabled` 在语义上属于
`px4_rates_bridge`，但 bridge 尚未发送 setpoint，因此当前恒为 false；接通 PX4 时不能让两个节点同时发布该话题。

## 3. 接口契约

### 3.1 17 维观测

观测顺序冻结，不得改动。向量类物理量在 FLU 机体系下表达；姿态四元数是 body(FLU) 到 world(ENU) 的旋转。

| 索引 | 内容 | 单位 | 说明 |
|---|---|---|---|
| 0..3 | 姿态四元数 `[w, x, y, z]` | 无量纲，模长 1 | body(FLU) 到 world(ENU)，Hamilton 约定，实部在前；模长容差 1e-2 |
| 4..6 | 机体系线速度 `[vx, vy, vz]` | m/s | FLU |
| 7..9 | 机体系角速度 `[wx, wy, wz]` | rad/s | FLU |
| 10..12 | 目标相对位置误差 `[dx, dy, dz]` | m | FLU，目标点减当前点；世界系计算时即 `R_wb^T (p_target_w - p_current_w)` |
| 13..16 | 上一帧策略节点发布的动作 `[rx, ry, rz, thrust]` | rad/s, rad/s, rad/s, 归一化 | commanded action，已限幅；不是飞控回读值 |

`last_action` 通常是上一帧策略节点发布的动作；首帧没有历史动作，mock 用 `hover_thrust` 作为初始猜测，不是真实动作。

拼装与校验规则：

- 17 维只在 `policy_runner_node._build_observation` 拼装一次，返回长度 17 的 tuple，别处不得重复拼装；
  消息里没有 17 维扁平数组字段。
- `geometry_msgs/Quaternion` 的字段定义顺序是 `x, y, z, w`，必须显式写 `[q.w, q.x, q.y, q.z]`，不能按下标取。
- 观测消息的 `header.frame_id` 必须为 `base_link_flu`；不匹配报 `INVALID_OBSERVATION` 并下发安全动作，
  避免把 FRD 数据当 FLU 用。
- 四元数各分量有限时，模长必须为 1（容差 1e-2）；超出报 `INVALID_OBSERVATION`。含非有限值时由后面的有限性检查报 `NONFINITE_OBSERVATION`。
- 任一元素非有限时不得推理，报 `NONFINITE_OBSERVATION` 并下发安全动作。
- `header.stamp` 是观测采样时刻，不是发布时刻。`policy_runner_node` 用它计算端到端延迟与看门狗年龄；
  `(0, 0)` 表示未打戳，此时端到端延迟填 -1，看门狗退化为按到达时刻判超时。
- 送进 ONNX 前转成 float32 并 reshape 为 `(1, 17)`；float32 转换后若溢出为非有限，同样跳过推理并报 `NONFINITE_OBSERVATION`。

### 3.2 4 维动作

`PolicyAction.header.frame_id` 固定为 `base_link_flu`。

| 字段 | 含义 | 单位与范围 |
|---|---|---|
| `desired_body_rates` | 期望三轴角速度，FLU | rad/s，单轴限幅 ±`max_tilt_rate` |
| `collective_thrust` | 归一化总推力 | 限幅到 `[min_thrust, max_thrust]` |
| `raw_action` | 正常路径与内置控制器模式为限幅前原始输出；兜底路径等于实际下发的安全动作 | 仅用于诊断，兜底时不会是 NaN |
| `inference_latency_ms` | 从进入 `observation_callback` 到 `_publish_action` 构造消息前的处理耗时 | ms，见下 |
| `end_to_end_latency_ms` | 观测 `header.stamp` 到 `publish()` 调用前的延迟 | ms，`< 0` 表示不可测 |

`inference_latency_ms` 在 `_publish_action` 填充消息前用 `perf_counter` 读取，覆盖观测拼装、
契约校验、非有限检查、推理和限幅，不含消息构造与 DDS 发布。看门狗超时下发的安全动作没有对应观测，
该字段填 -1.0。

`end_to_end_latency_ms` 的采样点紧贴 `publish()`。时间戳为 0、时间戳在未来、超过 1000ms 上界，
或该帧由看门狗兜底下发时，填 -1.0。

限幅前必须先判有限。CPython 的 `max(lo, min(hi, nan))` 返回上限，NaN 会被放行成满量程角速度；
代码在观测、推理输出两处先检查非有限值，再进入 `_clamp`。

### 3.3 安全状态

引擎模式 `engine` 取 `ONNX`、`BASELINE_NO_MODEL`、`BASELINE_FALLBACK`；`NORMAL` 与
`BASELINE_FALLBACK` 的 `message` 里带 `engine=<模式>`。

| `status_code` | 触发条件 | `is_safe` | `emergency_triggered` |
|---|---|---|---|
| `WAITING_OBSERVATION` | 启动后尚未收到任何观测 | false | false |
| `NORMAL` | 正常处理，message 带 `engine=<模式>` | true | false |
| `BASELINE_FALLBACK` | 指定模型且 `InferenceSession` 加载失败、`allow_fallback=true`，且当前流水线状态为 `NORMAL` | false | false |
| `NONFINITE_OBSERVATION` | 观测含非有限值（含 float32 转换溢出），已跳过推理 | false | false |
| `INVALID_OBSERVATION` | 观测 `frame_id` 不是 `base_link_flu`，或四元数各分量有限但模长偏离 1 | false | false |
| `NONFINITE_ACTION` | 推理输出含非有限值，已兜底 | false | false |
| `OBS_TIMEOUT` | 观测到达时刻或 `header.stamp` 的年龄超过 `watchdog_timeout_s` | false | true |
| `PIPELINE_EXCEPTION` | 回调内捕获未预期异常 | false | true |

具体故障（`NONFINITE_*`、`INVALID_OBSERVATION`、`OBS_TIMEOUT`、`PIPELINE_EXCEPTION`）优先于降级状态，
`BASELINE_FALLBACK` 只在流水线状态为 `NORMAL` 时出现。

`is_safe` 表示流水线无故障，且引擎不是 `BASELINE_FALLBACK`；它不表示可以上真机。`model_path` 为空时引擎是
`BASELINE_NO_MODEL`，流水线正常仍会报 `NORMAL`、`is_safe=true`，此时跑的是内置基准控制器。

看门狗以 20Hz 发布 `SafetyStatus`，是安全状态的唯一发布点。`offboard_enabled` 恒为 false。
首帧观测到达前报 `WAITING_OBSERVATION`，不下发动作。超时判据取两路年龄的较大者：
观测到达时刻的年龄、`header.stamp` 的年龄；只看回调时刻会漏掉队列积压。时钟回跳时重置到达时刻基线。
超时且此前收到过观测时，看门狗以 20Hz 向 `/haique/policy_action` 下发
`(0, 0, 0, clamp(hover_thrust, min_thrust, max_thrust))`，`raw_action` 与下发动作相同。

### 3.4 ONNX 模型

`model_path` 为空时使用内置基准控制器，不导入 numpy/onnxruntime。指定模型时：

- 代码按模型元数据取输入输出名字，不使用固定名字。
- 输入必须只有 1 个，rank 为 2，最后一维为 17，固定 batch 必须为 1（或动态维），
  dtype 必须为 `tensor(float)`。
- 使用第一个输出；输出 rank 为 1 或 2，最后一维为 4；rank 2 时固定 batch 必须为 1（或动态维）。
- 形状和 dtype 校验发现不了输入顺序错误：同形状的旧模型会被静默接受，按错误特征推理。模型必须按
  3.1 的冻结顺序导出，顺序变更后旧模型必须重新导出。
- 加载后校验不通过、导入 numpy/onnxruntime 失败，或模型加载失败且 `allow_fallback=false`，一律
  `RuntimeError` 退出，不降级。
- 只有 `InferenceSession` 加载失败且 `allow_fallback=true` 时降级为 `BASELINE_FALLBACK`。
- 不校验 opset 和输出 dtype。

内置基准控制器读取观测的 10..12 维（相对位置误差），输出
`[-0.5*rel_y, 0.5*rel_x, 0.0, hover_thrust + 0.1*rel_z]`。它只用于验证数据链路，不是可上真机的控制律。

## 4. 坐标系

| 名称 | 全称 | 使用方 | X | Y | Z |
|---|---|---|---|---|---|
| FLU | Front-Left-Up | ROS REP-103、Isaac Lab、本仓库 | 机头前 | 左 | 上 |
| FRD | Front-Right-Down | PX4、多数飞控固件 | 机头前 | 右 | 下 |

两套机体系相差绕 X 轴 180 度，X 分量不变，Y、Z 分量取反：

| 量 | FLU | FRD |
|---|---|---|
| 横滚角速度 | `wx` | `wx` |
| 俯仰角速度 | `wy` | `-wy` |
| 偏航角速度 | `wz` | `-wz` |
| 推力向量 | `[0, 0, +τ]` | `[0, 0, -τ]` |

推力是标量，换算是把它填进 `thrust_body` 时 z 分量取负。

世界系约定为 ENU（x 东、y 北、z 上，ROS REP-103）。PX4 里程计的姿态是 FRD 到 NED，
不能直接填进观测；需要先把机体系 FRD→FLU、世界系 NED→ENU 一起转换。

四元数的顺序和方向见 3.1。

## 5. 包结构

```text
onboard_ws/src/
├── haique_interfaces/       # 消息定义，ament_cmake + rosidl，msg/ 下三个 .msg
└── haique_policy_runner/    # 三个节点，ament_python
    ├── haique_policy_runner/{mock_telemetry_pub,policy_runner_node,px4_rates_bridge}.py
    ├── launch/mock_closed_loop.launch.py
    └── setup.py, package.xml
```

消息必须单独成包：`rosidl_generate_interfaces` 只能用在 CMake 包里，Python 包无法自己生成消息代码。
`haique_policy_runner` 在 `package.xml` 里声明依赖 `haique_interfaces`，colcon 会按依赖顺序先编译消息包。

## 6. 节点参数与校验

`policy_runner_node`：

| 参数 | 默认值 | 合法范围 | 含义 |
|---|---|---|---|
| `model_path` | `''` | 空或存在的文件路径 | 空表示使用内置基准控制器 |
| `allow_fallback` | `False` | bool | `InferenceSession` 加载失败时是否允许降级 |
| `hover_thrust` | `0.58`（未标定） | `(0, 1]` 且落在 `[min_thrust, max_thrust]` | 悬停归一化推力，兜底与内置控制器使用 |
| `max_tilt_rate` | `3.0` | `(0, 20]` | 三轴角速度限幅，rad/s |
| `min_thrust` | `0.15` | `[0, 1)` | 推力下限 |
| `max_thrust` | `0.85` | `(0, 1]` | 推力上限 |
| `watchdog_timeout_s` | `0.2` | `(0, 5]` | 观测超时阈值，秒 |

`mock_telemetry_pub`：

| 参数 | 默认值 | 合法范围 | 含义 |
|---|---|---|---|
| `frequency_hz` | `50.0` | `[1, 1000]` | 观测发布频率 |
| `hover_thrust` | `0.58` | `(0, 1]` | 初始 `last_action` 的推力分量 |

校验方式按节点区分：

- `policy_runner_node`：范围、非有限值、`min_thrust >= max_thrust`、`hover_thrust` 越界、
  `model_path` 不存在等语义错误在 `__init__` 抛 `ValueError`；参数类型不符由 rclpy 抛
  `ParameterException`（`InvalidParameterTypeException` 是其子类）。`main` 捕获后打印一行错误并退出 1。
  `allow_fallback` 声明为 BOOL，传整数或字符串同样由 rclpy 拒绝。
- `mock_telemetry_pub`：`declare_parameter` 阶段的 `InvalidParameterTypeException` 被转成
  `ValueError`（提示写成 `100.0` 而不是 `100`）；范围和有限性错误同样抛 `ValueError`，`main` 退出 1。
- 两处都不做静默修正。

0.58 没有实测依据，只用于 mock/台架；`policy_runner_node` 在 `hover_thrust` 仍为 0.58 时启动会打 WARN，
mock 只记录 INFO 参数值。代码注释给出当前硬件悬停推力的估计约为 0.13（未实测）；真机前必须按
`tau_hover = m * g / T_max` 标定。若标定值低于默认 `min_thrust=0.15`，启动校验会失败，必须同步下调
`min_thrust`。安全动作里仍保留一次 clamp 作为最后防线。

`mock_closed_loop.launch.py` 用同一个常量 `HOVER_THRUST=0.58` 同时传给两个节点；如果绕开 launch
单独运行节点并给两者不同的 `hover_thrust`，`last_action` 初值与兜底推力会不一致。

## 7. 已知限制

1. 没有飞机动力学模型。观测轨迹与动作无关，`last_action` 回灌只补上动作这一环，闭环不成立。
2. `px4_rates_bridge` 未接 PX4。仓库没有 `px4_msgs`，没有 offboard 心跳，也不发布 setpoint。
3. 没有状态估计包。PX4 里程计是 FRD/NED，需要转成 FLU/ENU 并组装成 17 维观测。
4. 没有安全抢权包。看门狗超时后会以 20Hz 下发悬停安全动作，但动作到不了飞控（bridge 丢弃）；
   RC 抢权、围栏、硬急停都没有实现。
5. `hover_thrust` 未标定；模型文件没有版本管理，`model_path` 只是裸路径。标定方法和与
   `min_thrust`/`max_thrust` 的关系见第 6 节。节点默认仍以 0.58 启动（只打 WARN），并会把它用于
   兜底动作；真机接入前应改为必须显式传入标定值。
6. 看门狗与 50Hz 推理回调共享 rclpy 默认单线程执行器。ONNX 推理一旦阻塞，看门狗定时器、
   安全动作和 `safety_status` 都会停止更新；推理耗时超过 `watchdog_timeout_s` 时还可能造成
   `OBS_TIMEOUT` 抖动。启用 ONNX 前必须把推理与看门狗隔离（独立线程/executor/安全节点），
   并给共享状态加锁。
7. 观测 schema 的 `rel_position_error` 类型、`last_action` 命名和 `schema_version` 尚未调整，
   属于跨仓库 schema 变更；消息里已留 TODO，不能单方面修改。
8. 没有自动化测试，也没有 CI；回归只能手动运行 mock 链路并检查频率与安全状态。
9. 过期观测没有在推理前拦截。回调只记录到达时刻与 `header.stamp`，年龄判据由 20Hz 看门狗执行；
   队列积压时，回调可能在看门狗下一次 tick 之前先处理一帧陈旧观测并发布策略动作。看门狗随后会
   报告 `OBS_TIMEOUT` 并下发安全动作，但两者之间存在一个窗口。真机前应在推理前按 `header.stamp`
   拒绝超过 `watchdog_timeout_s` 的帧，并且不让它刷新有效观测基线。
10. `header.stamp` 只按 `sec*1e9 + nanosec` 直接换算，没有校验 `nanosec` 是否落在 `[0, 1e9)`。
    发布方给出未规范化的时间戳时，换算结果可能落在未来，看门狗会把它当作“另一时钟域”忽略，
    退化为只看到达时刻。真实估计器必须保证时间戳规范化；接入前应补范围校验。
