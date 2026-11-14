# 海雀机载系统设计说明

面向要改这个系统的人。接口部分与训练仓库 `simulation/Haique27_RL/docs/INTERFACE_CONTRACT.md`（软链接到 `/home/wang/Haique27_RL`）一致，两边改动必须同步。

## 1. 系统边界：只做外层 50Hz 控制
一架多旋翼的控制分成两层。内层跑在 PX4 飞控板上，频率远高于外层，负责陀螺仪滤波、
角速度 PID、电机混控、失控保护。外层跑在机载 Linux 上，频率 50Hz，只回答一个问题：
此刻希望飞机以多快的角速度转动、输出多大的总推力。

本仓库只做外层，输出是三个期望角速度加一个归一化推力，不是四个电机的 PWM。
不允许端到端输出 PWM 的原因：

- 内层依赖 1kHz 级陀螺数据和硬实时调度，而机载 Linux 是非实时系统，调度抖动与 Python GC 都无法保证周期。
- 电机混控要考虑电池电压、桨叶差异、电机安装方向，这些信息只有飞控有。
- 策略一旦发散或输出 NaN，直接给 PWM 没有任何中间环节可以拦截；给角速度时，飞控的
  角速度环与姿态保护仍然生效。

代价是：本仓库不能直接指定姿态角，只能给角速度；推力是相对最大推力的归一化比例，
换桨、换电池后需要重新标定。

## 2. 分层架构
```text
[外层] 机载 Linux，本仓库，50Hz，非实时
  mock_telemetry_publisher --/haique/observation (50Hz, 17 维, FLU)--> policy_runner_node
  policy_runner_node       --/haique/policy_action (50Hz, 4 维, FLU)--> px4_rates_bridge
  policy_runner_node       --/haique/safety_status (20Hz)--> 当前无订阅者
  px4_rates_bridge         --FLU 转 FRD--> 换算结果丢弃，未接 PX4
        │ 目标链路，未实现，需要 px4_msgs 与 Micro-XRCE-DDS
        ▼
[内层] PX4 飞控（目标 v1.14 以上），数百 Hz 以上，硬实时
  输入：三轴期望角速度 (FRD) + 归一化推力 (thrust_body.z = -τ)
  输出：四路电机 PWM；反馈：/fmu/out/vehicle_odometry (FRD, 未接入)
```

## 3. 数据流
| 话题 | 消息类型 | 频率 | 发布者 | 订阅者 | 状态 |
|---|---|---|---|---|---|
| `/haique/observation` | `DroneObservation` | 50Hz | `mock_telemetry_publisher`，将来是状态估计节点 | `policy_runner_node` | 已实现，内容是三角函数模拟 |
| `/haique/policy_action` | `PolicyAction` | 正常 50Hz；观测超时期间 20Hz 安全动作 | `policy_runner_node` | `px4_rates_bridge`、`mock_telemetry_publisher` | 已实现 |
| `/haique/safety_status` | `SafetyStatus` | 20Hz | `policy_runner_node` | 无 | 已实现，当前无人订阅；`offboard_enabled` 恒为 false |
| `/fmu/out/vehicle_odometry` | `px4_msgs` 消息 | 未实现 | 无 | 无 | 未实现，仓库内没有 `px4_msgs` |
| `/fmu/in/vehicle_rates_setpoint` | `px4_msgs` 消息 | 未实现 | 无 | 无 | 未实现，bridge 只换算不发送 |

`mock_telemetry_publisher` 订阅动作只为回填 `last_action`；它不按动作更新姿态或速度，
链路仍是开环。

## 4. 17 维观测与 4 维动作契约
观测顺序固定，不得改动。全部物理量在 FLU 机体系下表达。

| 索引 | 内容 | 单位 | 说明 |
|---|---|---|---|
| 0..3 | 姿态四元数 `[w, x, y, z]` | 无量纲，模长 1 | body 到 world，Hamilton 约定，实部在前 |
| 4..6 | 机体系线速度 `[vx, vy, vz]` | m/s | FLU |
| 7..9 | 机体系角速度 `[wx, wy, wz]` | rad/s | FLU |
| 10..12 | 目标相对位置误差 `[dx, dy, dz]` | m | FLU，目标点减当前点 |
| 13..16 | 上一步动作 `[rx, ry, rz, thrust]` | rad/s, rad/s, rad/s, 归一化 | 实际下发的值，已限幅 |

拼装规则：

- 只允许在 `policy_runner_node` 的一个私有方法里拼装，返回长度为 17 的 tuple。
  别处不得重复拼装，消息里也不再保留 17 维扁平数组字段。
- `geometry_msgs/Quaternion` 的字段定义顺序是 `x, y, z, w`，禁止按下标取，
  必须显式写 `[q.w, q.x, q.y, q.z]`。
- 送进网络前转成 `float32` 并断言形状为 `(1, 17)`；任一元素非有限时不得推理。

动作字段：

| 字段 | 含义 | 单位与范围 |
|---|---|---|
| `desired_body_rates` | 期望三轴角速度，FLU | rad/s，单轴限幅 ±`max_tilt_rate` |
| `collective_thrust` | 归一化总推力 | 限幅到 `[min_thrust, max_thrust]` |
| `raw_action` | 正常路径与内置控制器模式为限幅前原始输出；兜底路径等于实际下发的安全动作 | 仅用于诊断，兜底时不会是 NaN |
| `inference_latency_ms` | 回调内部推理耗时 | ms，用 `perf_counter` 测 |
| `end_to_end_latency_ms` | 观测时间戳到 `publish()` 调用前 | ms，`< 0` 表示不可测（时间戳为 0、超出 1000ms 上界等） |

ONNX 约定：输入名 `obs`，形状 `[batch, 17]`，dtype 必须是 float32；输出名 `action`，
形状 `[batch, 4]`；opset 16。加载时校验输入输出维度与 dtype，不符一律报错退出，
不受 `allow_fallback` 影响。

`SafetyStatus` 的状态码由策略节点按流水线健康度填写，`engine` 取值 `ONNX`、`BASELINE_NO_MODEL`、`BASELINE_FALLBACK`：

| `status_code` | 触发条件 | `is_safe` | `emergency_triggered` |
|---|---|---|---|
| `WAITING_OBSERVATION` | 启动后尚未收到任何观测 | false | false |
| `NORMAL` | 正常处理，message 带 `engine=<模式>` | true | false |
| `BASELINE_FALLBACK` | 指定模型但加载失败、`allow_fallback=true`，且当前流水线状态为 `NORMAL` | false | false |
| `NONFINITE_OBSERVATION` | 观测含非有限值，已跳过推理 | false | false |
| `NONFINITE_ACTION` | 推理输出含非有限值，已兜底 | false | false |
| `OBS_TIMEOUT` | 超过 `watchdog_timeout_s` 无有效观测 | false | true |
| `PIPELINE_EXCEPTION` | 回调内捕获未预期异常 | false | true |

具体故障（`NONFINITE_*`、`PIPELINE_EXCEPTION`）优先于降级状态，`BASELINE_FALLBACK` 只在 `NORMAL` 时出现。

## 5. FLU 与 FRD
| 名称 | 全称 | 使用方 | X | Y | Z |
|---|---|---|---|---|---|
| FLU | Front-Left-Up | ROS REP-103、Isaac Lab、本仓库 | 机头前 | 左 | 上 |
| FRD | Front-Right-Down | PX4、多数飞控固件 | 机头前 | 右 | 下 |

两套坐标系同时存在，因为 PX4 的混控矩阵和传感器安装按 FRD 写死，ROS 规范按 FLU 写死，
改任何一边的代价都远大于中间做一次换算。两者相差绕 X 轴 180 度，所以 X 分量不变，
Y、Z 分量取反。

| 量 | FLU | FRD |
|---|---|---|
| 横滚角速度 | `wx` | `wx` |
| 俯仰角速度 | `wy` | `-wy` |
| 偏航角速度 | `wz` | `-wz` |
| 推力向量 | `[0, 0, +τ]` | `[0, 0, -τ]` |

记法：横滚绕前后轴，两套坐标系的 X 轴同向，符号不变；俯仰绕左右轴，Y 轴反向，取反；
偏航绕上下轴，Z 轴反向，取反。推力是标量，换算是把它填进 `thrust_body` 时 z 分量取负。

## 6. 包结构与职责
```text
onboard_ws/src/
├── haique_interfaces/       # 消息定义，ament_cmake + rosidl，msg/ 下三个 .msg
└── haique_policy_runner/    # 三个节点，ament_python
    ├── haique_policy_runner/{mock_telemetry_pub,policy_runner_node,px4_rates_bridge}.py
    ├── launch/mock_closed_loop.launch.py
    └── setup.py, package.xml
```

消息必须单独成包，理由有三条：

- `rosidl_generate_interfaces` 只能用在 CMake 包里，Python 包无法自己生成消息代码。
- 消息包被两个以上包依赖时，colcon 会自动推导编译顺序，不需要手工指定。
- 以后若用 C++ 重写推理节点（目的是消除 Python GC 停顿），它要与 Python 节点共用同一份定义；
  定义只有一处，不会出现两边字段不一致。

## 7. 节点参数
`policy_runner_node`：

| 参数 | 默认值 | 合法范围 | 含义 |
|---|---|---|---|
| `model_path` | `''` | 空或存在的文件路径 | 空表示使用内置基准控制器 |
| `allow_fallback` | `False` | bool | 指定了模型但加载失败时是否允许降级 |
| `hover_thrust` | `0.58` | `(0, 1]` | 悬停归一化推力，兜底与降级时使用 |
| `max_tilt_rate` | `3.0` | `(0, 20]` | 三轴角速度限幅，rad/s |
| `min_thrust` | `0.15` | `[0, 1)` | 推力下限 |
| `max_thrust` | `0.85` | `(0, 1]` | 推力上限 |
| `watchdog_timeout_s` | `0.2` | `(0, 5]` | 观测超时阈值，秒 |

`mock_telemetry_pub`：

| 参数 | 默认值 | 合法范围 | 含义 |
|---|---|---|---|
| `frequency_hz` | `50.0` | `[1, 1000]` | 观测发布频率 |
| `hover_thrust` | `0.58` | `(0, 1]` | 初始 `last_action` 的推力分量 |

参数在 `__init__` 里校验，越界、非有限值、`min_thrust >= max_thrust` 都会直接抛 `ValueError`，不做静默修正。

`hover_thrust` 的 0.58 没有实测依据，只是沿用旧值。真机前必须按
`tau_hover = m * g / T_max` 重新标定，当前硬件的估计值约为 0.13。给 0.58 会得到
接近悬停所需四倍的推力。安全动作里的推力会再夹进 `[min_thrust, max_thrust]`：
0.13 低于默认 `min_thrust=0.15`，实际下发 0.15，标定悬停推力时要同步下调 `min_thrust`。

## 8. 已知限制与后续计划
1. 没有飞机动力学模型。观测轨迹与动作无关，`last_action` 回灌只补上动作这一环，闭环不成立；下一步写刚体积分节点替换 mock 发布者。
2. `px4_rates_bridge` 未接 PX4。缺 `px4_msgs` 依赖、缺 offboard 心跳、缺 setpoint 发布。
3. 没有状态估计包。PX4 里程计是 FRD，需要换算成 FLU 并组装成 17 维观测。
4. 没有安全抢权包。看门狗超时后会以 20Hz 下发悬停安全动作，但动作到不了飞控
   （bridge 丢弃）；`safety_status` 仍无人订阅，RC 抢权、围栏、硬急停都不存在。
5. `hover_thrust` 未标定，模型文件也没有版本管理，`model_path` 只是一个裸路径。
