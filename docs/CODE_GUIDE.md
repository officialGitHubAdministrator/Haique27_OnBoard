# 海雀代码导读

面向会一点 Python、ROS 2 基础薄弱、刚接手仓库的人。建议按 1 到 6 节顺序读，第 7 节术语表备查。
接口契约以 [DESIGN.md](DESIGN.md) 为准，这里只解释代码怎么实现它；代码行为与文档冲突时以代码为准，并同步修正文档。

## 1. 先建立物理图景

### 1.1 两层控制

本仓库是外层：50Hz，17 维观测进，4 维动作出，不输出电机 PWM。内层在 PX4 上，把期望角速度变成电机控制信号。

```
外层 50Hz（本仓库）                    内层数百 Hz（PX4）
17 维观测 ──▶ 策略 ──▶ 4 维动作 ──▶ 角速度环 ──▶ 电机
```

为什么这样分层、为什么只给角速度，见 DESIGN 的“系统边界”一节；读代码时只需记住：策略负责 17 进 4 出，
异常值在发布前被限幅或替换成悬停动作。

### 1.2 这一版实际能跑什么

当前仓库能跑通的只有一条 mock 开环流水线：

- `mock_telemetry_pub.py` 以 50Hz 用三角函数生成观测，不包含飞机动力学。
- `policy_runner_node.py` 拼装 17 维观测，用 ONNX 或内置基准控制器推理，限幅后发布 4 维动作。ONNX 分支代码存在，但默认镜像没有 `onnxruntime`，仓库也没有模型文件，这条路径尚未验证。
- `px4_rates_bridge.py` 只做 FLU 到 FRD 换算，换算结果丢弃，不向 PX4 发送任何东西。

mock 的观测轨迹与动作无关，所以链路是开环的；launch 文件名仍叫 `mock_closed_loop`，名字是历史遗留。**这套代码不能用于真机飞行**，未实现项见 DESIGN 的“已知限制”一节。

### 1.3 17 个什么进、4 个什么出

| 输入（观测，17 维） | 维度 | 输出（动作，4 维） | 维度 |
|---|---|---|---|
| 姿态四元数 `[w,x,y,z]` | 4 | 期望横滚角速度 | 1 |
| 机体系线速度 | 3 | 期望俯仰角速度 | 1 |
| 机体系角速度 | 3 | 期望偏航角速度 | 1 |
| 目标相对位置误差 | 3 | 归一化总推力 | 1 |
| 上一步动作 | 4 | | |

看代码时先问：它在生产这 17 个数、消费这 17 个数、生产这 4 个数，还是消费这 4 个数。

### 1.4 两个仓库

训练仓库单独维护，不在本仓库内，负责训练和导出 ONNX。两边没有代码级依赖，连接点是一份接口契约，以训练仓库的 `docs/INTERFACE_CONTRACT.md` 为准。本仓库不含训练代码、训练脚本或 `.onnx` 模型文件。

## 2. ROS 2 最小必备概念

### 2.1 节点 Node

节点是继承 `rclpy.node.Node` 的类，`super().__init__('名字')` 里的名字会出现在 `ros2 node list`。本项目三个类：`MockTelemetryPublisher`、`PolicyRunnerNode`、`PX4RatesBridge`。

### 2.2 话题 Topic 与消息 Message

话题是带名字的单向管道，消息是管道里的类型。`/haique/observation` 对应 `DroneObservation`，`/haique/policy_action` 对应 `PolicyAction`，`/haique/safety_status` 对应 `SafetyStatus`。`.msg` 文件在 `haique_interfaces/msg/` 下，编译时由 `CMakeLists.txt` 的 `rosidl_generate_interfaces` 生成 Python 类。

### 2.3 发布者、订阅者、回调

发布者往话题写，订阅者从话题读。订阅时注册的回调在数据到达时由框架自动调用。主体只有一句 `rclpy.spin(node)`，不要自己写 `while True`。

```python
# mock_telemetry_pub.py
self.obs_pub = self.create_publisher(DroneObservation, '/haique/observation', 10)

# policy_runner_node.py，第三个参数是回调
self.obs_sub = self.create_subscription(
    DroneObservation, '/haique/observation', self.observation_callback, 10)
```

回调里不要睡长觉、不要做重活。默认是单线程执行器，阻塞回调会连带卡住定时器。

### 2.4 定时器 Timer

`create_timer(周期秒, 回调)` 是 50Hz 的来源，见 `mock_telemetry_pub.py` 的 `self.create_timer(1.0 / self.freq_hz, self.timer_callback)`。定时器不保证准点，回调超时会丢拍，用 `ros2 topic hz` 看真实频率。

### 2.5 参数 Parameter

先用 `declare_parameter` 声明并给默认值，再用 `get_parameter` 读，见 `policy_runner_node.py` 的 `__init__`。覆盖方式有两种：launch 里的 `parameters=[...]`，或命令行 `--ros-args -p max_thrust:=0.7`。

非法参数会让节点打印一行错误后退出，不会带着它继续跑，校验规则见 DESIGN 的“节点参数与校验”。参数名写错不会报错，只会使用默认值。

### 2.6 Launch 文件

`launch/mock_closed_loop.launch.py` 用三个 `Node(...)` 描述节点并返回 `LaunchDescription([...])`。`executable` 是 `setup.py` 里 `entry_points` 注册的命令名，写错会报 executable not found。

### 2.7 工作空间、包、colcon、source

`onboard_ws` 下 `src/` 放代码，`build/`、`install/`、`log/` 是 colcon 生成物，不要手改。包是 `src/` 下带 `package.xml` 的目录。编译完必须 `source install/setup.bash`，否则 `ros2 run` 找不到包；顺序是先 source `/opt/ros/humble/setup.bash`。

`build_workspace.sh` 用的是裸 `colcon build`，`install/` 里是文件副本，不是符号链接。改 `src/` 下的 `.py` 后必须重新执行 `./docker/build_workspace.sh`，只重启节点不生效。

### 2.8 QoS 深度

`create_publisher(..., 10)` 的 `10` 是队列深度，满了丢最旧的消息。50Hz 下深度 10 最坏会引入 200ms 排队延迟，压延迟时可以降到 1 到 2。

## 3. 逐文件精读

### 3.1 haique_interfaces：接口定义

`DroneObservation.msg` 只有结构化字段，没有 17 维扁平数组。同一份数据存两遍，早晚会有一遍写错，所以 17 维只在 `policy_runner_node._build_observation` 里拼。字段顺序固定为四元数、机体系线速度、机体系角速度、目标相对位置误差、上一步动作。

`PolicyAction.msg` 的 `raw_action` 在正常 ONNX 与内置控制器路径下是限幅前的原始输出；在兜底路径（非法观测、非有限观测或动作、超时、管线异常）下等于实际下发的安全动作，不含 NaN。`desired_body_rates` 与 `collective_thrust` 是限幅后真正下发的值。两个延迟字段的含义见第 3.3 节。

`SafetyStatus.msg` 的 `offboard_enabled` 语义上归桥节点所有，但当前话题的唯一发布者是 `policy_runner_node`，所以恒为 false。`emergency_triggered` 只在 `OBS_TIMEOUT` 和 `PIPELINE_EXCEPTION` 时为 true。

`CMakeLists.txt` 的 `rosidl_generate_interfaces` 只生成清单中列出的 `.msg`。没注册的文件不报错，但永远缺失；确认是否生成，去 `onboard_ws/install/haique_interfaces/local/lib/python3.*/dist-packages/haique_interfaces/msg/` 下找 `_drone_observation.py` 一类的文件。

### 3.2 mock_telemetry_pub.py：假传感器

职责：没有飞机、没有仿真器时以 `frequency_hz` 产生观测，让管道能跑。它是全项目唯一发布观测数据的节点，但观测是三角函数编的。

流程：

1. 声明 `frequency_hz`、`hover_thrust` 并校验范围，非法值直接退出。
2. 建观测发布者、动作订阅者、定时器。
3. `action_callback` 收到 `/haique/policy_action` 后整表替换 `last_action`；含非有限值时丢弃该帧并保留上一帧。
4. `timer_callback` 先递增 `tick_count`，用 `t = tick_count / frequency_hz` 计算相位，生成相对位置误差、姿态、线速度和角速度，按 ZYX（`R = Rz·Ry·Rx`）转成四元数并按字段名填 `w/x/y/z`。
5. `header.stamp` 取发布时刻。mock 没有真实传感器，可以这样；真实估计器必须填采样时刻。
6. 填 `frame_id='base_link_flu'`，发布观测。

坑：

- 它和策略节点互为对方的发布者与订阅者，形成一条环。环里跑的是动作，姿态和速度不看动作，所以仍是开环。
- 改发布频率会改变策略收到的观测频率，看门狗阈值要一起考虑。
- `last_action` 初值是 `[0, 0, 0, hover_thrust]`，不是真实动作。策略停摆超过 `3.0 / frequency_hz` 秒时只告警，不把动作归零。
- 相位用 `tick_count` 推进，不用 ROS 时钟，避免仿真重置或回放导致时钟跳变时正弦相位跳变。

### 3.3 policy_runner_node.py：策略节点

职责：消费观测，产出动作和安全状态。这是最需要读懂、也最可能被改的文件。

```
__init__               声明并校验 7 个参数 → 初始化推理引擎 → 建 1 订阅 2 发布 + 20Hz 看门狗
observation_callback   校验 frame_id/四元数 → 拼 17 维 → 查非有限 → 推理 → 查非有限 → 限幅 → 发布
watchdog_check         每 50ms 发布 safety_status；超时且收到过观测时以 20Hz 下发悬停动作
```

处理流程：

- `_build_observation` 校验 `frame_id == base_link_flu`、四元数模长为 1（容差 `1e-2`），按字段名拼出长度 17 的 tuple，并显式检查长度。四元数写成 `(q.w, q.x, q.y, q.z)`。
- `frame_id` 不符或模长超差抛 `InvalidObservationError`，发安全动作并置 `INVALID_OBSERVATION`。
- 任一元素非有限就跳过推理，发 `(0, 0, 0, clamp(hover_thrust, min_thrust, max_thrust))`，状态码 `NONFINITE_OBSERVATION`。ONNX 分支里 `1e300` 转 `float32` 溢出成 `inf` 也走这条路径。
- 有 ONNX 会话就推理，否则用内置比例控制器。内置控制器读第 10 到 12 个元素（相对位置误差），不是第 0 到 2 个。
- 推理结果再查非有限，异常发安全动作，状态码 `NONFINITE_ACTION`。
- 角速度三轴各限幅到 `±max_tilt_rate`，推力限幅到 `[min_thrust, max_thrust]`，然后发布。
- 回调内任何未预期异常都会置 `PIPELINE_EXCEPTION` 并尝试发安全动作。安全状态本身只由 20Hz 的 `watchdog_check` 对外发布。

8 个状态码、触发条件和优先级见 `docs/DESIGN.md` 的“安全状态”一节。

延迟字段在代码里的计时点：`inference_latency_ms` 从 `observation_callback` 第一行的 `perf_counter()` 开始，到 `_publish_action` 开头结束，所以它包含校验、拼装、推理和限幅，不只是网络前向；`end_to_end_latency_ms` 用观测的 `header.stamp` 和 `publish()` 前的 ROS 时钟相减。两者何时填 -1 见 DESIGN 的“4 维动作”。节点每 100 步打印一次两者的均值。

坑（代码特有的部分）：

- 限幅前必须先判有限。`_clamp` 的实现是 `max(lo, min(hi, value))`；在 CPython 中 `min(hi, nan)` 返回 `hi`，当前调用的角速度与推力上限都是正数，所以 NaN 会被洗成上限，放行成满量程角速度和最大推力。不能把有限性检查交给 `_clamp`。
- `max_tilt_rate` 名字看起来像倾角，实际限制的是三轴角速度。
- `allow_fallback` 不能用 `bool(value)` 转换，`bool('false')` 是真。launch 用 `ParameterValue(..., value_type=bool)` 传入真正的布尔值；节点把它声明成 BOOL，命令行传 `1` 或带引号的 `'true'` 会被 rclpy 拒绝。`_parse_bool` 只是防御性检查。
- 超时后只有此前收到过至少一帧观测才下发安全动作，避免上电阶段就向飞控灌 setpoint。动作以 20Hz 持续发布，但仍到不了 PX4，`safety_status` 也没有常驻订阅者。
- 看门狗和 50Hz 回调共用默认单线程执行器，ONNX 推理一旦阻塞，看门狗也跟着停。详见 DESIGN 的“已知限制”一节。
- 过期观测只由看门狗事后发现，推理前不拦截；`header.stamp` 也只换算、不校验 `nanosec` 范围。这两点同样记在 DESIGN 的“已知限制”里，真机接入前要补。

其余契约细节不在这里重复：参数校验和 `hover_thrust` 标定见 DESIGN 的“节点参数与校验”，
ONNX 启动失败分类见 DESIGN 的“ONNX 模型”，看门狗两路年龄判据见 DESIGN 的“安全状态”。

### 3.4 px4_rates_bridge.py：坐标桥

职责：订阅动作，把 FLU 换算成 FRD，然后不发布任何东西。

```text
roll_frd  =  roll_flu
pitch_frd = -pitch_flu
yaw_frd   = -yaw_flu
thrust_body.z = -thrust
```

收到首帧有效动作时打一条 WARN，说明转换结果被丢弃；非有限输入整帧丢弃并限流告警；换算结果只写限流 DEBUG 日志。

坑：文件名看起来像链路已通，实际在这里断掉。仓库没有 `px4_msgs`，接通要先引入它，再同时发 `OffboardControlMode` 心跳和 `VehicleRatesSetpoint`。换算符号不要凭感觉改，它直接决定 PX4 收到的俯仰、偏航和推力方向。

### 3.5 mock_closed_loop.launch.py：启动清单

三个 `Node`：mock 观测、策略节点、坐标桥。两个节点用文件顶部的常量 `HOVER_THRUST` 保证悬停推力一致；策略节点还传限幅、看门狗，以及两个 launch 参数 `model_path`（默认空）和 `allow_fallback`（默认 false）。`executable` 必须和 `entry_points` 完全一致，包括下划线。

启用 ONNX 模型时不必改 launch 文件，直接覆盖 `model_path`，命令见 README 的“构建与运行”。

`policy_runner_node` 退出时，`OnProcessExit` 会触发 `Shutdown` 关闭整栈，不会出现“控制器已退出、mock 还在打印”的假正常。桥退出只打 `LogInfo`，severity 是 INFO，文本带 `[WARN]`。

### 3.6 两个包的构建文件

`haique_policy_runner/setup.py` 的 `data_files` 把 `package.xml`、`resource/` 标记、`launch/*.launch.py` 装到 `share/` 下，`entry_points` 注册三个命令指向各自的 `main`。新 launch 文件必须以 `.launch.py` 结尾，否则 `glob` 匹配不到。

`haique_policy_runner/package.xml` 声明 `rclpy`、`haique_interfaces`、`geometry_msgs`、`std_msgs`，以及 `launch`、`launch_ros` 两个 `exec_depend`；`build_type` 是 `ament_python`，colcon 按依赖推导编译顺序。`package.xml` 还列了 `ament_copyright`、`ament_flake8`、`ament_pep257`、`python3-pytest` 四个测试依赖，但仓库没有测试文件，也没有 CI。

`haique_interfaces/package.xml` 的 `build_type` 是 `ament_cmake`，并通过 `member_of_group` 声明属于 `rosidl_interface_packages` 组。

### 3.7 docker 目录

`build_workspace.sh`、`run_mock_loop.sh`、`run_desktop_container.sh` 默认都用 `osrf/ros:humble-desktop`，可以用 `HAIQUE_IMAGE` 覆盖。

- `build_workspace.sh` 把 `onboard_ws` 挂到 `/root/ros2_ws`，在容器里执行 `colcon build`，结束后把 `build/install/log` 的属主改回宿主机用户。
- `run_mock_loop.sh` 挂载工作区，带 `--net=host`，source 后直接 `ros2 launch`，但使用独立 IPC 命名空间；其他容器可能只能发现话题而收不到数据。需要检查话题时，在 `run_desktop_container.sh` 的开发容器内启动 launch。
- `run_desktop_container.sh` 建名为 `haique_ros2_humble` 的长期容器，带 `--net=host`、`--ipc=host`、X11 转发、`/dev` 挂载和 `--privileged`，所以能开 rviz2；检测到 `nvidia-smi` 与 `nvidia-container-toolkit` 命令时会加 `--gpus all`，有 `/dev/dri` 时会映射 DRI 设备。容器基本不受限，个人机器可以接受，共用机器要谨慎。
- `build_image.sh` 构建 `haique/ros2-humble:latest`。`Dockerfile` 在官方镜像上安装 `numpy`、`onnx`、`onnxruntime`、`transforms3d`、`scipy`，并编译 Micro-XRCE-DDS-Agent。默认镜像没有 `onnxruntime`，所以跑 ONNX 要用这个镜像。

## 4. 一帧数据的时间线

三个节点是三个进程，下面的顺序表示因果，不表示严格同刻；具体耗时取决于机器负载。

```text
t = 0       mock_telemetry_pub.timer_callback 被 50Hz 定时器触发
            - 取 last_action，即上一帧收到的动作
            - 用 sin/cos 生成相对位置误差、姿态、线速度、角速度
            - header.stamp = self.get_clock().now()
            - publish /haique/observation

t ≈ 0       policy_runner_node.observation_callback 被唤起
            - 记录到达时刻与 header.stamp
            - 校验 frame_id 与四元数模长：不合格发安全动作，INVALID_OBSERVATION，返回
            - 拼 17 维 tuple，检查长度
            - 非有限检查：不合格发安全动作，NONFINITE_OBSERVATION，返回
            - 推理：ONNX 会话或内置基准控制器
            - 非有限检查：不合格发安全动作，NONFINITE_ACTION
            - 限幅：角速度 ±max_tilt_rate，推力 [min_thrust, max_thrust]
            - 置 NORMAL，算 inference_latency_ms 与 end_to_end_latency_ms
            - publish /haique/policy_action

t ≈ 0       px4_rates_bridge.action_callback 被唤起
            - 非有限检查，FLU 转 FRD，结果丢弃，收到首帧有效动作时打 WARN

t ≈ 20ms    mock 的下一帧：last_action 通常已被上一帧动作替换

每 50ms     policy_runner_node.watchdog_check
            - 尚未收到观测：报 WAITING_OBSERVATION，不下发动作
            - 年龄超过 watchdog_timeout_s：报 OBS_TIMEOUT；收到过观测时同时以 20Hz 发布安全动作
            - 未超时：发布当前流水线状态（NORMAL / BASELINE_FALLBACK 等）
```

评估链路延迟时看 `end_to_end_latency_ms`，它包含观测从发布到被回调处理的传输和排队；`inference_latency_ms` 只覆盖回调内部。

## 5. 坐标系与四元数

FLU（前-左-上）是 ROS 与 Isaac Lab 的标准，FRD（前-右-下）是 PX4 的标准。完整对照表、世界系 ENU/NED 的约定见 `docs/DESIGN.md` 的“坐标系”一节。代码里真正要小心的是下面三点。

1. **四元数顺序。** 本项目统一 `[w, x, y, z]`（Hamilton，实部在前），而 `geometry_msgs/Quaternion` 的字段定义顺序是 `x, y, z, w`，`ros2 topic echo` 打印出来也是 `x, y, z, w`。拼 17 维时必须显式写 `(q.w, q.x, q.y, q.z)`，不能按消息下标取。
2. **四元数方向。** 观测里的四元数表示 body(FLU) 到 world(ENU) 的旋转。PX4 的 `q` 是 FRD 到 NED，不能直接复用，必须同时转换机体系和世界系。写成 world 到 body，观测里的姿态就是反的。
3. **模长校验。** 有限四元数的模长必须为 1（容差 `1e-2`），超出报 `INVALID_OBSERVATION`。四元数含非有限值时先不做模长判断，随后由统一的有限性检查报 `NONFINITE_OBSERVATION`。

mock 用 ZYX（yaw-pitch-roll）的 `R = Rz·Ry·Rx` 公式生成四元数；改欧拉顺序会直接改变姿态含义。桥的符号换算只有四个式子，见第 3.4 节，不要凭感觉改。

## 6. 动手实验

实验都在容器里做。先进入开发容器并 source 环境：

```bash
./docker/run_desktop_container.sh          # 在仓库根目录执行
# 容器内
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
```

下面每个“终端”指宿主机另开一个终端并执行 `./docker/run_desktop_container.sh`；容器已在运行时脚本会
`docker exec` 进同一个容器。进入后都要先 source 上面两行环境。
容器里的 `/root/ros2_ws` 就是宿主机的 `onboard_ws`。改完 `src/` 下的 `.py` 后，回到宿主机仓库根目录执行 `./docker/build_workspace.sh`，再重启节点。每个实验开始前先停掉上一实验的节点，避免同名节点和话题混在一起。

### 实验 1：看数据在流

```bash
ros2 launch haique_policy_runner mock_closed_loop.launch.py      # 终端 A
ros2 node list                                                   # 终端 B
ros2 topic hz /haique/observation                                # 终端 B，Ctrl+C 停止
ros2 topic echo /haique/observation --once
ros2 topic echo /haique/observation --field last_action   # Ctrl+C 停止
ros2 topic echo /haique/policy_action --once
ros2 topic info /haique/safety_status -v
```

验证什么：三个节点都在，观测稳定在 50Hz，`/haique/safety_status` 的订阅者数量是 0；`--field last_action` 会显示它跟随上一帧动作变化，而不是常量。

### 实验 2：参数限幅与 fail-fast

```bash
ros2 run haique_policy_runner mock_telemetry_pub                                # 终端 A
ros2 run haique_policy_runner policy_runner_node \
  --ros-args -p hover_thrust:=0.30 -p max_thrust:=0.30                          # 终端 B
ros2 topic echo /haique/policy_action                                            # 终端 C
```

验证什么：`collective_thrust` 不超过 0.30；`raw_action` 里的推力在 0.29 到 0.31 之间（内置控制器输出 `0.30 + 0.1 * rel_z`），说明限幅作用在原始输出之后。

再试非法参数：

```bash
ros2 run haique_policy_runner policy_runner_node --ros-args -p min_thrust:=0.90
ros2 run haique_policy_runner policy_runner_node --ros-args -p hover_thrust:=0.90
ros2 run haique_policy_runner policy_runner_node --ros-args -p watchdog_timeout_s:=1
```

验证什么：第一条因 `min_thrust >= max_thrust` 抛 `ValueError`；第二条因 `hover_thrust` 不在 `[min_thrust, max_thrust]` 内抛 `ValueError`；第三条是整数覆盖 DOUBLE 参数，由 rclpy 抛 `ParameterException`，`main` 捕获后打印一行可读错误。三条都不会带着非法参数继续跑，也不打 traceback。

### 实验 3：触发看门狗

先确认观测真的在流，否则状态会停在 `WAITING_OBSERVATION`，看不到超时。

```bash
ros2 launch haique_policy_runner mock_closed_loop.launch.py   # 终端 A
ros2 topic hz /haique/observation                            # 终端 B
ros2 topic echo /haique/safety_status                        # 终端 C
pkill -f 'lib/haique_policy_runner/mock_telemetry_pub'       # 终端 D
ros2 topic hz /haique/policy_action                          # 终端 D
```

验证什么：约 `watchdog_timeout_s` 后状态码变成 `OBS_TIMEOUT`，`emergency_triggered` 为 true。`/haique/policy_action` 转为 20Hz 的安全动作（零角速度加悬停推力），但 bridge 会丢弃，飞控收不到；`safety_status` 没有常驻订阅者，终端 C 的 echo 只是临时订阅。mock 退出不会触发整栈关闭，因为 launch 只为策略节点的退出注册了 `Shutdown`。

### 实验 4：写一个最小订阅节点

在容器内新建 `/tmp/watch.py`（宿主机的 `/tmp` 没有挂载进容器），不要放进 `src/`，避免污染仓库。

```python
import rclpy
from rclpy.node import Node
from haique_interfaces.msg import PolicyAction


class Watcher(Node):
    def __init__(self):
        super().__init__('my_watcher')
        self.create_subscription(PolicyAction, '/haique/policy_action', self.cb, 10)

    def cb(self, msg):
        r = msg.desired_body_rates
        self.get_logger().info(
            f"rates=({r.x:+.3f}, {r.y:+.3f}, {r.z:+.3f}) "
            f"thrust={msg.collective_thrust:.3f} "
            f"infer={msg.inference_latency_ms:.2f}ms "
            f"e2e={msg.end_to_end_latency_ms:.2f}ms")


def main():
    rclpy.init()
    node = Watcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

```bash
python3 /tmp/watch.py
```

验证什么：最小节点就是继承 `Node`、注册回调、`spin`。对比 `infer` 与 `e2e` 的差值，主要反映观测从发布到回调的传输/排队，以及动作消息构造的开销。

## 7. 术语表

| 词 | 解释 |
|---|---|
| FLU / FRD | 机体系：前-左-上 / 前-右-下；完整对照见 DESIGN 的“坐标系”一节 |
| ENU / NED | 世界系：东-北-上 / 北-东-下 |
| Offboard | PX4 模式：由外部计算机发送控制指令 |
| setpoint | 设定值，希望达到的状态或指令 |
| Micro-XRCE-DDS | PX4 与机载计算机之间的 DDS 通信中间件，`Dockerfile` 里安装了 Agent |
