# 海雀项目代码导读

读者假设：会一点 Python，ROS 2 基础薄弱，刚接手这个仓库。建议按 1 到 8 节顺序读。
接口契约以 `docs/DESIGN.md` 为准，本文只解释代码怎么实现它。

## 1. 先建立物理图景

### 1.1 为什么分内外两层

内层在 PX4 飞控板上，频率远高于外层，把"期望角速度"变成四个电机的油门，负责陀螺滤波、
PID、混控和失控保护。外层在机载 Linux 上，50Hz，决定接下来该怎么转、出多大力，
本项目只做外层：17 个数进，4 个数出。

```
外层 50Hz（本仓库）            内层数百 Hz（PX4）
17 维观测 ──▶ 策略 ──▶ 4 维动作 ──▶ 角速度环 ──▶ 电机
```

不让策略直接输出电机 PWM，原因有两个：机载 Linux 不是实时系统；混控参数只有飞控知道。
输出异常时，角速度接口还能被飞控的姿态保护兜住，PWM 接口没有任何兜底。

### 1.2 17 个什么进、4 个什么出

| 输入（观测，17 维） | 维度 | 输出（动作，4 维） | 维度 |
|---|---|---|---|
| 姿态四元数 `[w,x,y,z]` | 4 | 期望横滚角速度 | 1 |
| 机体系线速度 | 3 | 期望俯仰角速度 | 1 |
| 机体系角速度 | 3 | 期望偏航角速度 | 1 |
| 目标相对位置误差 | 3 | 归一化总推力 | 1 |
| 上一步动作 | 4 | | |

看任何一段代码，先问：它在生产这 17 个数、消费这 17 个数、生产这 4 个数，还是消费这 4 个数。

### 1.3 两个仓库

机载工作区是当前仓库，放 ROS 2 节点；训练仓库在 `/home/wang/Haique27_RL`，负责
Isaac Lab 训练和导出 ONNX。工作区里的 `simulation/Haique27_RL` 是它的软链接。
两边没有代码级依赖，连接点只有一份接口契约。训练仓库的环境和脚本目前基本是空的。

## 2. ROS 2 最小必备概念

### 2.1 节点 Node
节点是继承 `rclpy.node.Node` 的类，`super().__init__('名字')` 里的名字会出现在
`ros2 node list`。本项目三个类：`MockTelemetryPublisher`、`PolicyRunnerNode`、`PX4RatesBridge`。

### 2.2 话题 Topic 与消息 Message
话题是带名字的单向管道，消息是管道里的类型。`/haique/observation` 对应
`DroneObservation`，`/haique/policy_action` 对应 `PolicyAction`，`/haique/safety_status`
对应 `SafetyStatus`。`.msg` 文件在 `haique_interfaces/msg/` 下，编译时由
`CMakeLists.txt` 的 `rosidl_generate_interfaces` 生成 Python 类。

### 2.3 发布者、订阅者、回调
发布者往话题写，订阅者从话题读。订阅时注册的回调，在数据到达时被框架自动调用。
主体只有一句 `rclpy.spin(node)`，不要自己写 `while True`。

```python
# mock_telemetry_pub.py
self.obs_pub = self.create_publisher(DroneObservation, '/haique/observation', 10)
# policy_runner_node.py，第三个参数是回调
self.obs_sub = self.create_subscription(
    DroneObservation, '/haique/observation', self.observation_callback, 10)
```

回调里不要睡长觉、不要做重活。默认是单线程执行器，阻塞回调会连带卡住定时器。

### 2.4 定时器 Timer
`create_timer(周期秒, 回调)` 是 50Hz 的唯一来源，见 `mock_telemetry_pub.py` 的
`self.create_timer(1.0 / self.freq_hz, self.timer_callback)`。定时器不保证准点，
回调超时会丢拍，用 `ros2 topic hz` 看真实频率。

### 2.5 参数 Parameter
先用 `declare_parameter` 声明并给默认值，再用 `get_parameter` 读，见
`policy_runner_node.py` 的 `__init__`。覆盖方式有两种：launch 里的 `parameters=[...]`，
或命令行 `--ros-args -p max_thrust:=0.5`。当前代码在 `__init__` 校验参数，
非法值抛 `ValueError` 退出。

### 2.6 Launch 文件
`launch/mock_closed_loop.launch.py` 用三个 `Node(...)` 描述节点并返回
`LaunchDescription([...])`。`executable` 是 `setup.py` 里 `entry_points` 注册的命令名，
写错会报 executable not found；参数名写错不报错，只会用默认值。

### 2.7 工作空间、包、colcon、source
`onboard_ws` 下 `src/` 放代码，`build/`、`install/`、`log/` 是 colcon 生成物，不要手改。
包是 `src/` 下带 `package.xml` 的目录。编译完必须 `source install/setup.bash`，
否则 `ros2 run` 找不到包；顺序是先 source `/opt/ros/humble/setup.bash`。
`build_workspace.sh` 用的是裸 `colcon build`，`install/` 里是文件副本，不是符号链接；
所以改 `src/` 下的 `.py` 后必须重新执行 `./docker/build_workspace.sh`，只重启节点不生效。

### 2.8 QoS 深度
`create_publisher(..., 10)` 的 `10` 是队列深度，满了丢最旧的。深度 10 最坏会引入
200ms 排队延迟，真机压延迟时可以降到 1~2。

## 3. 逐文件精读

### 3.1 haique_interfaces：接口在这
`DroneObservation.msg` 只有结构化字段，没有 17 维扁平数组。同一份数据存两遍，早晚
会有一遍写错，所以 17 维只在 `policy_runner_node` 的一个私有方法里拼。
`PolicyAction.msg` 的 `raw_action` 只在正常 ONNX 与内置控制器路径是限幅前原始输出，
兜底路径（非有限观测/动作、超时、管线异常）里等于实际下发的安全动作、不含 NaN；
`desired_body_rates` 与 `collective_thrust` 是限幅后真正下发的值，延迟字段分别是回调内耗时与端到端耗时。
`SafetyStatus.msg` 的 `offboard_enabled` 归桥节点所有，策略节点恒置 false。
`CMakeLists.txt` 里 `rosidl_generate_interfaces` 只生成清单中列出的 `.msg`，没注册的
文件不报错但永远缺失；确认是否生成，去 `install/haique_interfaces/` 找 `_xxx.py`。

### 3.2 mock_telemetry_pub.py：假传感器
职责：没有飞机、没有仿真器时以 50Hz 产生观测，让管道能跑。它是全项目唯一产生物理
数据的节点，但数据是三角函数编的。

流程：声明 `frequency_hz` 与 `hover_thrust` 并校验；建发布者、定时器、订阅；
订阅 `/haique/policy_action` 更新 `last_action`；`timer_callback` 用 `sin/cos` 生成
相对位置误差、姿态、两个速度；ZYX 欧拉角转四元数并按 `[w,x,y,z]` 填字段；
最后设时间戳与 `frame_id='base_link_flu'` 并发布。

坑：

- 它和策略节点互为对方的发布者与订阅者，形成一条环。环里跑的是动作不是物理，
  姿态和速度不看动作，所以是开环。
- 改发布频率会改变策略收到的观测频率，看门狗阈值要一起考虑。

### 3.3 policy_runner_node.py：策略节点
职责：消费观测，产出动作和安全状态。这是最需要读懂、也最可能被改的文件。

```
__init__               校验 7 个参数，初始化推理引擎，建 1 订阅 2 发布，20Hz 看门狗
observation_callback   拼 17 维 → 查非有限 → 推理 → 查非有限 → 限幅 → 发动作
watchdog_check         超时下发安全动作并报 OBS_TIMEOUT，否则发布流水线状态
```

流程：私有方法拼长度 17 的 tuple 并断言，四元数显式写 `[q.w, q.x, q.y, q.z]`。
任一元素非有限就跳过推理，发 `(0, 0, 0, clamp(hover_thrust, min_thrust, max_thrust))`，
状态码 `NONFINITE_OBSERVATION`；回调抛异常则置 `PIPELINE_EXCEPTION`。
有 ONNX 会话就推理，否则用内置比例控制器，它读第 10~12 个元素而不是第 0~2 个。
结果再查非有限，异常发安全动作，状态码 `NONFINITE_ACTION`。限幅后记录两个延迟字段并发布。
状态码共 7 个，见 `docs/DESIGN.md` 第 4 节的表；`emergency_triggered` 只在
`OBS_TIMEOUT` 与 `PIPELINE_EXCEPTION` 时为 true。

坑：

- 限幅前必须先判有限。`max(-r, min(r, nan))` 在 CPython 下返回上限，NaN 会被放行
  成满量程角速度。
- 参数校验在 `__init__`，不要改成"修正后继续跑"，否则日志数值和实际生效值对不上。
- ONNX 失败分四种：缺 `numpy`/`onnxruntime`、维度或 dtype 不符，一律
  `RuntimeError` 退出，不受 `allow_fallback` 影响；只有"模型文件加载失败"看这个开关，
  为真时降级，状态码 `BASELINE_FALLBACK` 且 `is_safe` 为 false。
- 安全动作的推力会被夹进 `[min_thrust, max_thrust]`：0.13 的悬停标定值低于默认
  `min_thrust=0.15`，实际下发 0.15，标定时要同步下调 `min_thrust`。
- 看门狗超时后（且收到过至少一帧观测）会以 20Hz 向 `/haique/policy_action` 持续
  发布安全动作；动作仍到不了 PX4，`safety_status` 也没有订阅者。
- 超时判据取两路年龄的较大者：观测到达时刻、`header.stamp` 的年龄。只看回调执行
  时刻会漏掉队列积压——推理变慢时数据已经陈旧，回调却"刚到"。

### 3.4 px4_rates_bridge.py：坐标桥
职责：订阅动作，把 FLU 换算成 FRD（`[wx, -wy, -wz]`、`thrust_body.z = -τ`），
然后不发布任何东西。首帧打 WARN 说明结果被丢弃，非有限输入丢帧并限流告警。

坑：文件名看起来像链路已通，实际在这里断掉。仓库没有 `px4_msgs`，接通要先引入它，
再同时发 `OffboardControlMode` 心跳和 `VehicleRatesSetpoint`。换算符号不要改，
它和训练仓库 `math_utils.py` 的 `[1,-1,-1]` 是同一套约定。

### 3.5 mock_closed_loop.launch.py：启动清单
三个 `Node`：mock 观测、策略节点、坐标桥。两个节点用文件顶部的常量 `HOVER_THRUST`
保证悬停推力一致；策略节点还传限幅、看门狗，以及两个 launch 参数 `model_path`
（默认空）和 `allow_fallback`（默认 false）。`executable` 必须和 `entry_points`
完全一致，包括下划线。

启用 ONNX 模型时不必改文件，直接覆盖参数。下面路径是占位，换成实际模型；容器内
`/root/ros2_ws` 就是宿主机的 `onboard_ws`，模型文件要自己放进去：

```bash
ros2 launch haique_policy_runner mock_closed_loop.launch.py \
  model_path:=/root/ros2_ws/models/policy.onnx allow_fallback:=false
```

`policy_runner_node` 退出时，`OnProcessExit` 会触发 `Shutdown` 关闭整栈，不会出现
"控制器已退出、mock 还在打印"的假正常；桥退出只打 `LogInfo`，severity 是 INFO、文本带 `[WARN]`。

### 3.6 两个包的构建文件
`setup.py` 的 `data_files` 把 `package.xml`、`resource/` 标记、`launch/*.launch.py`
装到 `share/` 下，`entry_points` 注册三个命令指向各自的 `main`。新 launch 文件必须以
`.launch.py` 结尾，否则 `glob` 匹配不到。

`package.xml` 声明 `rclpy`、`haique_interfaces`、`geometry_msgs`、`std_msgs`，
`build_type` 是 `ament_python`，colcon 按依赖推导编译顺序；`setup.cfg` 不用改。
`haique_interfaces/package.xml` 的 `build_type` 是 `ament_cmake`，并属于
`rosidl_interface_packages` 组，缺一个消息都生成不出来。

### 3.7 docker 目录
`build_workspace.sh` 挂载 `onboard_ws` 到 `/root/ros2_ws`，在容器里 `colcon build`，
结束后把生成目录属主改回宿主机用户。`run_mock_loop.sh` 同样挂载，source 后直接
`ros2 launch`，是看数据的最短路径。`run_desktop_container.sh` 建名为
`haique_ros2_humble` 的长期容器，带 `--net=host`、X11 转发、`/dev` 挂载和
`--privileged`，所以能开 rviz2；容器基本不受限，个人机器可以接受，共用机器要谨慎。

`Dockerfile` 装 `onnxruntime`、numpy 和 Micro-XRCE-DDS-Agent，用户与工作目录和脚本
一致（root、`/root/ros2_ws`）。默认仍用官方镜像，所以容器里没有这些依赖；要跑 ONNX
先 `./docker/build_image.sh` 构建 `haique/ros2-humble:latest`，再用
`HAIQUE_IMAGE=haique/ros2-humble:latest ./docker/run_mock_loop.sh` 启动。

## 4. 一帧数据的时间线

以下为量级示意，实际取决于机器负载。

```
t = 0       mock_telemetry_publisher.timer_callback 被 50Hz 定时器触发
            - 取 last_action，即上一帧收到的动作
            - 用 sin/cos 生成 rel_position_error、orientation、两个速度
            - header.stamp = self.get_clock().now()
            - publish /haique/observation
t ≈ 0.1ms   policy_runner_node.observation_callback 被唤起
            - 拼 17 维 tuple，断言长度
            - 非有限检查：不合格发安全动作，NONFINITE_OBSERVATION，返回
            - 推理：ONNX 会话或内置基准控制器
            - 非有限检查：不合格发安全动作，NONFINITE_ACTION
            - 限幅：角速度 ±max_tilt_rate，推力 [min_thrust, max_thrust]
            - 算 inference_latency_ms 与 end_to_end_latency_ms
            - publish /haique/policy_action
t ≈ 0.2ms   px4_rates_bridge.action_callback 被唤起
            - 非有限检查，FLU 转 FRD，结果丢弃，首帧打 WARN
t ≈ 20ms    mock 的下一帧：last_action 已被上一步动作替换
每 50ms     policy_runner_node.watchdog_check
            - 年龄超过 watchdog_timeout_s: 报 OBS_TIMEOUT
            - 收到过观测时同时以 20Hz 发布安全动作 (0,0,0,clamp(hover))
            - 未超时: 发布当前流水线状态 (NORMAL / BASELINE_FALLBACK 等)
```

真正该盯的是 `end_to_end_latency_ms`：观测时间戳到 `publish()` 调用前；`inference_latency_ms`
只覆盖回调内部，不含通信与周期开销。超过 1000ms 上界视为时间戳不可信，填 -1；看门狗
下发的安全动作没有对应观测，两个延迟字段都是 -1。

## 5. 坐标系与四元数

FLU（前-左-上）是 ROS 与 Isaac Lab 的标准，FRD（前-右-下）是 PX4 的标准。
两者相差绕 X 轴 180 度，所以 X 分量不变，Y、Z 取反：

```
wx_frd =  wx_flu
wy_frd = -wy_flu
wz_frd = -wz_flu
thrust_body.z = -τ
```

记法：横滚绕前后轴，X 同向，符号不变；俯仰绕左右轴，Y 反向，取反；偏航绕上下轴，
Z 反向，取反。推力是标量，只有填进 `thrust_body` 时 z 分量取负。

四元数有两个坑：
1. 顺序。本项目统一 `[w, x, y, z]`（Hamilton，实部在前），而
   `geometry_msgs/Quaternion` 的字段定义顺序是 `x, y, z, w`，`ros2 topic echo`
   打印出来也是 `x, y, z, w`。所以拼 17 维时必须写 `[q.w, q.x, q.y, q.z]`。
2. 方向。四元数表示 body 到 world 的旋转，和 PX4 的 `q` 一致。写成 world 到 body，
   观测里的姿态就是反的。模长明显偏离 1 说明上游有 bug。

## 6. 缺陷现状

### 6.1 本轮已修复

| 编号 | 问题 | 现状 |
|---|---|---|
| B-1 | 观测顺序与契约不一致 | 删掉 17 维扁平字段，改成单一拼装点加长度断言 |
| B-2 | `last_action` 是常量 | mock 订阅动作并回填，初始值取 `hover_thrust` |
| B-3 | NaN 被限幅成满量程 | 观测与动作各查一次非有限，异常时发安全动作并置状态码 |
| B-4 | ONNX 分支不可达、失败静默降级 | 缺依赖、维度或 dtype 不符一律报错；仅加载失败受 `allow_fallback` 控制 |
| B-5 | 延迟指标误导 | 拆成推理耗时与端到端耗时两个字段 |
| B-6 | 用 `time.time()` 判超时 | 绝对时间改用 ROS 时钟，`perf_counter` 只测耗时 |
| B-7 | 参数无校验 | 按冻结清单校验范围与组合，非法值 `__init__` 抛错 |
| B-8 | `offboard_enabled` 谎报 | 策略节点恒置 false，所有权归桥节点 |
| B-9 | 桥静默丢弃结果 | 首帧 WARN，非有限输入丢帧并限流告警 |
| B-10 | 两个死消息文件 | `Observation.msg` 与 `ControlAction.msg` 已删除 |
| B-11 | 代码和日志里有 emoji | 节点日志去掉 emoji，保留 rclpy severity 与模块标签（`[PolicyRunner]` 等） |

### 6.2 仍未修复

| 问题 | 后果 | 下一步 |
|---|---|---|
| 没有飞机动力学模型 | 观测是写死的正弦曲线，动作不影响姿态，闭环不成立 | 写刚体积分节点替掉 mock |
| 桥未接 PX4 | 飞控收不到指令，端到端链路不通 | 引入 `px4_msgs`，发心跳与 setpoint |
| 没有状态估计包 | 无法从里程计生成 17 维观测 | 订阅 `/fmu/out/vehicle_odometry` 并转 FLU |
| 没有安全抢权包 | 看门狗会发安全动作，但动作到不了飞控；无 RC 抢权、无围栏、无硬急停 | 按 `HARDWARE.md` 接：CH5 解锁、CH6 三档（下=Offboard）、CH7 急停、CH8 返航 |
| `hover_thrust` 未标定 | 0.58 无实测依据，当前硬件估计约 0.13，推力偏大 | 按 `m*g/T_max` 标定 |
| 没有版本化模型管理 | `model_path` 是裸路径，换模型无法追溯 | 文件名带版本，记录训练配置 |

## 7. 动手实验

实验都在容器里做。先进入开发容器并 source 环境：

```bash
./docker/run_desktop_container.sh          # 在仓库根目录执行
# 容器内
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
```
容器里的 `/root/ros2_ws` 就是宿主机的 `onboard_ws`。

### 实验 1：看数据在流

```bash
ros2 launch haique_policy_runner mock_closed_loop.launch.py      # 终端 A
ros2 topic hz /haique/observation                                # 终端 B
ros2 topic echo /haique/observation --once
ros2 topic info /haique/safety_status -v
ros2 topic echo /haique/policy_action --once
```

验证了什么：三个节点都在，观测稳定在 50Hz，`last_action` 随动作变化而非常量。
`topic info -v` 能看到 `/haique/safety_status` 的订阅者数量是 0（上面查的就是它）。

### 实验 2：参数限幅与 fail-fast

```bash
ros2 run haique_policy_runner mock_telemetry_pub                                # 终端 A
ros2 run haique_policy_runner policy_runner_node --ros-args -p max_thrust:=0.30  # 终端 B
ros2 topic echo /haique/policy_action                                            # 终端 C
```

验证了什么：`collective_thrust` 卡在 0.30，`raw_action` 里的推力仍是原值，
说明限幅作用在原始输出之后。再试非法参数：

```bash
ros2 run haique_policy_runner policy_runner_node --ros-args -p min_thrust:=0.90
```

验证了什么：进程在 `__init__` 抛 `ValueError` 退出，不会带着非法参数继续跑。

### 实验 3：触发看门狗

先确认观测真的在流（`ros2 topic hz /haique/observation` 有输出），否则状态会停在
`WAITING_OBSERVATION`，看不到超时。

```bash
ros2 launch haique_policy_runner mock_closed_loop.launch.py   # 终端 A
ros2 topic hz /haique/observation                            # 终端 B
ros2 topic echo /haique/safety_status                        # 终端 C
pkill -f mock_telemetry_pub                                  # 终端 D
```

验证了什么：约 0.2 秒后状态码变成 `OBS_TIMEOUT`，`emergency_triggered` 为 true。
`/haique/policy_action` 转为 20Hz 的安全动作（零角速度加悬停推力），但 bridge 会丢弃，
飞控收不到；`safety_status` 依然没有订阅者。

### 实验 4：写一个最小订阅节点

新建 `/tmp/watch.py`，不要放进 `src/`，避免污染仓库。

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
            f"thrust={msg.collective_thrust:.3f} e2e={msg.end_to_end_latency_ms:.2f}ms")


rclpy.init()
node = Watcher()
rclpy.spin(node)
node.destroy_node()
rclpy.shutdown()
```

```bash
python3 /tmp/watch.py
```

验证了什么：最小节点就是继承 `Node`、注册回调、`spin`。对比 `e2e` 与
`inference_latency_ms` 的差值，那就是通信和周期开销。

### 实验 5：改输入，看连锁反应

改 `src/` 下的 `.py` 后必须重新构建，`install/` 里是文件副本不是符号链接：

```bash
./docker/build_workspace.sh
```

把 `mock_telemetry_pub.py` 里相对位置误差的系数从 `0.5` 改成 `2.0`，构建完再重启 launch：

```bash
ros2 topic echo /haique/observation --once    # rel_position_error 变大
ros2 topic echo /haique/policy_action --once  # 角速度输出跟着变大
```

验证了什么：观测到动作的因果链是通的。改姿态振幅不会影响动作，因为内置基准控制器
只读相对位置误差，这本身说明它不是一个真正的姿态控制器。试完改回 0.5。

## 8. 术语表

| 词 | 解释 |
|---|---|
| 节点 Node | 继承 `Node` 的类，一个专职程序 |
| 话题 Topic | 有名字的单向数据管道 |
| 消息 Message | 话题里数据的类型，由 `.msg` 定义 |
| 发布者 Publisher | 往话题写数据的一方 |
| 订阅者 Subscriber | 从话题读数据的一方 |
| 回调 Callback | 数据到达时框架自动调用的函数 |
| 定时器 Timer | 按固定周期调用回调，50Hz 的来源 |
| 参数 Parameter | 启动时可覆盖的数值旋钮 |
| launch | 一次启动多个节点的清单 |
| 工作空间 Workspace | `onboard_ws`，含 `src/build/install/log` |
| 包 Package | 带 `package.xml` 的代码单元 |
| colcon | ROS 2 编译工具，按依赖顺序编译包 |
| QoS depth | 队列长度，满了丢最旧的消息 |
| FLU | 机体系：前-左-上，ROS 标准 |
| FRD | 机体系：前-右-下，PX4 标准 |
| 四元数 | 表示姿态的 4 个数 `[w,x,y,z]`，无奇异 |
| Offboard | PX4 模式：外部计算机发控制指令 |
| setpoint | 设定值，希望达到的状态 |
| 归一化推力 | 推力占最大推力的比例，`[0,1]` |
| 看门狗 Watchdog | 监控心跳，超时判定故障 |
| 开环 / 闭环 | 开环执行不检查结果，闭环用结果影响下一次决策 |
