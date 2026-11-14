# 海雀多旋翼强化学习机载控制系统 - 项目全景与 Agent 快速接入指南
> **文档用途**：本文件是本项目的**唯一全景事实源（Single Source of Truth）**。任何新的 AI 辅助 Agent（如 Claude / GPT / Gemini 等）进入本工作区后，**优先阅读本文件即可直接获取项目全貌与开发规范，无需用户反复解释背景**。

---

## 一、 项目背景与核心目标

* **项目代号**：海雀（Haique）多旋翼强化学习机载飞控项目。
* **业务场景**：利用强化学习（RL）在仿真环境中训练多旋翼姿态/角速度追踪策略，将导出的神经网络模型（ONNX 格式）部署在机载计算机（如板载 Linux 单板计算机 / 仿真机）上，实现 **50Hz 闭环机动飞行控制**。
* **控制方式**：**机体角速度与推力控制（Body Rates & Collective Thrust Control）**。
  * 策略输入：**17 维观测向量**（相对目标点误差、机体姿态四元数、线速度、角速度、上一时刻动作）。
  * 策略输出：**4 维动作**（FLU 机体系三轴期望角速度 $[ \omega_x, \omega_y, \omega_z ]$ + 标一化总推力 $T \in [0, 1]$）。
  * 飞控接口：通过 Micro-XRCE-DDS-Agent 与 **PX4 Autopilot (v1.14+)** 建立通信，利用 Offboard 模式向 PX4 注入 `VehicleRatesSetpoint`。

---

## 二、 运行环境与技术栈选型

* **宿主机环境**：Ubuntu 24.04 LTS (x86_64)。
* **ROS 2 版本**：**ROS 2 Humble Hawksbill**。
  * **容器化隔离架构**：由于 Ubuntu 24.04 默认使用 ROS 2 Jazzy，为保证与 PX4 稳定生态兼容，**全套 ROS 2 开发、编译与运行均运行在 Docker 镜像 `osrf/ros:humble-desktop` 内**。
  * **挂载开发模式**：宿主机负责用桌面 IDE（VS Code / Cursor）舒适写代码；Docker 容器通过挂载目录负责无污染的 `colcon build` 与节点运行。
* **语言演进路线**：
  * **第一阶段（当前）**：Python 原型（零依赖快速启动、接口校验、50Hz Mock 闭环打通）。
  * **第二阶段**：C++（`rclcpp` + ONNX Runtime C++ API）重构核心推理与坐标转换节点，消除 Python GC 停顿，保障嵌入式端 <1ms 推理硬实时。

---

## 三、 团队协作哲学（关键）

* **现状约束**：团队大部分成员对 ROS 2、CMake、坐标系转换及强化学习落地经验较为薄弱。
* **指导原则**：**“队长搭骨架与基准，组员做填空与调优”**。
* **落地策略**：
  1. 所有通信消息在 `haique_interfaces` 中硬性定义，杜绝组员间接口不一致。
  2. 提供零硬件依赖的 Mock 闭环（`mock_telemetry_pub`），让组员在没有真实飞控、没有复杂仿真器时，敲一条命令就能看到数据流流动。
  3. 代码中保留详尽中文注释、坐标系转换推导及带有 `TODO for Member` 标识的填空点。

---

## 四、 核心架构与 50Hz 数据流水线

```text
┌────────────────────────────────────────────────────────────────────────┐
│                        飞控底层 (PX4 Autopilot)                         │
└──────────────┬──────────────────────────────────────────▲──────────────┘
               │ (Micro-XRCE-DDS: uORB 话题)               │
               ▼ /fmu/out/vehicle_odometry (FRD)          │ /fmu/in/vehicle_rates_setpoint (FRD)
┌──────────────────────────────────────────┐              │
│ 1. 状态估计节点 (haique_state_estimator)  │              │
│    - FRD 坐标系 -> FLU 机体系变换         │              │
│    - 拼接打包 17 维观测向量              │              │
└──────────────┬───────────────────────────┘              │
               │ /haique/observation (DroneObservation.msg, 50Hz)
               ▼                                          │
┌──────────────────────────────────────────┐              │
│ 2. 策略调度节点 (haique_policy_runner)    │              │
│    - ONNX 模型推理 (或内置基准策略)      │              │
│    - 动作限幅与观测丢包看门狗监控        │              │
└──────────────┬───────────────────────────┘              │
               │ /haique/policy_action (PolicyAction.msg, 50Hz)
               ▼                                          │
┌──────────────────────────────────────────┐              │
│ 3. 飞控适配桥 (px4_rates_bridge)         │              │
│    - FLU 机体系 -> FRD 坐标系转换        │              │
│    - 发布 Offboard 模式心跳与期望角速度   ├──────────────┘
└──────────────────────────────────────────┘
```

### 坐标系转换核心约定（必读）
* **机载算法统一标准**：**FLU（前-左-上，Front-Left-Up）**。
* **PX4 飞控底层标准**：**FRD（前-右-下，Front-Right-Down）**。
* **角速度转换公式**：
  $$\omega_{x, FRD} = \omega_{x, FLU}$$
  $$\omega_{y, FRD} = -\omega_{y, FLU}$$
  $$\omega_{z, FRD} = -\omega_{z, FLU}$$
  $$\text{thrust}_{FRD} = [0, 0, -T_{collective}]^T$$

---

## 五、 工程目录结构与资产清单

```text
DRONE_Workspace/
├── PROJECT_AGENT_BRIEF.md          # [本文件] 项目全景与 Agent 快速接入指南
├── README.md                       # 快速引导说明
├── docker/                         # Docker 容器化环境工具集
│   ├── Dockerfile                  # 自定义 ROS 2 Humble + Micro-XRCE-DDS 镜像配方
│   ├── run_desktop_container.sh    # 交互式桌面容器启动脚本 (支持 RViz2 X11 穿透)
│   ├── build_workspace.sh          # 容器化一键编译脚本 (无需进入容器)
│   └── run_mock_loop.sh            # 容器化一键运行 50Hz 虚拟闭环
├── onboard_ws/                     # ROS 2 机载工作空间 (挂载入容器 /root/ros2_ws)
│   └── src/
│       ├── haique_interfaces/      # [已就绪] 统一接口定义包 (CMake/rosidl)
│       │   └── msg/
│       │       ├── DroneObservation.msg  # 17 维统一机载观测向量
│       │       ├── PolicyAction.msg      # 4 维机体角速度与推力动作
│       │       └── SafetyStatus.msg      # 状态监控与急停报警消息
│       ├── haique_policy_runner/   # [已就绪] 策略调度与 50Hz Mock 核心功能包
│       │   ├── haique_policy_runner/
│       │   │   ├── mock_telemetry_pub.py # 50Hz 虚拟遥测发生器 (零硬件依赖)
│       │   │   ├── policy_runner_node.py # 策略推理、时延诊断与看门狗
│       │   │   └── px4_rates_bridge.py   # FLU->FRD 坐标转换与 PX4 桥接模板
│       │   └── launch/
│       │       └── mock_closed_loop.launch.py # 闭环联合启动文件
│       ├── haique_state_estimator/ # [待分配] 真实 PX4 里程计转 17 维观测包
│       └── haique_safety_guard/    # [待分配] RC 遥控器抢权、围栏与硬急停保护包
└── simulation/                     # 强化学习与 SITL 仿真资产目录 (待展开)
```

---

## 六、 常用操作命令（开箱即用）

所有操作均已封装为免配环境的脚本，直接在宿主机工作区根目录执行：

### 1. 一键编译工作区
```bash
./docker/build_workspace.sh
```

### 2. 一键运行 50Hz 虚拟闭环验证
```bash
./docker/run_mock_loop.sh
```
*控制台将打印 50Hz 遥测帧率、单步推理耗时（ms）及动作发布状态。*

### 3. 进入交互式 Docker 开发终端（支持图形界面）
```bash
./docker/run_desktop_container.sh
```

---

## 七、 后续 Agent 开发指引与待办任务（Roadmap）

当有新的 Agent 协助用户时，请根据以下优先级直接承接任务，无需再从零搭建基础：

1. **真实状态估计包（`haique_state_estimator`）**：
   * 编写节点订阅 `/fmu/out/vehicle_odometry`。
   * 实现 FRD 坐标系转 FLU 坐标系、机体线速度/角速度提取。
   * 结合用户指定的目标航路点，组装为 `DroneObservation.msg` 并替换掉 `mock_telemetry_pub`。
2. **遥控器抢权与安全卫士（`haique_safety_guard`）**：
   * 监听遥控器拨杆（Channel 5 切 Offboard，Channel 6 急停返航）。
   * 一旦遥控器推杆介入或姿态超过 45°，立即向飞控注入降级/自稳指令。
3. **模型导入与 C++ 推理重构**：
   * 将 RL 导出的 `.onnx` 模型放入模型目录，并在 `policy_runner_node` 中挂载推理。
   * 为主力组员提供 `haique_policy_runner_cpp` 的 C++ 模版与 CMake 配置。
