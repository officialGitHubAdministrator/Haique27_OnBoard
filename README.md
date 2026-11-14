# 海雀 (Haique) 机载控制工作区

7 寸四旋翼强化学习项目的机载部分，运行在机载 Linux 计算机上，ROS 2 Humble。
策略以 50Hz 输出三轴期望角速度与归一化总推力，交给 PX4 的内层角速度环执行；
神经网络不直接输出电机 PWM。

当前能跑通的只有一条 50Hz 模拟数据流水线。没有飞控、没有真实状态估计，
`px4_rates_bridge` 只做坐标换算，不向 PX4 发送任何 setpoint。

## 目录结构

```text
DRONE_Workspace/
├── docs/
│   ├── DESIGN.md                  # 系统边界、架构、接口契约、参数、已知限制
│   └── CODE_GUIDE.md              # 带人读代码：ROS 2 概念、逐文件、动手实验
├── docker/
│   ├── Dockerfile                 # 自定义镜像配方，含 onnxruntime 与 DDS Agent
│   ├── build_image.sh             # 构建 haique/ros2-humble:latest
│   ├── build_workspace.sh         # 在容器里执行 colcon build（文件副本，不是符号链接）
│   ├── run_mock_loop.sh           # 启动 50Hz 模拟流水线
│   └── run_desktop_container.sh   # 交互式开发容器，可用 rviz2
├── onboard_ws/
│   └── src/
│       ├── haique_interfaces/     # 消息定义包，CMake + rosidl 构建
│       └── haique_policy_runner/  # 三个节点，Python 构建
└── simulation/
    └── Haique27_RL -> /home/wang/Haique27_RL   # 训练仓库软链接
```

## 编译与运行

宿主机是 Ubuntu 24.04，ROS 2 Humble 装在容器里（镜像 `osrf/ros:humble-desktop`），编译与运行都在容器内完成。

```bash
# 编译 onboard_ws（容器内执行 colcon build，结束后修复文件属主）
./docker/build_workspace.sh

# 启动模拟流水线：mock 观测 + 策略 + 坐标桥，三个节点一起起来，Ctrl+C 停止
./docker/run_mock_loop.sh

# 进入交互式开发容器（想单独敲 ros2 命令时用）
./docker/run_desktop_container.sh

# 可选：要跑 ONNX 模型时先构建自定义镜像，再用 HAIQUE_IMAGE 指定
./docker/build_image.sh
HAIQUE_IMAGE=haique/ros2-humble:latest ./docker/run_mock_loop.sh
```

进入容器后先 source 两层环境，再启动：

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch haique_policy_runner mock_closed_loop.launch.py
```

## 当前状态

| 能力 | 状态 |
|---|---|
| 50Hz 模拟观测发布 | 已实现，数据是三角函数 |
| 17 维观测拼装、4 维动作发布 | 已实现，顺序与契约一致 |
| 推力限幅、角速度限幅、参数校验 | 已实现 |
| 非有限值拦截、看门狗超时下发安全动作 | 已实现 |
| ONNX 模型加载与推理路径 | 代码路径已实现，仓库环境未验证：默认镜像无 onnxruntime、无 .onnx、未构建自定义镜像 |
| FLU 到 FRD 角速度换算 | 已实现，换算结果被丢弃 |
| 向 PX4 发 offboard 心跳与 setpoint | 未实现 |
| 真实状态估计（PX4 里程计转观测） | 未实现 |
| 飞机动力学与真实闭环 | 未实现 |
| 遥控器抢权、安全守卫 | 未实现 |

## 文档索引

设计说明见 [docs/DESIGN.md](docs/DESIGN.md)，代码走读见 [docs/CODE_GUIDE.md](docs/CODE_GUIDE.md)，训练仓库在 `/home/wang/Haique27_RL`。
