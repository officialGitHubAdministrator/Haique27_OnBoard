# 海雀机载控制系统工作区

7 寸四旋翼强化学习项目的机载部分，ROS 2 Humble，Python 节点，运行在机载 Linux 计算机上。
策略正常以 50Hz 输出三轴期望角速度和归一化总推力；观测超时期间改为 20Hz 悬停安全动作。
设计上由 PX4 的内层角速度环执行，不直接输出电机 PWM。

**当前代码不能用于真机飞行。** 没有 PX4 链路、没有真实状态估计、没有 RC 抢权/围栏/硬急停。
默认悬停推力 `hover_thrust=0.58` 未标定，真机前必须重新标定。
`px4_rates_bridge` 只做 FLU 到 FRD 的换算，结果直接丢弃。

## 已跑通部分

唯一跑得通的是零硬件 mock 开环流水线。`mock_telemetry_pub.py` 以 50Hz 发布三角函数观测；
`policy_runner_node.py` 拼装 17 维观测、推理或使用内置基准控制器、限幅并发布 4 维动作；
`px4_rates_bridge.py` 订阅动作做坐标换算。观测不随动作变化，闭环不成立；launch 文件名仍叫
`mock_closed_loop`，名字是历史遗留。

| 能力 | 状态 |
|---|---|
| 50Hz mock 观测与 17 维拼装 | 已实现 |
| 4 维动作、限幅、参数校验、看门狗、状态发布 | 已实现 |
| FLU 到 FRD 换算 | 已实现（结果被丢弃） |
| ONNX 模型加载与推理 | 未验证 |
| PX4 offboard 心跳与 setpoint | 未实现 |
| 真实状态估计、飞机动力学 | 未实现 |
| RC 抢权、围栏、硬急停 | 未实现 |
| 自动化测试 | 无 |

ONNX 分支的加载、形状校验和推理代码存在，但默认镜像没有 onnxruntime，仓库也没有 `.onnx` 模型，未验证；自定义镜像见下文。

## 构建与运行

开发环境是 Ubuntu 24.04 宿主机 + Docker 容器。ROS 2 Humble 在容器里，默认镜像
`osrf/ros:humble-desktop`，编译和运行都在容器内完成。

```bash
./docker/build_workspace.sh        # 容器内 colcon build，结束后修复生成文件属主
./docker/run_mock_loop.sh          # mock 观测 + 策略 + 坐标桥，Ctrl+C 停止
./docker/run_desktop_container.sh  # 交互式开发容器，可执行 ros2 命令或开 rviz2
```

ONNX 推理需要带 onnxruntime 的自定义镜像，并在启动时显式传入模型路径：

```bash
./docker/build_image.sh
HAIQUE_IMAGE=haique/ros2-humble:latest ./docker/run_desktop_container.sh
# 容器内
source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash
ros2 launch haique_policy_runner mock_closed_loop.launch.py \
  model_path:=/root/ros2_ws/models/policy.onnx
```

模型放在宿主机的 `onboard_ws/models/`，对应容器内的 `/root/ros2_ws/models/`；模型文件不入库。
`run_mock_loop.sh` 不转发 launch 参数，所以要像上面这样手动 `ros2 launch`。`run_desktop_container.sh`
会复用已有的 `haique_ros2_humble` 容器；如果它是用默认镜像建的，先 `docker rm -f haique_ros2_humble`
删掉，再启动才会用上自定义镜像。

## 验证

仓库没有自动化测试，也没有 CI。手动回归是三项：编译通过、启动 mock 流水线、另开一个容器确认观测约 50Hz
且安全状态为 `NORMAL`。推荐在 `run_desktop_container.sh` 进入的同一个开发容器里启动和检查；`run_mock_loop.sh`
使用独立 IPC 命名空间，跨容器订阅可能收不到数据。命令见 [CONTRIBUTING.md](CONTRIBUTING.md) 的“本地验证”。

## 目录

```text
DRONE_Workspace/
├── CONTRIBUTING.md            # 团队协作流程与评审规则
├── docs/{DESIGN.md,CODE_GUIDE.md}
├── docker/                    # 镜像构建、工作区编译、运行脚本
└── onboard_ws/src/
    ├── haique_interfaces/     # 三个 .msg，ament_cmake + rosidl
    └── haique_policy_runner/  # 三个节点，ament_python
```

## 文档

先读 [docs/DESIGN.md](docs/DESIGN.md) 了解系统边界和接口契约，再读
[docs/CODE_GUIDE.md](docs/CODE_GUIDE.md) 逐文件看代码。提 PR 前读
[CONTRIBUTING.md](CONTRIBUTING.md)。

训练仓库单独维护，不在本仓库内；跨仓库接口以其 `docs/INTERFACE_CONTRACT.md` 为基准，当前为 v4，
已与本仓库实现对齐。再改接口时两边同步修改，见 [docs/DESIGN.md](docs/DESIGN.md) 开头的约定。
