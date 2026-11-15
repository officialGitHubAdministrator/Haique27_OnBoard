# 海雀机载协作规范

写给团队成员，也写给在仓库里干活的编码 Agent。目标是用最少的流程守住飞行代码的几条安全底线：
日常改动开一个 PR、一位队友看过就能合；只有改到第 3 节列出的地方，才需要多贴一点证据。

规则用着别扭，直接提 PR 改这份文件。

## 1. 日常流程

```text
从默认分支拉分支 → 改 → 本地验证 → 开 PR → 一位队友 approve → squash 合并
```

- 不直接 push 默认分支，不 force push 别人的分支。管理员按第 9 节设置一次分支保护，之后靠 GitHub
  拦，不靠自觉。
- approve 之后由作者自己点 Squash and merge。
- 分支名看得懂就行，建议 `feat/xxx`、`fix/xxx`、`docs/xxx`。
- 一个 PR 做一件事。重构、格式化和功能分开提，review 的人才看得过来。
- 纯文档、注释、错字不用跑本地验证。
- 合并后出了问题，先 revert 那个 PR 让大家能继续干活，再慢慢查。

## 2. 本地验证

改了代码，PR 里贴下面三项的实际输出：

```bash
./docker/build_workspace.sh       # 1. 编译通过

# 终端 A：进入开发容器，source 后启动 mock 流水线
./docker/run_desktop_container.sh
source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash
ros2 launch haique_policy_runner mock_closed_loop.launch.py   # 2. 保持运行

# 终端 B：再执行一次 run_desktop_container.sh 进入同一容器，同样先 source 两行
ros2 topic hz /haique/observation              # 3a. 约 50Hz
ros2 topic echo /haique/safety_status --once   # 3b. status_code: NORMAL
```

这里要让 launch 和检查命令在同一个开发容器里运行。`run_mock_loop.sh` 使用独立 IPC 命名空间，
另一个容器虽能发现话题，但 DDS 数据收不到；如果需要跨容器检查，先讨论是否给它加 `--ipc=host`。

改了 `src/` 一定要重新执行 `./docker/build_workspace.sh`，`install/` 里是文件副本，只重启节点不会生效。

仓库目前没有 CI，也没有自动化测试。PR 里不要写“测试通过”，贴命令输出。

## 3. 高风险改动

改到下表左列就算高风险，不用自己判断。PR 标题前加 `[高风险]`，除第 2 节外再贴右列的证据。
review 的人要真的看这些证据，看不懂就不 approve；尽量找熟悉这块代码的人 review。

| 改到哪里 | 为什么要小心 | 额外贴什么 |
|---|---|---|
| `haique_interfaces/msg/*.msg`，或 17 维观测的拼装顺序 | 和训练仓库对不上，模型会吃错输入，而且不报错 | 训练仓库 `docs/INTERFACE_CONTRACT.md` 对应改动的链接 |
| `policy_runner_node.py` 的任何改动（只改日志文字或注释除外） | 观测校验、限幅、看门狗都在这里，是现在唯一的安全兜底 | CODE_GUIDE 实验 3 的输出：`OBS_TIMEOUT`、`emergency_triggered: true`、`/haique/policy_action` 约 20Hz。动了限幅或参数校验，再贴实验 2 的输出 |
| `px4_rates_bridge.py` 的任何改动，任何地方的四元数顺序 | 符号错了，飞机会往反方向转 | 见表下的符号检查 |
| `hover_thrust`、`min_thrust`、`max_thrust`、`max_tilt_rate`、`watchdog_timeout_s`、`allow_fallback` 的默认值，包括 launch 里的值 | 推力或限幅错了，上真机就是事故 | 新值从哪来（实测或计算），见 DESIGN 的“节点参数与校验” |
| `docker/`、两个 `package.xml`、新依赖 | 影响所有人的开发环境 | 删掉 `onboard_ws/` 下的 `build/`、`install/`、`log/` 后重跑第 2 节。改了 `Dockerfile` 还要跑 `./docker/build_image.sh`，`docker rm -f haique_ros2_humble`，再用 `HAIQUE_IMAGE=haique/ros2-humble:latest` 重跑第 2 节 |

实验 2、3 也在这个开发容器里做。

桥的符号检查：不要同时开 launch，在开发容器的两个终端里分别执行

```bash
ros2 run haique_policy_runner px4_rates_bridge --ros-args --log-level debug
ros2 topic pub --once /haique/policy_action haique_interfaces/msg/PolicyAction \
  "{desired_body_rates: {x: 0.1, y: 0.2, z: 0.3}, collective_thrust: 0.5}"
```

第一个终端应打印 `FLU->FRD: roll=0.1000, pitch=-0.2000, yaw=-0.3000, thrust_z=-0.5000`。

## 4. 红线

不管多赶，都不要做：

- 为了让程序跑通，删掉或放宽 `frame_id`、四元数模长、非有限值检查、限幅、看门狗。
- 凭感觉改 FLU/FRD 符号，或四元数的 `[w, x, y, z]` 顺序。
- 只改本仓库或只改训练仓库的接口定义，另一边不管。
- 调大 `watchdog_timeout_s` 来消掉超时报错，而不去查延迟从哪来。
- 提交 `build/`、`install/`、`log/`、模型文件、bag 数据或任何密钥。`.gitignore` 挡住了其中大部分，
  只忽略 `.env` 和 `.env.*`（`.env.example` 除外）；其他密钥文件自己留意，不要用 `git add -f` 绕过。
- 接上 PX4 以后，没拆桨、飞机没固定，就测试会让电机转起来的链路。

## 5. 用 Agent 写代码

- 让 Agent 开工前先读本文件第 3、4 节和 `docs/DESIGN.md`。
- Agent 说“已完成”“测过了”不算数，以 PR 里贴的命令输出为准。
- Agent 生成的 diff，人先读一遍再开 PR。改到第 3 节的文件要逐行读，不能只看 Agent 的总结。
- 不让 Agent 合并 PR、force push、改分支保护或 `.github/`。
- 不把 `.env`、token、SSH 密钥交给 Agent。
- 同一个问题 Agent 连续两次没修好，就停下来自己看，或者在群里问，别让它继续叠补丁。

## 6. PR 描述

三段就够，没有的段落删掉：

```markdown
## 改了什么、为什么

## 怎么验证的
命令和关键输出；纯文档改动写“文档改动”。

## 高风险项
对照第 3 节，贴额外证据。
```

模板在 `.github/pull_request_template.md`，GitHub 开 PR 时会自动填入；要改模板就在这里改。

## 7. 文档跟着代码走

改了接口契约、参数、安全语义或已知限制，在同一个 PR 里更新 `docs/DESIGN.md`。README 的状态表只写
验证过的事实。同一件事只在一处写清楚，其他地方链接过去。

## 8. 先问再动手

遇到下面的情况，先在 issue 或群里说清楚想做什么、为什么、试过什么，再开工：

- 要改 `.msg` 或 17 维顺序，但训练仓库那边还没商量好。
- 要引入 `px4_msgs` 或其他新依赖，或者改 docker 的权限、网络、设备设置。
- 改动越做越大，已经不是一个 PR 能讲清的事。

## 9. 仓库管理员一次性设置

- 在 GitHub 的 Settings → Branches 给默认分支加保护规则：合并前必须走 PR，至少 1 个 approve，
  并勾选不允许管理员绕过（Do not allow bypassing the above settings），否则管理员不受限制。
- 公开仓库免费可用。私有仓库在免费账户下 GitHub 不执行分支保护：个人仓库需要 GitHub Pro
  （学生可通过 GitHub Education 免费申请），组织仓库需要 GitHub Team。做不到就只能靠第 1 节的约定。
- Settings → General 里只保留 Allow squash merging。
- 确认 `.github/pull_request_template.md` 已入库。
