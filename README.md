# 海雀 (Haique) 多旋翼强化学习机载控制系统

欢迎来到海雀项目机载 ROS 2 工作区！

> 💡 **给所有 AI 助手与新加入组员的必读指引**：
> 本项目的全景架构、数据流定义、环境规范及团队分工已完整记录在：
> 👉 **[PROJECT_AGENT_BRIEF.md](file:///home/wang/桌面/DRONE_Workspace/PROJECT_AGENT_BRIEF.md)**
> 在使用任何新 Agent 或向新成员介绍时，直接引用或阅读该文档即可，无需重复说明背景！

---

## ⚡ 30 秒极速上手

本工作区通过 Docker（ROS 2 Humble）实现了“宿主机写代码、容器内秒编译运行”，宿主机无需配置任何复杂 ROS 环境。

### 1. 编译工作区
```bash
./docker/build_workspace.sh
```

### 2. 运行 50Hz 零硬件依赖虚拟闭环测试
```bash
./docker/run_mock_loop.sh
```

### 3. 进入交互式开发容器
```bash
./docker/run_desktop_container.sh
```
进入后可直接运行 `rviz2`，图形界面将无缝穿透显示在宿主机桌面上。
