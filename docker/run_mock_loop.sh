#!/usr/bin/env bash
# ==============================================================================
# 海雀27机载系统 - 零硬件依赖虚拟闭环快速运行脚本
# 用途: 启动 50Hz 遥测模拟 + 策略调度 + 坐标系转换全闭环
# ==============================================================================

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ONBOARD_WS="${WORKSPACE_DIR}/onboard_ws"
IMAGE_NAME="osrf/ros:humble-desktop"

echo "=========================================================="
echo "🚀 正在启动 50Hz 零硬件依赖闭环测试 (Docker 容器内)"
echo "💡 按 Ctrl+C 可停止测试"
echo "=========================================================="

docker run --rm -it \
    --net=host \
    -v "${ONBOARD_WS}:/root/ros2_ws" \
    -w /root/ros2_ws \
    "${IMAGE_NAME}" \
    bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch haique_policy_runner mock_closed_loop.launch.py"
