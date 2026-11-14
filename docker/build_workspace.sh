#!/usr/bin/env bash
# ==============================================================================
# 海雀27机载系统 - 容器化一键编译脚本
# 用途: 在无需宿主机配置 ROS 2 环境的情况下，直接调用 Humble 容器编译代码
# ==============================================================================

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ONBOARD_WS="${WORKSPACE_DIR}/onboard_ws"
IMAGE_NAME="osrf/ros:humble-desktop"

echo "=========================================================="
echo "🔨 正在调用 ROS 2 Humble 容器编译 onboard_ws ..."
echo "📂 工作区目录: ${ONBOARD_WS}"
echo "=========================================================="

docker run --rm \
    -v "${ONBOARD_WS}:/root/ros2_ws" \
    -w /root/ros2_ws \
    "${IMAGE_NAME}" \
    bash -c "source /opt/ros/humble/setup.bash && colcon build"

# 修复编译生成文件的宿主机权限 (避免 root 占有导致宿主机 IDE 无法读写)
if [ -n "$SUDO_UID" ]; then
    chown -R ${SUDO_UID}:${SUDO_GID} "${ONBOARD_WS}/build" "${ONBOARD_WS}/install" "${ONBOARD_WS}/log" 2>/dev/null || true
else
    sudo chown -R $(id -u):$(id -g) "${ONBOARD_WS}/build" "${ONBOARD_WS}/install" "${ONBOARD_WS}/log" 2>/dev/null || true
fi

echo "=========================================================="
echo "✅ 编译成功！"
echo "💡 快速测试运行: ./docker/run_mock_loop.sh"
echo "=========================================================="
