#!/usr/bin/env bash
# ==============================================================================
# 海雀27机载 ROS 2 Humble 桌面开发容器启动脚本
# 支持 X11 图形界面穿透 (RViz2 / rqt)、主机网络共享与目录无缝挂载
# ==============================================================================

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ONBOARD_WS="${WORKSPACE_DIR}/onboard_ws"
CONTAINER_NAME="haique_ros2_humble"
IMAGE_NAME="osrf/ros:humble-desktop"

echo "=========================================================="
echo "🚀 启动海雀27 ROS 2 Humble 桌面容器环境"
echo "📂 宿主机挂载目录: ${ONBOARD_WS}"
echo "=========================================================="

# 允许本地 X11 图形界面显示穿透
xhost +local:root > /dev/null 2>&1 || true
xhost +local:$USER > /dev/null 2>&1 || true

# 检查容器是否已经在运行
if [ "$(docker ps -q -f name=^/${CONTAINER_NAME}$)" ]; then
    echo "💡 检测到容器已经在后台运行，直接进入终端..."
    docker exec -it "${CONTAINER_NAME}" bash
    exit 0
fi

# 检查容器是否处于已创建/停止状态
if [ "$(docker ps -aq -f name=^/${CONTAINER_NAME}$)" ]; then
    echo "🔄 发现已存在的容器，正在启动并进入..."
    docker start "${CONTAINER_NAME}" > /dev/null 2>&1 || docker rm -f "${CONTAINER_NAME}" > /dev/null
    if [ "$(docker ps -q -f name=^/${CONTAINER_NAME}$)" ]; then
        docker exec -it "${CONTAINER_NAME}" bash
        exit 0
    fi
fi

# 检测是否真正支持 Docker NVIDIA GPU 穿透 (需同时具备驱动和 nvidia-container-toolkit)
GPU_FLAGS=""
if command -v nvidia-smi &> /dev/null && command -v nvidia-container-toolkit &> /dev/null; then
    GPU_FLAGS="--gpus all"
    echo "⚡ 检测到已配置 NVIDIA 容器运行时，已启用 GPU 穿透"
else
    echo "🖥️  使用标准 CPU / OpenGL 图形渲染 (RViz2 可正常流畅运行)"
fi

# 如果宿主机存在 DRI 设备 (核显/独显直接渲染节点)，进行设备映射加速图形
DRI_FLAGS=""
if [ -d "/dev/dri" ]; then
    DRI_FLAGS="--device /dev/dri:/dev/dri"
fi

echo "🐳 正在创建并启动新开发容器 [${CONTAINER_NAME}]..."

docker run -it \
    --name "${CONTAINER_NAME}" \
    --net=host \
    --ipc=host \
    --privileged \
    ${GPU_FLAGS} \
    ${DRI_FLAGS} \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev:/dev \
    -v "${ONBOARD_WS}:/root/ros2_ws" \
    -w /root/ros2_ws \
    "${IMAGE_NAME}" \
    bash -c '
        echo "========================================================"
        echo "✅ ROS 2 Humble 桌面环境已就绪！"
        echo "💡 提示："
        echo "  - 输入 rviz2 即可在宿主机桌面弹出 RViz 图形界面"
        echo "  - 工作目录已挂载至 /root/ros2_ws (对应桌面的 onboard_ws)"
        echo "  - 编译工作区命令: colcon build --symlink-install"
        echo "========================================================"
        exec bash
    '
