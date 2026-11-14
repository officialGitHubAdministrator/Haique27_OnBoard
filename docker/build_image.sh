#!/usr/bin/env bash
# ==============================================================================
# 海雀27机载系统 - 自定义镜像构建脚本
# 用途: 构建带 onnxruntime 的镜像，供 docker/*.sh 通过 HAIQUE_IMAGE 覆盖官方镜像
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE_TAG="haique/ros2-humble:latest"

echo "=========================================================="
echo "正在构建自定义镜像 ${IMAGE_TAG} (含 onnxruntime)"
echo "Dockerfile: ${SCRIPT_DIR}/Dockerfile"
echo "=========================================================="

# 构建上下文限定在 docker/，镜像内工作目录为 /root/ros2_ws，与运行脚本的挂载点一致
docker build -t "${IMAGE_TAG}" -f "${SCRIPT_DIR}/Dockerfile" "${SCRIPT_DIR}"

echo "=========================================================="
echo "镜像构建完成: ${IMAGE_TAG}"
echo "使用方式: HAIQUE_IMAGE=${IMAGE_TAG} ./docker/run_mock_loop.sh"
echo "=========================================================="
