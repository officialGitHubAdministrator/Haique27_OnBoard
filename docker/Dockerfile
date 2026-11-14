# ROS 2 Humble Desktop with Micro-XRCE-DDS and ONNX Runtime
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# 替换为国内镜像源加速 apt (可选)，安装基础依赖与编译工具
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    nano \
    sudo \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-rmw-cyclonedds-cpp \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# 安装 Python 推理库与科学计算依赖
RUN pip3 install --no-cache-dir \
    numpy \
    onnx \
    onnxruntime \
    transforms3d \
    scipy

# 编译并安装 Micro-XRCE-DDS-Agent (PX4 v1.14 官方通信中间件)
RUN git clone -b v2.4.1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent \
    && cd /tmp/Micro-XRCE-DDS-Agent \
    && mkdir build && cd build \
    && cmake .. \
    && make -j$(nproc) \
    && make install \
    && ldconfig /usr/local/lib/ \
    && rm -rf /tmp/Micro-XRCE-DDS-Agent

# 与 docker/*.sh 保持一致：脚本以 root 运行并挂载工作区到 /root/ros2_ws。
# 自动 source ROS 2 环境
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "if [ -f /root/ros2_ws/install/setup.bash ]; then source /root/ros2_ws/install/setup.bash; fi" >> /root/.bashrc

USER root
WORKDIR /root/ros2_ws

CMD ["/bin/bash"]
