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

# 配置用户与工作区权限 (匹配宿主机用户 UID=1000)
ARG USERNAME=wang
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# 自动 source ROS 2 环境
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /root/ros2_ws/install/setup.bash ]; then source /root/ros2_ws/install/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /home/$USERNAME/ros2_ws/install/setup.bash ]; then source /home/$USERNAME/ros2_ws/install/setup.bash; fi" >> /home/$USERNAME/.bashrc

USER $USERNAME
WORKDIR /home/$USERNAME/ros2_ws

CMD ["/bin/bash"]
