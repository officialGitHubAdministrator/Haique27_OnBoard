#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
海雀机载系统 - 零硬件依赖虚拟遥测发布节点 (Mock Telemetry Publisher)
用途: 
  在未连接 PX4 飞控或仿真器的情况下，以 50Hz 产生拟真无人机运动观测数据。
  帮助组员在宿主机或 Docker 容器内迅速验证 50Hz 闭环数据流、通信时延与节点健康。
"""

import math
import time
import rclpy
from rclpy.node import Node
from haique_interfaces.msg import DroneObservation
from geometry_msgs.msg import Point, Quaternion, Vector3


class MockTelemetryPublisher(Node):
    def __init__(self):
        super().__init__('mock_telemetry_publisher')

        # 声明参数
        self.declare_parameter('frequency_hz', 50.0)
        self.freq_hz = self.get_parameter('frequency_hz').get_parameter_value().double_value
        timer_period = 1.0 / self.freq_hz

        # 创建发布者
        self.obs_pub = self.create_publisher(
            DroneObservation, 
            '/haique/observation', 
            10
        )

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = time.time()
        self.tick_count = 0
        self.last_action = [0.0, 0.0, 0.0, 0.58]  # 初始假动作 (悬停推力 ~0.58)

        self.get_logger().info(
            f"✅ [MockTelemetryPublisher] 已启动，模拟数据正以 {self.freq_hz:.1f}Hz 发往 /haique/observation"
        )

    def timer_callback(self):
        now_time = self.get_clock().now()
        t = time.time() - self.start_time
        self.tick_count += 1

        msg = DroneObservation()
        msg.header.stamp = now_time.to_msg()
        msg.header.frame_id = 'base_link_flu'

        # 1. 模拟目标相对位置误差 (模拟无人机在 3D 空间微小飘动逼近目标点)
        rel_x = 0.5 * math.cos(0.5 * t)
        rel_y = 0.5 * math.sin(0.5 * t)
        rel_z = 0.1 * math.sin(1.0 * t)
        msg.rel_position_error = Point(x=rel_x, y=rel_y, z=rel_z)

        # 2. 模拟微小姿态角变换 (小角度倾斜四元数)
        roll = 0.05 * math.sin(1.2 * t)
        pitch = 0.05 * math.cos(1.2 * t)
        yaw = 0.1 * math.sin(0.3 * t)

        # 简易欧拉角转四元数 (小角近似)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        msg.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)

        # 3. 模拟机体线速度 (m/s, FLU)
        msg.body_linear_velocity = Vector3(
            x=-0.25 * math.sin(0.5 * t),
            y=0.25 * math.cos(0.5 * t),
            z=0.1 * math.cos(1.0 * t)
        )

        # 4. 模拟机体角速度 (rad/s, FLU)
        msg.body_angular_velocity = Vector3(
            x=0.06 * math.cos(1.2 * t),
            y=-0.06 * math.sin(1.2 * t),
            z=0.03 * math.cos(0.3 * t)
        )

        # 5. 上一时刻动作反馈 [roll_rate, pitch_rate, yaw_rate, thrust]
        msg.last_action = self.last_action

        # 6. 打包成 17 维连续数组 (张量入口)
        # [rel_pos(3), quat(4), lin_vel(3), ang_vel(3), last_action(4)]
        msg.flat_obs = [
            float(rel_x), float(rel_y), float(rel_z),
            float(qw), float(qx), float(qy), float(qz),
            float(msg.body_linear_velocity.x), float(msg.body_linear_velocity.y), float(msg.body_linear_velocity.z),
            float(msg.body_angular_velocity.x), float(msg.body_angular_velocity.y), float(msg.body_angular_velocity.z),
            float(self.last_action[0]), float(self.last_action[1]), float(self.last_action[2]), float(self.last_action[3])
        ]

        self.obs_pub.publish(msg)

        # 每 100 帧 (约2秒) 打印一次状态，保持控制台整洁
        if self.tick_count % 100 == 0:
            self.get_logger().info(
                f"[MockTelemetry] 累计发送 {self.tick_count} 帧 | 目标距: ({rel_x:.2f}, {rel_y:.2f}, {rel_z:.2f})m"
            )


def main(args=None):
    rclpy.init(args=args)
    node = MockTelemetryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
