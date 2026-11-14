#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
海雀机载系统 - 零硬件依赖虚拟遥测发布节点 (Mock Telemetry Publisher)
用途:
  在未连接 PX4 飞控或仿真器的情况下，按 frequency_hz 产生拟真无人机运动观测数据，
  用于验证以 frequency_hz 发布的开环虚拟遥测数据流、通信时延与节点健康。
  本节点姿态/速度/位置是时间的纯函数，策略动作只被回显，不参与状态演化。

约定:
  本节点只填充结构化字段，不拼装 17 维扁平观测向量；拼装唯一入口在 policy_runner_node。
"""

import math
import sys

import rclpy
from rclpy.exceptions import InvalidParameterTypeException
from rclpy.node import Node
from haique_interfaces.msg import DroneObservation, PolicyAction
from geometry_msgs.msg import Point, Quaternion, Vector3


class MockTelemetryPublisher(Node):
    def __init__(self):
        super().__init__('mock_telemetry_publisher')

        # 参数先声明、校验，再创建定时器：非法配置 fail-fast，不留下半启动状态。
        # 注意：hover_thrust 默认值 0.58 没有实测依据，仅保持既有行为不变。
        # 真机前需按 tau_hover = m*g / T_max 重新标定，当前硬件约 0.13。
        try:
            self.declare_parameter('frequency_hz', 50.0)
        except InvalidParameterTypeException as exc:
            # 整数覆盖会在 declare_parameter 阶段抛类型异常，转成可读的 ValueError。
            raise ValueError('frequency_hz 必须是浮点数，请写成 100.0 而不是 100') from exc
        try:
            self.declare_parameter('hover_thrust', 0.58)
        except InvalidParameterTypeException as exc:
            raise ValueError('hover_thrust 必须是浮点数，请写成 1.0 而不是 1') from exc

        freq_hz = float(self.get_parameter('frequency_hz').value)
        hover_thrust = float(self.get_parameter('hover_thrust').value)
        self._validate_parameters(freq_hz, hover_thrust)
        self.freq_hz = freq_hz
        self.hover_thrust = hover_thrust

        # 上一步动作反馈的初值：角速度为 0，推力取悬停值。
        # 之后由 /haique/policy_action 回调整表替换，不在原地修改。
        self.last_action = [0.0, 0.0, 0.0, self.hover_thrust]
        # 最近一次有效动作到达的 ROS 时钟纳秒；None 表示从未收到。
        self.last_action_ns = None
        self.tick_count = 0

        self.obs_pub = self.create_publisher(
            DroneObservation,
            '/haique/observation',
            10
        )

        # 订阅策略动作，用于回填 last_action，使观测中的"上一步动作"真实可用。
        self.action_sub = self.create_subscription(
            PolicyAction,
            '/haique/policy_action',
            self.action_callback,
            10
        )

        # 波形相位按 tick 计数推进，不用 ROS 时钟：
        # use_sim_time 切换、仿真重置或 rosbag 回放会让时钟跳变甚至回退，
        # 导致正弦相位跳变或为负。header.stamp 仍取 ROS 时钟，二者职责分离。
        self.timer = self.create_timer(1.0 / self.freq_hz, self.timer_callback)

        # 打印实际生效的参数值，便于排查"参数没生效"。
        self.get_logger().info(
            f'[MockTelemetryPublisher] 已启动: frequency_hz={self.freq_hz:g}, '
            f'hover_thrust={self.hover_thrust:g}, 主题=/haique/observation'
        )

    @staticmethod
    def _validate_parameters(freq_hz: float, hover_thrust: float) -> None:
        # 参数非法直接抛 ValueError 退出；静默修正会让配置错误难以定位。
        if not math.isfinite(freq_hz) or not (1.0 <= freq_hz <= 1000.0):
            raise ValueError(f'frequency_hz 必须位于 [1, 1000]，当前为 {freq_hz!r}')
        if not math.isfinite(hover_thrust) or not (0.0 < hover_thrust <= 1.0):
            raise ValueError(f'hover_thrust 必须位于 (0, 1]，当前为 {hover_thrust!r}')

    def action_callback(self, msg: PolicyAction):
        # 回调保持轻量：只做非有限值校验与整表赋值，发布仍由 frequency_hz 定时器负责。
        action = [
            float(msg.desired_body_rates.x),
            float(msg.desired_body_rates.y),
            float(msg.desired_body_rates.z),
            float(msg.collective_thrust),
        ]

        if not all(math.isfinite(value) for value in action):
            # 非有限动作会污染下一步观测输入，整帧丢弃并保留上一帧有效值。
            self.get_logger().warn(
                f'收到含非有限值的策略动作，已丢弃并保留上一帧: {action}',
                throttle_duration_sec=2.0
            )
            return

        self.last_action = action
        self.last_action_ns = self.get_clock().now().nanoseconds

    def timer_callback(self):
        now = self.get_clock().now()
        # tick 先自增再算相位：t 只随发布帧数单调推进，与时钟跳变解耦。
        self.tick_count += 1
        t = self.tick_count / self.freq_hz

        # 动作陈旧性检查：策略节点停摆时 last_action 会永久冻结，
        # 长时测试会误以为控制链路仍在工作。这里只告警，不归零也不回落悬停：
        # 真机 PX4 会保持最后 setpoint，伪造一次零动作阶跃比保持更失真。
        if self.last_action_ns is None:
            self.get_logger().warn(
                '尚未收到 /haique/policy_action，last_action 持续回显初值 '
                f'[0.0, 0.0, 0.0, {self.hover_thrust:g}]',
                throttle_duration_sec=2.0
            )
        else:
            age_s = (now.nanoseconds - self.last_action_ns) / 1e9
            stale_threshold_s = 3.0 / self.freq_hz
            if age_s > stale_threshold_s:
                self.get_logger().warn(
                    f'策略动作已陈旧 {age_s:.2f}s (阈值 {stale_threshold_s:.3f}s)，'
                    'last_action 保持最后一次有效值，不归零也不回落悬停',
                    throttle_duration_sec=2.0
                )

        msg = DroneObservation()
        msg.header.stamp = now.to_msg()
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

        # ZYX(yaw-pitch-roll) 精确转换，R = Rz·Ry·Rx
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
        # 回调整表替换 self.last_action、默认单线程执行，故此处拿到的一定是某一整帧；
        # 任何一处改成原地修改都会破坏该前提。
        msg.last_action = self.last_action

        # 6. 不再拼装 17 维扁平向量：顺序唯一拼装点在 policy_runner_node。

        self.obs_pub.publish(msg)

        # 每 2 秒最多打印一次状态，避免与发布频率耦合。
        self.get_logger().info(
            f'[MockTelemetry] 累计发送 {self.tick_count} 帧 | '
            f'目标距: ({rel_x:.2f}, {rel_y:.2f}, {rel_z:.2f})m',
            throttle_duration_sec=2.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MockTelemetryPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (ValueError, RuntimeError) as exc:
        print(f'[mock_telemetry_pub] 启动失败: {exc}', file=sys.stderr)
        raise SystemExit(1)
    finally:
        if node is not None:
            node.destroy_node()
        # 信号处理器可能已关闭上下文，重复 shutdown 会抛 RCLError。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
