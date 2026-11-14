#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
海雀机载系统 - PX4 角速度控制适配与坐标转换桥接节点 (PX4 Rates Bridge)
核心功能:
  1. 订阅策略动作 /haique/policy_action (FLU 坐标系: 前-左-上);
  2. 执行无人机经典坐标系转换: FLU -> FRD (前-右-下, PX4 原生机体系标准);
     数学对应关系:
       omega_x (FRD) =  omega_x (FLU)   [滚转保持一致]
       omega_y (FRD) = -omega_y (FLU)   [俯仰轴取反: FLU左手向上为正，FRD右后为正]
       omega_z (FRD) = -omega_z (FLU)   [偏航轴取反: FLU逆时针朝天为正，FRD顺时针朝地为正]
       thrust (FRD)  = [0, 0, -collective_thrust] (PX4 Z轴朝下，推力向上为负)
  3. 当前仅做 FLU->FRD 转换，结果无消费者，不发布任何 setpoint：
     真正接通 /fmu/in/vehicle_rates_setpoint 需要引入 px4_msgs，本节点尚未实现。
"""

import math
import sys

import rclpy
from rclpy.node import Node
from haique_interfaces.msg import PolicyAction


class PX4RatesBridge(Node):
    def __init__(self):
        super().__init__('px4_rates_bridge')

        # 未实现告警只打一次，避免 50Hz 回调把控制台刷满。
        self._warned_unimplemented = False

        # 订阅策略动作
        self.action_sub = self.create_subscription(
            PolicyAction,
            '/haique/policy_action',
            self.action_callback,
            10
        )

        self.get_logger().info("[PX4RatesBridge] 坐标转换与飞控适配节点已启动 (FLU -> FRD)")

    def action_callback(self, action_msg: PolicyAction):
        flu_roll = action_msg.desired_body_rates.x
        flu_pitch = action_msg.desired_body_rates.y
        flu_yaw = action_msg.desired_body_rates.z
        flu_thrust = action_msg.collective_thrust

        # 非有限值无法安全转换，整帧丢弃；限流告警避免故障时刷屏。
        if not all(math.isfinite(value) for value in (flu_roll, flu_pitch, flu_yaw, flu_thrust)):
            self.get_logger().warn(
                '收到含非有限值的策略动作，已丢弃该帧，不参与坐标转换',
                throttle_duration_sec=2.0
            )
            return

        # 坐标系转换核心公式 (FLU -> FRD)
        frd_roll_rate = flu_roll
        frd_pitch_rate = -flu_pitch
        frd_yaw_rate = -flu_yaw
        frd_thrust_z = -flu_thrust

        # 转换结果目前没有消费者，用限流 DEBUG 日志消费并暴露四个值，便于诊断。
        # 限流日志要求同一调用点的 kwargs 恒定，故四个值只放进消息体。
        self.get_logger().debug(
            f'FLU->FRD: roll={frd_roll_rate:.4f}, pitch={frd_pitch_rate:.4f}, '
            f'yaw={frd_yaw_rate:.4f}, thrust_z={frd_thrust_z:.4f}',
            throttle_duration_sec=1.0
        )

        # 转换结果目前没有任何消费者，必须在首帧明确告知，否则易被误判为链路已通。
        if not self._warned_unimplemented:
            self._warned_unimplemented = True
            self.get_logger().warn(
                'PX4 输出尚未实现：FLU->FRD 转换结果被丢弃，不会发送任何 setpoint。'
                '接通 /fmu/in/vehicle_rates_setpoint 需要引入 px4_msgs (OffboardControlMode 与 '
                'VehicleRatesSetpoint)，当前链路未连通。'
            )

        # TODO: 接通 PX4 Micro-XRCE-DDS 时:
        # 1. 引入 px4_msgs.msg 中的 VehicleRatesSetpoint 与 OffboardControlMode;
        # 2. 以 50Hz 发布 offboard_control_mode (body_rate = True);
        # 3. 将 frd_roll_rate、frd_pitch_rate、frd_yaw_rate 及 frd_thrust_z
        #    填入 VehicleRatesSetpoint 并发布至 /fmu/in/vehicle_rates_setpoint。


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PX4RatesBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (ValueError, RuntimeError) as exc:
        print(f'[px4_rates_bridge] 启动失败: {exc}', file=sys.stderr)
        raise SystemExit(1)
    finally:
        if node is not None:
            node.destroy_node()
        # 信号处理器可能已关闭上下文，重复 shutdown 会抛 RCLError。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
