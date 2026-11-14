#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
海雀机载系统 - PX4 角速度控制适配与坐标转换桥接节点 (PX4 Rates Bridge)
核心功能与教学意义:
  1. 订阅策略动作 /haique/policy_action (FLU 坐标系: 前-左-上);
  2. 执行无人机经典坐标系转换: FLU -> FRD (前-右-下, PX4 原生机体系标准);
     数学对应关系:
       omega_x (FRD) =  omega_x (FLU)   [滚转保持一致]
       omega_y (FRD) = -omega_y (FLU)   [俯仰轴取反: FLU左手向上为正，FRD右后为正]
       omega_z (FRD) = -omega_z (FLU)   [偏航轴取反: FLU逆时针朝天为正，FRD顺时针朝地为正]
       thrust (FRD)  = [0, 0, -collective_thrust] (PX4 Z轴朝下，推力向上为负)
  3. 为组员提供标准的 PX4 Offboard 模式心跳维持与 Setpoint 发布模板。
"""

import rclpy
from rclpy.node import Node
from haique_interfaces.msg import PolicyAction


class PX4RatesBridge(Node):
    def __init__(self):
        super().__init__('px4_rates_bridge')

        # 订阅策略动作
        self.action_sub = self.create_subscription(
            PolicyAction,
            '/haique/policy_action',
            self.action_callback,
            10
        )

        self.get_logger().info("✅ [PX4RatesBridge] 坐标转换与飞控适配节点已启动 (FLU <-> FRD)")

    def action_callback(self, action_msg: PolicyAction):
        flu_roll = action_msg.desired_body_rates.x
        flu_pitch = action_msg.desired_body_rates.y
        flu_yaw = action_msg.desired_body_rates.z
        flu_thrust = action_msg.collective_thrust

        # =========================================================================
        # 📐 坐标系转换核心公式 (FLU -> FRD)
        # =========================================================================
        frd_roll_rate = flu_roll
        frd_pitch_rate = -flu_pitch
        frd_yaw_rate = -flu_yaw
        frd_thrust_z = -flu_thrust

        # =========================================================================
        # 💡 [组员开发提示 TODO]:
        # 接通 PX4 Micro-XRCE-DDS 时:
        # 1. 引入 px4_msgs.msg 中的 VehicleRatesSetpoint 与 OffboardControlMode
        # 2. 以 50Hz 发布 offboard_control_mode (body_rate = True)
        # 3. 将转换后的 frd_roll_rate, frd_pitch_rate, frd_yaw_rate 及 frd_thrust_z
        #    填充入 VehicleRatesSetpoint 并发布至 /fmu/in/vehicle_rates_setpoint
        # =========================================================================


def main(args=None):
    rclpy.init(args=args)
    node = PX4RatesBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
