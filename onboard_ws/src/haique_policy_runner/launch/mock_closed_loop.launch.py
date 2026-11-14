#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
海雀机载系统 - 零硬件依赖闭环联合启动文件 (Mock Closed-Loop Launch)
启动命令:
  ros2 launch haique_policy_runner mock_closed_loop.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mock_telemetry_node = Node(
        package='haique_policy_runner',
        executable='mock_telemetry_pub',
        name='mock_telemetry_publisher',
        output='screen',
        parameters=[{'frequency_hz': 50.0}]
    )

    policy_runner_node = Node(
        package='haique_policy_runner',
        executable='policy_runner_node',
        name='policy_runner_node',
        output='screen',
        parameters=[{
            'max_tilt_rate': 3.0,
            'min_thrust': 0.15,
            'max_thrust': 0.85,
            'watchdog_timeout_s': 0.2
        }]
    )

    px4_bridge_node = Node(
        package='haique_policy_runner',
        executable='px4_rates_bridge',
        name='px4_rates_bridge',
        output='screen'
    )

    return LaunchDescription([
        mock_telemetry_node,
        policy_runner_node,
        px4_bridge_node
    ])
