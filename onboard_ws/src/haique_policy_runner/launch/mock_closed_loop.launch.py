#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
海雀机载系统 - 零硬件依赖闭环联合启动文件 (Mock Closed-Loop Launch)
启动命令:
  ros2 launch haique_policy_runner mock_closed_loop.launch.py
启用 ONNX:
  ros2 launch haique_policy_runner mock_closed_loop.launch.py \
    model_path:=/abs/path/policy.onnx allow_fallback:=false

policy_runner_node 退出时整栈一起退出: 它是控制链的动作来源，
只打一行 "process has died" 会让 mock 继续 50Hz 打印，看起来一切正常。
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 两个节点必须使用同一个悬停推力: mock 用它作为 last_action 初值，
# policy_runner 用它作为兜底动作与内置控制器的基准。
# 两者不一致会引入稳态偏差。
# 0.58 没有实测依据，只用于 mock/台架; 真机前必须按 tau_hover = m*g / T_max
# 标定并显式传入，当前硬件约为 0.13。若标定值低于 min_thrust，
# policy_runner 启动时会拒绝运行，需要同步下调 min_thrust。
HOVER_THRUST = 0.58


def generate_launch_description():
    declare_model_path = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='ONNX 策略模型路径；留空则使用内置基准控制器'
    )
    declare_allow_fallback = DeclareLaunchArgument(
        'allow_fallback',
        default_value='false',
        description='模型加载失败时是否允许降级为内置基准控制器'
    )

    mock_telemetry_node = Node(
        package='haique_policy_runner',
        executable='mock_telemetry_pub',
        name='mock_telemetry_publisher',
        output='screen',
        parameters=[{
            'frequency_hz': 50.0,
            'hover_thrust': HOVER_THRUST,
        }]
    )

    policy_runner_node = Node(
        package='haique_policy_runner',
        executable='policy_runner_node',
        name='policy_runner_node',
        output='screen',
        parameters=[{
            # LaunchConfiguration 给到的是替换结果，字符串由节点侧解析。
            'model_path': LaunchConfiguration('model_path'),
            # 布尔参数用 ParameterValue 声明类型，避免 'false' 当成真值。
            'allow_fallback': ParameterValue(
                LaunchConfiguration('allow_fallback'), value_type=bool),
            'hover_thrust': HOVER_THRUST,
            'max_tilt_rate': 3.0,
            'min_thrust': 0.15,
            'max_thrust': 0.85,
            'watchdog_timeout_s': 0.2,
        }]
    )

    px4_bridge_node = Node(
        package='haique_policy_runner',
        executable='px4_rates_bridge',
        name='px4_rates_bridge',
        output='screen'
    )

    # policy 是控制链核心: 退出即代表闭环失效，必须让整栈一起退出。
    shutdown_on_policy_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=policy_runner_node,
            on_exit=[
                EmitEvent(event=Shutdown(
                    reason='policy_runner_node 已退出，控制链失效'))
            ]
        )
    )

    # bridge 目前只是 stub，退出不致命，仅提示 PX4 输出链路断开。
    warn_on_bridge_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=px4_bridge_node,
            on_exit=[
                LogInfo(msg='[WARN] px4_rates_bridge 已退出，PX4 输出链路断开')
            ]
        )
    )

    return LaunchDescription([
        declare_model_path,
        declare_allow_fallback,
        mock_telemetry_node,
        policy_runner_node,
        px4_bridge_node,
        shutdown_on_policy_exit,
        warn_on_bridge_exit,
    ])
