#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
海雀机载系统 - 强化学习策略调度节点 (Policy Runner Node)
核心功能:
  1. 订阅 50Hz 观测向量 /haique/observation (17 维);
  2. 调用 ONNX Runtime (或轻量内置策略) 进行前向推理;
  3. 诊断单步推理时延 (ms);
  4. 实施推力限幅与机动保护检查;
  5. 发布 4 维动作 /haique/policy_action 与健康状态 /haique/safety_status;
  6. 观测丢包看门狗 (Watchdog): 超过 200ms 无新数据自动切入急停/降级保护。
"""

import time
import math
import rclpy
from rclpy.node import Node
from haique_interfaces.msg import DroneObservation, PolicyAction, SafetyStatus
from geometry_msgs.msg import Vector3

# 尝试导入 onnxruntime，未安装时平滑降级为内置基准策略
try:
    import numpy as np
    import onnxruntime as ort
    HAS_ONNX = True
except ImportError:
    HAS_ONNX = False


class PolicyRunnerNode(Node):
    def __init__(self):
        super().__init__('policy_runner_node')

        # 声明 ROS 2 参数
        self.declare_parameter('model_path', '')
        self.declare_parameter('max_tilt_rate', 3.0)       # 最大机体角速度保护 (rad/s)
        self.declare_parameter('min_thrust', 0.15)         # 最小推力防止空中失速
        self.declare_parameter('max_thrust', 0.85)         # 最大推力保护电调与电池
        self.declare_parameter('watchdog_timeout_s', 0.2)   # 观测超时时间 (0.2s = 10 帧丢包)

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.max_tilt_rate = self.get_parameter('max_tilt_rate').get_parameter_value().double_value
        self.min_thrust = self.get_parameter('min_thrust').get_parameter_value().double_value
        self.max_thrust = self.get_parameter('max_thrust').get_parameter_value().double_value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout_s').get_parameter_value().double_value

        # 初始化 ONNX 推理会话 (如配置了模型)
        self.ort_session = None
        self._init_policy_engine()

        # 订阅与发布
        self.obs_sub = self.create_subscription(
            DroneObservation,
            '/haique/observation',
            self.observation_callback,
            10
        )

        self.action_pub = self.create_publisher(
            PolicyAction,
            '/haique/policy_action',
            10
        )

        self.safety_pub = self.create_publisher(
            SafetyStatus,
            '/haique/safety_status',
            10
        )

        # 内部状态追踪与看门狗
        self.last_obs_time = time.time()
        self.step_counter = 0
        self.total_latency_ms = 0.0
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_check)  # 20Hz 看门狗

        self.get_logger().info("🚀 [PolicyRunnerNode] 策略调度节点已就绪，等待 /haique/observation 输入...")

    def _init_policy_engine(self):
        """初始化神经网络推理引擎"""
        if self.model_path and HAS_ONNX:
            try:
                self.get_logger().info(f"正在载入 ONNX 模型: {self.model_path}")
                self.ort_session = ort.InferenceSession(
                    self.model_path,
                    providers=['CPUExecutionProvider']
                )
                self.input_name = self.ort_session.get_inputs()[0].name
                self.get_logger().info("✅ ONNX 模型加载成功！")
            except Exception as e:
                self.get_logger().error(f"❌ 加载 ONNX 模型失败: {e}，将启用内置安全基准策略。")
                self.ort_session = None
        else:
            if not HAS_ONNX:
                self.get_logger().warn("⚠️ 当前运行环境未安装 onnxruntime，使用内置轻量基准策略运行验证。")
            else:
                self.get_logger().info("ℹ️ 未指定 model_path，当前使用零外部依赖内置基准策略。")

    def observation_callback(self, obs_msg: DroneObservation):
        """核心 50Hz 策略推理回调流水线"""
        t_start = time.perf_counter()
        self.last_obs_time = time.time()
        self.step_counter += 1

        flat_obs = list(obs_msg.flat_obs)

        # 1. 神经网络前向推理
        # =========================================================================
        # 💡 [组员开发提示 TODO]:
        # 当训练出新的 RL policy.onnx 时，通过 launch 文件传递 model_path 即可无缝替换
        # 输入维度: [1, 17], 输出维度: [1, 4]
        # =========================================================================
        if self.ort_session is not None and HAS_ONNX:
            inp = np.array([flat_obs], dtype=np.float32)
            out = self.ort_session.run(None, {self.input_name: inp})[0][0]
            raw_action = [float(out[0]), float(out[1]), float(out[2]), float(out[3])]
        else:
            # 内置基准反馈控制 (PD 阻尼姿态，目标相对位置作为假输入模拟平稳悬停)
            # 保证无模型时也能平稳运转
            rel_x, rel_y, rel_z = flat_obs[0], flat_obs[1], flat_obs[2]
            roll_rate = -0.5 * rel_y
            pitch_rate = 0.5 * rel_x
            yaw_rate = 0.0
            thrust = 0.58 + 0.1 * rel_z
            raw_action = [roll_rate, pitch_rate, yaw_rate, thrust]

        # 2. 安全保护与饱和限幅 (Safety Guard Clamping)
        roll_rate = max(-self.max_tilt_rate, min(self.max_tilt_rate, raw_action[0]))
        pitch_rate = max(-self.max_tilt_rate, min(self.max_tilt_rate, raw_action[1]))
        yaw_rate = max(-self.max_tilt_rate, min(self.max_tilt_rate, raw_action[2]))
        thrust = max(self.min_thrust, min(self.max_thrust, raw_action[3]))

        # 3. 计算推理耗时
        t_elapsed_ms = (time.perf_counter() - t_start) * 1000.0
        self.total_latency_ms += t_elapsed_ms

        # 4. 发布策略动作消息
        action_msg = PolicyAction()
        action_msg.header.stamp = self.get_clock().now().to_msg()
        action_msg.header.frame_id = 'base_link_flu'
        action_msg.desired_body_rates = Vector3(x=roll_rate, y=pitch_rate, z=yaw_rate)
        action_msg.collective_thrust = float(thrust)
        action_msg.raw_action = raw_action
        action_msg.inference_latency_ms = float(t_elapsed_ms)

        self.action_pub.publish(action_msg)

        # 5. 周期性统计日志 (每 100 步统计一次平均延迟)
        if self.step_counter % 100 == 0:
            avg_lat = self.total_latency_ms / 100.0
            self.total_latency_ms = 0.0
            self.get_logger().info(
                f"[PolicyRunner] 步数: {self.step_counter} | "
                f"推理延迟: {t_elapsed_ms:.2f}ms (百帧均值: {avg_lat:.2f}ms) | "
                f"动作: 角速度=({roll_rate:.2f}, {pitch_rate:.2f}, {yaw_rate:.2f}) 推力={thrust:.2f}"
            )

    def watchdog_check(self):
        """观测丢包看门狗监控"""
        dt = time.time() - self.last_obs_time
        safety_msg = SafetyStatus()
        safety_msg.header.stamp = self.get_clock().now().to_msg()

        if dt > self.watchdog_timeout:
            safety_msg.is_safe = False
            safety_msg.offboard_enabled = False
            safety_msg.emergency_triggered = True
            safety_msg.status_code = "OBS_TIMEOUT"
            safety_msg.message = f"观测数据超时中断! 已离线 {dt:.3f}s > {self.watchdog_timeout}s"
            self.safety_pub.publish(safety_msg)
            self.get_logger().warn(
                f"🚨 [WATCHDOG] 触发急停报警: {safety_msg.message}", 
                throttle_duration_sec=2.0
            )
        else:
            safety_msg.is_safe = True
            safety_msg.offboard_enabled = True
            safety_msg.emergency_triggered = False
            safety_msg.status_code = "NORMAL"
            safety_msg.message = "策略执行流水线正常"
            self.safety_pub.publish(safety_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PolicyRunnerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
