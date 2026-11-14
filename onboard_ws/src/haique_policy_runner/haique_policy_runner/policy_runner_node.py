#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
海雀机载系统 - 强化学习策略调度节点 (Policy Runner Node)
核心功能:
  1. 订阅 50Hz 观测 /haique/observation，在唯一入口拼装 17 维策略输入;
  2. 调用 ONNX Runtime 或内置基准控制器前向推理;
  3. 观测与推理结果做非有限值防护，动作做角速度与推力限幅;
  4. 发布 4 维动作 /haique/policy_action 与健康状态 /haique/safety_status;
  5. 观测丢包看门狗: 超过 watchdog_timeout_s 无新观测即报告 OBS_TIMEOUT;
  6. OBS_TIMEOUT 期间以 20Hz 主动下发悬停安全动作，看门狗真正执行动作，
     而不只是播报；仅在曾经收到过观测后生效，上电阶段不下发。

状态分两层:
  引擎模式 _engine_mode 说明当前用的是 ONNX 还是内置基准控制器;
  流水线状态 _status_code / _status_message / _is_safe 说明流水线健康度。
安全状态只有一个发布点: 20Hz 的 watchdog_check。
"""

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from haique_interfaces.msg import DroneObservation, PolicyAction, SafetyStatus

# onnxruntime 只在指定了 model_path 时才必需，默认路径下节点零第三方依赖。
try:
    import numpy as np
    import onnxruntime as ort
    HAS_ONNX = True
except ImportError:
    np = None
    ort = None
    HAS_ONNX = False


OBS_DIM = 17                    # 冻结的观测维度，顺序见 DroneObservation.msg
ACTION_DIM = 4                  # [roll_rate, pitch_rate, yaw_rate, thrust]
MAX_E2E_LATENCY_MS = 1000.0     # 端到端延迟上界，超出视为时间戳不可信

# 引擎模式: 决定"当前到底在跑网络还是内置基准控制器"。
ENGINE_ONNX = 'ONNX'
ENGINE_BASELINE_NO_MODEL = 'BASELINE_NO_MODEL'
ENGINE_BASELINE_FALLBACK = 'BASELINE_FALLBACK'

STATUS_WAITING = 'WAITING_OBSERVATION'
STATUS_NORMAL = 'NORMAL'
STATUS_BASELINE_FALLBACK = 'BASELINE_FALLBACK'
STATUS_NONFINITE_OBSERVATION = 'NONFINITE_OBSERVATION'
STATUS_NONFINITE_ACTION = 'NONFINITE_ACTION'
STATUS_OBS_TIMEOUT = 'OBS_TIMEOUT'
STATUS_PIPELINE_EXCEPTION = 'PIPELINE_EXCEPTION'

# 只有真故障才置 emergency_triggered。
# 非有限输入与降级模式已安全兜底，不算紧急。
EMERGENCY_STATUS_CODES = frozenset({STATUS_OBS_TIMEOUT, STATUS_PIPELINE_EXCEPTION})


class NonFiniteObservationError(ValueError):
    """观测在 float32 转换后出现非有限值，例如 1e300 溢出成 inf。"""


class PolicyRunnerNode(Node):
    """策略调度节点。

    看门狗超时后不再只播报: watchdog_check 会以 20Hz 下发悬停安全动作，
    但仅在曾经收到过观测后生效，上电阶段不向飞控灌 setpoint。
    """

    def __init__(self):
        super().__init__('policy_runner_node')

        # 参数先声明、读取、校验，全部通过后才创建订阅与定时器。
        # 非法参数在启动阶段就抛 ValueError，不留下半启动状态。
        self.declare_parameter('model_path', '')
        self.declare_parameter('allow_fallback', False)
        # hover_thrust 的 0.58 没有实测依据，仅保持既有行为不变。
        # 真机前必须按 tau_hover = m*g / T_max 重新标定，当前硬件约为 0.13。
        # 若标定值低于 min_thrust，兜底推力会被夹到 min_thrust，
        # 需同步下调 min_thrust。
        self.declare_parameter('hover_thrust', 0.58)
        self.declare_parameter('max_tilt_rate', 3.0)
        self.declare_parameter('min_thrust', 0.15)
        self.declare_parameter('max_thrust', 0.85)
        self.declare_parameter('watchdog_timeout_s', 0.2)

        self.model_path = str(self.get_parameter('model_path').value)
        # launch 侧可能以字符串传布尔值，必须显式解析，不能用 bool('false')。
        self.allow_fallback = self._parse_bool(
            self.get_parameter('allow_fallback').value, 'allow_fallback')
        self.hover_thrust = float(self.get_parameter('hover_thrust').value)
        self.max_tilt_rate = float(self.get_parameter('max_tilt_rate').value)
        self.min_thrust = float(self.get_parameter('min_thrust').value)
        self.max_thrust = float(self.get_parameter('max_thrust').value)
        self.watchdog_timeout = float(self.get_parameter('watchdog_timeout_s').value)
        self._validate_parameters()

        # 推理引擎句柄，None 表示使用内置基准控制器。
        self.ort_session = None
        self.input_name = None
        self.output_name = None
        self._engine_mode = ENGINE_BASELINE_NO_MODEL
        self._init_policy_engine()

        # 流水线状态是安全状态的唯一来源，由 observation_callback 更新。
        # 首帧观测到达前不允许报安全，也不能按超时故障处理。
        self._status_code = STATUS_WAITING
        self._status_message = '启动后尚未收到任何观测'
        self._is_safe = False

        # 绝对时间一律用整数纳秒: rclpy 的 Time 相减在负值时抛 ValueError。
        # None 表示尚未收到任何观测，对应 WAITING_OBSERVATION。
        self._last_obs_time_ns = None
        # 超时判据用观测时间戳年龄: 队列积压时回调"刚到"但数据已陈旧。
        self._last_obs_stamp_ns = None
        # 只有收到过观测后才允许看门狗下发动作，避免上电即灌 setpoint。
        self._has_received_obs = False
        self._watchdog_timeout_ns = int(self.watchdog_timeout * 1e9)

        self._step_counter = 0
        self._stat_count = 0
        self._latency_sum_ms = 0.0
        self._e2e_sum_ms = 0.0
        self._e2e_count = 0
        self._last_published_action = [0.0, 0.0, 0.0, self.hover_thrust]

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

        self.watchdog_timer = self.create_timer(0.05, self.watchdog_check)  # 20Hz

        self.get_logger().info(
            f'[PolicyRunnerNode] 就绪 engine={self._engine_mode} '
            f'角速度限幅={self.max_tilt_rate}rad/s '
            f'推力=[{self.min_thrust}, {self.max_thrust}] '
            f'看门狗={self.watchdog_timeout}s'
        )

    @staticmethod
    def _parse_bool(value, parameter_name):
        """解析布尔参数。

        launch 用 LaunchConfiguration 传参时值是字符串，而 bool('false') 为 True，
        所以必须显式解析；无法识别的取值直接报错，避免误判降级开关。
        """
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            text = value.strip().lower()
            if text in ('1', 'true', 'yes', 'on'):
                return True
            if text in ('0', 'false', 'no', 'off', ''):
                return False
        raise ValueError(f'{parameter_name} 必须是布尔值，当前为 {value!r}')

    def _validate_parameters(self):
        """校验冻结参数清单，非法即抛 ValueError 退出。"""
        # 静默修正会让配置错误在飞行中难以定位，因此这里一律 fail-fast。
        if self.model_path and not os.path.isfile(self.model_path):
            raise ValueError(
                f'model_path 必须为空或存在的文件，当前为 {self.model_path!r}'
            )
        if not math.isfinite(self.hover_thrust) or not (0.0 < self.hover_thrust <= 1.0):
            raise ValueError(f'hover_thrust 必须位于 (0, 1]，当前为 {self.hover_thrust!r}')
        if not math.isfinite(self.max_tilt_rate) or not (0.0 < self.max_tilt_rate <= 20.0):
            raise ValueError(
                f'max_tilt_rate 必须位于 (0, 20]，当前为 {self.max_tilt_rate!r}')

        if not math.isfinite(self.min_thrust) or not (0.0 <= self.min_thrust < 1.0):
            raise ValueError(f'min_thrust 必须位于 [0, 1)，当前为 {self.min_thrust!r}')
        if not math.isfinite(self.max_thrust) or not (0.0 < self.max_thrust <= 1.0):
            raise ValueError(f'max_thrust 必须位于 (0, 1]，当前为 {self.max_thrust!r}')
        if self.min_thrust >= self.max_thrust:
            raise ValueError(
                f'min_thrust 必须小于 max_thrust，'
                f'当前为 {self.min_thrust} 与 {self.max_thrust}'
            )
        if not math.isfinite(self.watchdog_timeout) or not (0.0 < self.watchdog_timeout <= 5.0):
            raise ValueError(
                f'watchdog_timeout_s 必须位于 (0, 5]，当前为 {self.watchdog_timeout!r}'
            )

    def _init_policy_engine(self):
        """初始化推理引擎，按契约严格处理加载失败。"""
        if self.model_path:
            # 指定了模型却缺少运行库属于部署错误，不能静默降级。
            if not HAS_ONNX:
                raise RuntimeError(
                    f'已指定 model_path={self.model_path!r}，但未安装 numpy/onnxruntime，'
                    '无法加载模型'
                )
            self.get_logger().info(f'正在加载 ONNX 模型: {self.model_path}')
            try:
                self.ort_session = ort.InferenceSession(
                    self.model_path,
                    providers=['CPUExecutionProvider']
                )
            except Exception as exc:
                if not self.allow_fallback:
                    raise RuntimeError(
                        f'加载 ONNX 模型失败且 allow_fallback=False: {exc}'
                    ) from exc
                self.get_logger().error(
                    f'加载 ONNX 模型失败: {exc}，allow_fallback=True，'
                    '降级为内置基准控制器。'
                )
                self.ort_session = None
                self._engine_mode = ENGINE_BASELINE_FALLBACK
                return
            self._validate_onnx_io()
            self._engine_mode = ENGINE_ONNX
            return

        # 未指定模型属于开发模式，保持可运行。
        self._engine_mode = ENGINE_BASELINE_NO_MODEL
        if not HAS_ONNX:
            self.get_logger().warn(
                '当前环境未安装 numpy/onnxruntime，使用内置基准控制器运行。'
            )
        else:
            self.get_logger().info('未指定 model_path，使用内置基准控制器运行。')

    @staticmethod
    def _last_dim(shape):
        """取 ONNX shape 的最后一维，动态维(字符串/None)视为无法校验。"""
        if not shape:
            return None
        last = shape[-1]
        return last if isinstance(last, int) else None

    def _validate_onnx_io(self):
        """校验模型输入输出契约，避免形状不符的模型输出废动作。"""
        inputs = self.ort_session.get_inputs()
        outputs = self.ort_session.get_outputs()
        if len(inputs) != 1:
            raise RuntimeError(f'ONNX 模型必须只有 1 个输入，实际 {len(inputs)} 个')
        if not outputs:
            raise RuntimeError('ONNX 模型缺少输出定义')

        input_meta, output_meta = inputs[0], outputs[0]
        input_shape = list(input_meta.shape)
        output_shape = list(output_meta.shape)

        if len(input_shape) not in (1, 2):
            raise RuntimeError(
                f'ONNX 输入 rank 必须为 1 或 2，期望 [batch, {OBS_DIM}]，'
                f'实际 {input_shape}'
            )
        if self._last_dim(input_shape) != OBS_DIM:
            raise RuntimeError(
                f'ONNX 输入形状必须为 [batch, {OBS_DIM}]，实际 {input_shape}'
            )
        if len(output_shape) not in (1, 2):
            raise RuntimeError(
                f'ONNX 输出 rank 必须为 1 或 2，期望 [batch, {ACTION_DIM}]，'
                f'实际 {output_shape}'
            )
        if self._last_dim(output_shape) != ACTION_DIM:
            raise RuntimeError(
                f'ONNX 输出形状必须为 [batch, {ACTION_DIM}]，实际 {output_shape}'
            )
        input_type = getattr(input_meta, 'type', '未知')
        if input_type != 'tensor(float)':
            raise RuntimeError(
                f'ONNX 输入 dtype 必须为 float32 (tensor(float))，实际为 {input_type}'
            )

        self.input_name = input_meta.name
        self.output_name = output_meta.name
        self.get_logger().info(
            f'ONNX 模型加载成功: 输入 {self.input_name}{input_shape} ({input_type}) '
            f'-> 输出 {self.output_name}{output_shape}'
        )

    def _build_observation(self, obs_msg: DroneObservation):
        """17 维观测向量的唯一拼装点，返回 tuple[float, ...]。

        冻结顺序: [quat(w,x,y,z), lin_vel(3), ang_vel(3), rel_pos(3), last_action(4)]。
        四元数必须显式按字段名取 w 在前。
        geometry_msgs/Quaternion 的字段顺序是 x, y, z, w，
        按消息下标取会把实部顺序搞错。
        """
        q = obs_msg.orientation
        lin = obs_msg.body_linear_velocity
        ang = obs_msg.body_angular_velocity
        rel = obs_msg.rel_position_error
        last = list(obs_msg.last_action)

        observation = (
            float(q.w), float(q.x), float(q.y), float(q.z),
            float(lin.x), float(lin.y), float(lin.z),
            float(ang.x), float(ang.y), float(ang.z),
            float(rel.x), float(rel.y), float(rel.z),
            float(last[0]), float(last[1]), float(last[2]), float(last[3]),
        )
        # 长度不符说明消息字段被改过，直接报错，不让模型静默吃错输入。
        if len(observation) != OBS_DIM:
            raise ValueError(f'观测向量长度必须为 {OBS_DIM}，实际 {len(observation)}')
        return observation

    def _infer(self, observation):
        """返回长度为 4 的原始动作 [roll_rate, pitch_rate, yaw_rate, thrust]。"""
        if self.ort_session is None:
            # 内置基准控制仅用于验证数据链路，不是可上真机的控制律。
            rel_x, rel_y, rel_z = observation[10], observation[11], observation[12]
            return [-0.5 * rel_y, 0.5 * rel_x, 0.0, self.hover_thrust + 0.1 * rel_z]

        # 仅在真正调用 ONNX 时才引入 numpy 张量，拼装路径保持纯 Python。
        model_input = np.asarray(observation, dtype=np.float32).reshape(1, OBS_DIM)
        # 1e300 转 float32 会静默溢出成 inf，送进网络前必须再查有限性。
        if not np.isfinite(model_input).all():
            raise NonFiniteObservationError(
                '观测值转 float32 后非有限，已跳过推理并输出悬停兜底动作'
            )
        outputs = self.ort_session.run([self.output_name], {self.input_name: model_input})[0]
        flat_output = np.asarray(outputs, dtype=np.float32).reshape(-1)
        if int(flat_output.size) != ACTION_DIM:
            raise ValueError(
                f'ONNX 输出形状必须为 [batch, {ACTION_DIM}]，'
                f'实际 {list(np.shape(outputs))}'
            )
        return [float(flat_output[0]), float(flat_output[1]),
                float(flat_output[2]), float(flat_output[3])]

    @staticmethod
    def _clamp(value, lo, hi):
        """显式限幅，语义为 max(lo, min(hi, value))。

        NaN 会被洗成上限（CPython 行为），非有限防护由调用方负责，
        不能指望限幅函数兜住非有限值。
        """
        return max(lo, min(hi, value))

    def _safe_action(self):
        """悬停兜底动作，推力必须夹进安全包线。

        真机悬停推力可能低于 min_thrust（当前硬件约 0.13，默认下限 0.15），
        不夹紧就会在兜底时发出越包线推力。
        """
        thrust = self._clamp(float(self.hover_thrust), self.min_thrust, self.max_thrust)
        return (0.0, 0.0, 0.0, thrust)

    def _set_pipeline_status(self, status_code, message, is_safe):
        """更新流水线状态，对外发布统一由 watchdog_check 负责。"""
        self._status_code = status_code
        self._status_message = message
        self._is_safe = bool(is_safe)

    def _publish_safe_action(self, obs_msg, t_start, status_code, message):
        """兜底路径唯一入口: 更新状态、限流告警、发布悬停兜底动作。

        obs_msg / t_start 允许为 None: 看门狗超时下发时没有对应观测，
        此时 inference_latency_ms 与 end_to_end_latency_ms 都填 -1.0。
        """
        self._set_pipeline_status(status_code, message, False)
        self.get_logger().warn(message, throttle_duration_sec=2.0)
        safe_action = self._safe_action()
        self._publish_action(obs_msg, t_start, safe_action, safe_action)

    @staticmethod
    def _stamp_to_ns(stamp):
        """把 header.stamp 转成整数纳秒；(0,0) 表示未打戳，返回 None。"""
        try:
            sec = int(stamp.sec)
            nanosec = int(stamp.nanosec)
        except (TypeError, ValueError):
            return None
        if sec == 0 and nanosec == 0:
            return None
        return sec * 1000000000 + nanosec

    def _compute_end_to_end_latency_ms(self, obs_msg: DroneObservation):
        """观测 header.stamp 到 publish 调用前的延迟 (ms)，不可测返回 -1.0。"""
        stamp_ns = self._stamp_to_ns(obs_msg.header.stamp)
        if stamp_ns is None:
            # 时间戳为 0 表示发布方未打戳，无法计算真实延迟。
            return -1.0

        # 整数纳秒相减不会抛异常; rclpy 的 Time - Time 在负值时抛 ValueError。
        now_ns = self.get_clock().now().nanoseconds
        delta_ms = (now_ns - stamp_ns) / 1e6
        if not math.isfinite(delta_ms) or not (0.0 <= delta_ms <= MAX_E2E_LATENCY_MS):
            # 负值或超上界说明时间戳不可信，按不可测处理。
            return -1.0
        return float(delta_ms)

    def _publish_action(self, obs_msg, t_start, action, raw_action):
        """填充并发布动作消息，同时累计延迟统计。

        obs_msg 为 None 表示看门狗超时下发，此时两个延迟字段都填 -1.0。
        """
        if t_start is None:
            inference_ms = -1.0
        else:
            # perf_counter 只用于测回调内部耗时，绝对时间一律使用 ROS 时钟。
            inference_ms = (time.perf_counter() - t_start) * 1000.0

        action_msg = PolicyAction()
        action_msg.header.stamp = self.get_clock().now().to_msg()
        action_msg.header.frame_id = 'base_link_flu'
        action_msg.desired_body_rates = Vector3(
            x=float(action[0]), y=float(action[1]), z=float(action[2])
        )
        action_msg.collective_thrust = float(action[3])
        action_msg.raw_action = [float(value) for value in raw_action]
        action_msg.inference_latency_ms = float(inference_ms)

        # 端到端延迟采样点紧贴 publish，不把构造消息的耗时算进去。
        if obs_msg is None:
            end_to_end_ms = -1.0
        else:
            end_to_end_ms = self._compute_end_to_end_latency_ms(obs_msg)
        action_msg.end_to_end_latency_ms = float(end_to_end_ms)

        if inference_ms >= 0.0:
            self._stat_count += 1
            self._latency_sum_ms += inference_ms
        if end_to_end_ms >= 0.0:
            self._e2e_sum_ms += end_to_end_ms
            self._e2e_count += 1

        self.action_pub.publish(action_msg)
        self._last_published_action = [float(action[0]), float(action[1]),
                                       float(action[2]), float(action[3])]

    def _log_periodic_stats(self):
        """每 100 步打印一次统计，措辞区分推理耗时与端到端延迟。"""
        if self._step_counter % 100 != 0:
            return
        avg_inference_ms = self._latency_sum_ms / max(1, self._stat_count)
        if self._e2e_count > 0:
            e2e_text = f'{self._e2e_sum_ms / self._e2e_count:.2f}ms'
        else:
            e2e_text = '不可用'
        roll, pitch, yaw, thrust = self._last_published_action

        self._latency_sum_ms = 0.0
        self._e2e_sum_ms = 0.0
        self._e2e_count = 0
        self._stat_count = 0

        self.get_logger().info(
            f'[PolicyRunner] 步数: {self._step_counter} | '
            f'engine={self._engine_mode} | '
            f'推理耗时均值: {avg_inference_ms:.2f}ms | '
            f'端到端延迟均值: {e2e_text} | '
            f'动作: 角速度=({roll:.2f}, {pitch:.2f}, {yaw:.2f}) 推力={thrust:.2f} | '
            f'状态: {self._status_code}'
        )

    def observation_callback(self, obs_msg: DroneObservation):
        """核心 50Hz 推理回调流水线。"""
        t_start = time.perf_counter()
        # 看门狗与延迟都用整数纳秒，避免 Time 相减在时钟回跳时抛异常。
        self._last_obs_time_ns = self.get_clock().now().nanoseconds
        # 同时记录观测时间戳，识别队列积压造成的"回调刚到但数据陈旧"。
        self._last_obs_stamp_ns = self._stamp_to_ns(obs_msg.header.stamp)
        self._has_received_obs = True
        self._step_counter += 1

        try:
            observation = self._build_observation(obs_msg)

            # 非有限观测直接跳过推理: 让模型吃到 NaN 只会产生无意义输出。
            if not all(math.isfinite(value) for value in observation):
                self._publish_safe_action(
                    obs_msg, t_start, STATUS_NONFINITE_OBSERVATION,
                    '观测向量含非有限值，已跳过推理并输出悬停兜底动作')
                return

            try:
                raw_action = self._infer(observation)
            except NonFiniteObservationError as exc:
                # float32 转换后溢出成 inf，等价于观测非有限。
                self._publish_safe_action(
                    obs_msg, t_start, STATUS_NONFINITE_OBSERVATION, str(exc))
                return

            # 推理结果非有限同样不能放行，回退到悬停兜底动作。
            if not all(math.isfinite(value) for value in raw_action):
                self._publish_safe_action(
                    obs_msg, t_start, STATUS_NONFINITE_ACTION,
                    '推理输出含非有限值，已回退到悬停兜底动作')
                return

            action = [
                self._clamp(raw_action[0], -self.max_tilt_rate, self.max_tilt_rate),
                self._clamp(raw_action[1], -self.max_tilt_rate, self.max_tilt_rate),
                self._clamp(raw_action[2], -self.max_tilt_rate, self.max_tilt_rate),
                self._clamp(raw_action[3], self.min_thrust, self.max_thrust),
            ]

            self._set_pipeline_status(
                STATUS_NORMAL,
                f'策略推理流水线正常 engine={self._engine_mode}', True)
            self._publish_action(obs_msg, t_start, action, raw_action)

        except Exception as exc:
            # 回调内任何异常都要反映到安全状态，由看门狗统一对外发布。
            try:
                self._publish_safe_action(
                    obs_msg, t_start, STATUS_PIPELINE_EXCEPTION,
                    f'推理流水线异常: {exc}')
            except Exception:
                # 兜底发布失败时不再向外抛，避免异常逃出回调终止执行器。
                self.get_logger().error(
                    '兜底动作发布失败。', throttle_duration_sec=2.0)

        finally:
            self._log_periodic_stats()

    def _resolve_pipeline_status(self):
        """把流水线状态与引擎模式合成对外状态。

        降级模式必须在流水线正常时持续可见，否则真机会误判在跑网络。
        具体故障（非有限/异常）优先展示，恢复 NORMAL 后再显示降级状态。
        """
        code = self._status_code
        message = self._status_message
        is_safe = self._is_safe
        if code == STATUS_NORMAL and self._engine_mode == ENGINE_BASELINE_FALLBACK:
            code = STATUS_BASELINE_FALLBACK
            is_safe = False
            message = (f'引擎处于降级模式 engine={ENGINE_BASELINE_FALLBACK}，'
                       '模型加载失败后使用内置基准控制器，不得上真机')
        return code, message, is_safe

    def _stamp_age_ns(self, now_ns):
        """最近一帧观测 header.stamp 年龄; 未打戳或时间戳在未来返回 None。"""
        stamp_ns = self._last_obs_stamp_ns
        if stamp_ns is None:
            return None
        age_ns = now_ns - stamp_ns
        if age_ns < 0:
            # 时间戳在未来: 大概率来自另一时钟域，跳过该路而不是报超时。
            return None
        return age_ns

    def _observation_age_ns(self, arrival_age_ns, now_ns):
        """到达时刻年龄与观测时间戳年龄取较大者，返回 (年龄, 来源)。

        队列积压时回调"刚到"，但 header.stamp 已经陈旧;
        只看回调执行时刻会把陈旧数据当成正常。
        """
        age_ns = arrival_age_ns
        source = '到达时刻'
        stamp_age_ns = self._stamp_age_ns(now_ns)
        if stamp_age_ns is not None and stamp_age_ns > age_ns:
            age_ns = stamp_age_ns
            source = '观测时间戳'
        return age_ns, source

    def watchdog_check(self):
        """20Hz 看门狗: 安全状态的唯一发布点，超时时也下发兜底动作。"""
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        safety_msg = SafetyStatus()
        safety_msg.header.stamp = now.to_msg()
        # offboard_enabled 的所有权属于 px4_rates_bridge，这里恒为 False。
        safety_msg.offboard_enabled = False

        if self._last_obs_time_ns is None:
            # 尚未收到任何观测: 等待首帧，不是超时故障，也不下发动作。
            code, message, is_safe = (
                STATUS_WAITING, '启动后尚未收到任何观测', False)
        else:
            arrival_age_ns = now_ns - self._last_obs_time_ns
            if arrival_age_ns < 0:
                # 时钟回跳（sim 重启/回放/NTP）: 重置基线，否则会误报超时。
                arrival_age_ns = 0
                self._last_obs_time_ns = now_ns
                self.get_logger().warn(
                    '检测到时钟回跳，已重置观测超时基线。',
                    throttle_duration_sec=2.0
                )

            age_ns, age_source = self._observation_age_ns(arrival_age_ns, now_ns)
            if age_ns > self._watchdog_timeout_ns:
                code = STATUS_OBS_TIMEOUT
                is_safe = False
                message = (f'观测数据超时: {age_source}年龄 {age_ns / 1e9:.3f}s '
                           f'超过阈值 {self.watchdog_timeout}s')
                if self._has_received_obs:
                    # 看门狗执行动作: 超时期间以 20Hz 持续下发悬停安全动作。
                    self._publish_safe_action(
                        None, None, STATUS_OBS_TIMEOUT, message)
            else:
                code, message, is_safe = self._resolve_pipeline_status()

        safety_msg.is_safe = is_safe
        safety_msg.emergency_triggered = code in EMERGENCY_STATUS_CODES
        safety_msg.status_code = code
        safety_msg.message = message
        self.safety_pub.publish(safety_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PolicyRunnerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (ValueError, RuntimeError) as exc:
        # 参数或模型非法: 给出可读信息后退出，不打 traceback。
        print(f'[policy_runner_node] 启动失败: {exc}', file=sys.stderr)
        raise SystemExit(1)
    finally:
        if node is not None:
            node.destroy_node()
        # SIGTERM 下 rclpy 可能已经关过一次，重复 shutdown 会抛 RCLError。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
