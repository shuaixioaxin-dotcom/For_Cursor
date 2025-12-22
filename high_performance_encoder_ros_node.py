#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass, field
from threading import Lock
from typing import Dict, List, Optional, Tuple
from collections import deque
import statistics

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import serial


@dataclass
class CycleDiagnostics:
    """单次采样周期的诊断信息"""
    ts: float = 0.0
    expected: int = 0
    ok: int = 0
    missing: int = 0
    crc_fail: int = 0
    malformed: int = 0
    unexpected_slave: int = 0
    duplicate_ok: int = 0
    raw_bytes: int = 0
    missing_slaves: List[int] = field(default_factory=list)


@dataclass
class RollingDiagnostics:
    """滚动窗口统计"""
    window_size: int = 200
    cycles: int = 0
    expected: int = 0
    ok: int = 0
    missing: int = 0
    crc_fail: int = 0
    malformed: int = 0
    unexpected_slave: int = 0
    duplicate_ok: int = 0
    raw_bytes: int = 0

    loss_rate_history: deque = field(default_factory=lambda: deque(maxlen=200))
    crc_rate_history: deque = field(default_factory=lambda: deque(maxlen=200))

    def push(self, c: CycleDiagnostics):
        self.cycles += 1
        self.expected += c.expected
        self.ok += c.ok
        self.missing += c.missing
        self.crc_fail += c.crc_fail
        self.malformed += c.malformed
        self.unexpected_slave += c.unexpected_slave
        self.duplicate_ok += c.duplicate_ok
        self.raw_bytes += c.raw_bytes

        loss_rate = (c.missing / c.expected) if c.expected > 0 else 0.0
        crc_rate = (c.crc_fail / max(c.ok + c.crc_fail, 1))
        self.loss_rate_history.append(loss_rate)
        self.crc_rate_history.append(crc_rate)

    def snapshot(self) -> Dict[str, float]:
        exp = max(self.expected, 1)
        ok = self.ok
        miss = self.missing
        loss_rate_total = miss / exp
        crc_total_den = max(self.ok + self.crc_fail, 1)
        crc_rate_total = self.crc_fail / crc_total_den

        def safe_mean(xs: deque) -> float:
            return float(statistics.mean(xs)) if len(xs) > 0 else 0.0

        def safe_max(xs: deque) -> float:
            return float(max(xs)) if len(xs) > 0 else 0.0

        return {
            "cycles": float(self.cycles),
            "expected": float(self.expected),
            "ok": float(ok),
            "missing": float(miss),
            "loss_rate_total": float(loss_rate_total),
            "loss_rate_avg": safe_mean(self.loss_rate_history),
            "loss_rate_max": safe_max(self.loss_rate_history),
            "crc_fail": float(self.crc_fail),
            "crc_rate_total": float(crc_rate_total),
            "crc_rate_avg": safe_mean(self.crc_rate_history),
            "malformed": float(self.malformed),
            "unexpected_slave": float(self.unexpected_slave),
            "duplicate_ok": float(self.duplicate_ok),
            "raw_bytes": float(self.raw_bytes),
        }


class BaseEncoderReader:
    """编码器读取器基类"""

    def __init__(
        self,
        port: str,
        baudrate: int,
        joint_count: int,
        logger=None,
        *,
        max_angle_jump_deg: float = 120.0,
        stale_warn_s: float = 1.0,
        diagnostics_window: int = 200,
    ):
        self.port = port
        self.baudrate = baudrate
        self.joint_count = joint_count
        self.encoder_data = {f"J{i+1}": 0.0 for i in range(joint_count)}
        self.lock = Lock()
        self.logger = logger
        self.connection: Optional[serial.Serial] = None

        # 数据准确性/可靠性检查配置
        self.max_angle_jump_deg = float(max_angle_jump_deg)
        self.stale_warn_s = float(stale_warn_s)

        # 每个关节上一帧角度与更新时间，用于检测异常跳变/陈旧数据
        now = time.time()
        self._last_angle = {f"J{i+1}": 0.0 for i in range(joint_count)}
        self._last_update_ts = {f"J{i+1}": now for i in range(joint_count)}

        # 串口诊断统计
        self._diag_lock = Lock()
        self._last_cycle_diag = CycleDiagnostics(ts=now)
        self._rolling = RollingDiagnostics(window_size=int(diagnostics_window))
        self._rolling.loss_rate_history = deque(maxlen=int(diagnostics_window))
        self._rolling.crc_rate_history = deque(maxlen=int(diagnostics_window))
        self._reconnects = 0

    def crc16_modbus(self, data: bytes) -> int:
        """计算Modbus CRC16校验码"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return ((crc & 0xFF) << 8) | ((crc >> 8) & 0xFF)

    def connect(self) -> bool:
        """建立串口连接"""
        try:
            self.connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.001,
                write_timeout=0.1,
            )
            if self.logger:
                self.logger.info(f"成功连接到串口: {self.port}")
            return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"连接串口失败 {self.port}: {e}")
            return False

    def disconnect(self):
        """关闭串口连接"""
        if self.connection and self.connection.is_open:
            try:
                self.connection.close()
            finally:
                self.connection = None

    def _record_cycle_diag(self, diag: CycleDiagnostics):
        with self._diag_lock:
            self._last_cycle_diag = diag
            self._rolling.push(diag)

    def get_diagnostics_report(self) -> Dict[str, object]:
        """返回用于日志的诊断汇总（丢包/CRC/帧错误等）"""
        with self._diag_lock:
            last = self._last_cycle_diag
            rolling = self._rolling.snapshot()

        return {
            "last_cycle": {
                "ts": last.ts,
                "expected": last.expected,
                "ok": last.ok,
                "missing": last.missing,
                "crc_fail": last.crc_fail,
                "malformed": last.malformed,
                "unexpected_slave": last.unexpected_slave,
                "duplicate_ok": last.duplicate_ok,
                "raw_bytes": last.raw_bytes,
                "missing_slaves": last.missing_slaves,
            },
            "rolling": rolling,
            "reconnects": self._reconnects,
        }

    def get_data_quality_warnings(self) -> List[str]:
        """返回基于历史值的“可能不准确/不新鲜”告警（不阻塞发布）"""
        now = time.time()
        warnings: List[str] = []

        # 1) 陈旧数据：超过 stale_warn_s 未更新
        if self.stale_warn_s > 0:
            stale = []
            for k, ts in self._last_update_ts.items():
                if now - ts > self.stale_warn_s:
                    stale.append(k)
            if stale:
                warnings.append(
                    f"数据可能陈旧: {stale} 超过 {self.stale_warn_s:.2f}s 未更新"
                )

        # 2) 异常跳变：与上一帧差值过大
        if self.max_angle_jump_deg > 0:
            jumps = []
            for k, ang in self.encoder_data.items():
                prev = self._last_angle.get(k, ang)
                # 处理 0/360 环绕：取最小角度差
                d = abs((ang - prev + 180.0) % 360.0 - 180.0)
                if d > self.max_angle_jump_deg:
                    jumps.append(f"{k}:{prev:.1f}->{ang:.1f} (Δ{d:.1f}°)")
            if jumps:
                warnings.append(
                    f"检测到异常跳变(阈值 {self.max_angle_jump_deg:.1f}°): "
                    + ", ".join(jumps[:8])
                    + (" ..." if len(jumps) > 8 else "")
                )

        return warnings


class OptimizedEncoderReader(BaseEncoderReader):
    """优化版编码器读取器 - 流水线操作（带丢包/CRC/帧诊断）"""

    def __init__(
        self,
        port: str,
        baudrate: int,
        joint_count: int,
        logger=None,
        *,
        max_angle_jump_deg: float = 120.0,
        stale_warn_s: float = 1.0,
        diagnostics_window: int = 200,
    ):
        super().__init__(
            port,
            baudrate,
            joint_count,
            logger,
            max_angle_jump_deg=max_angle_jump_deg,
            stale_warn_s=stale_warn_s,
            diagnostics_window=diagnostics_window,
        )
        self.connect()

    def read_all_encoders_optimized(self) -> Dict[str, float]:
        """优化后的读取方法 - 流水线操作"""
        if not self.connection or not self.connection.is_open:
            if not self.connect():
                return self.encoder_data.copy()

        diag = CycleDiagnostics(ts=time.time())

        with self.lock:
            try:
                # 第一阶段：快速发送所有请求
                requests: List[Tuple[int, bytes]] = []
                for i in range(self.joint_count):
                    slave_addr = 0x01 + i
                    frame = bytes([slave_addr, 0x03, 0x00, 0x01, 0x00, 0x01])
                    crc = self.crc16_modbus(frame)
                    frame += bytes([(crc >> 8) & 0xFF, crc & 0xFF])
                    requests.append((slave_addr, frame))

                expected_addrs = [addr for addr, _ in requests]
                diag.expected = len(expected_addrs)

                # 批量发送请求
                self.connection.reset_input_buffer()
                for _, frame in requests:
                    self.connection.write(frame)
                    time.sleep(0.0014)  # 发送间隔

                # 第二阶段：批量读取响应
                wait_time = max(0.001, len(requests) * 0.0002)
                time.sleep(wait_time)

                total_response = b""
                start_time = time.time()
                read_timeout = max(0.002, len(requests) * 0.0003)
                while time.time() - start_time < read_timeout:
                    if self.connection.in_waiting > 0:
                        data = self.connection.read(self.connection.in_waiting)
                        total_response += data
                    time.sleep(0.001)

                diag.raw_bytes = len(total_response)

                # 第三阶段：解析响应（同时统计丢包/CRC/格式错误）
                ok_addrs = self._parse_responses(total_response, expected_addrs, diag)

                missing = sorted(list(set(expected_addrs) - set(ok_addrs)))
                diag.missing = len(missing)
                diag.missing_slaves = missing

            except Exception as e:
                if self.logger:
                    self.logger.error(f"读取编码器错误: {e}")
                self.disconnect()
                self._reconnects += 1
                self.connect()

        self._record_cycle_diag(diag)
        return self.encoder_data.copy()

    def _parse_responses(
        self,
        response_data: bytes,
        expected_addrs: List[int],
        diag: CycleDiagnostics,
    ) -> List[int]:
        """解析批量响应数据（7字节帧: addr,0x03,0x02,hi,lo,crc_hi,crc_lo）"""
        ok_addrs: List[int] = []
        ok_seen = set()

        n = len(response_data)
        pos = 0

        while pos < n:
            b0 = response_data[pos]
            if b0 not in expected_addrs:
                pos += 1
                continue

            # 尝试读取一帧
            if pos + 7 > n:
                # 不足一帧，视为残帧/丢字节
                diag.malformed += 1
                break

            frame = response_data[pos : pos + 7]

            # 基本结构检查
            if frame[1] != 0x03 or frame[2] != 0x02:
                diag.malformed += 1
                pos += 1  # resync
                continue

            # CRC校验
            crc_calculated = self.crc16_modbus(frame[:5])
            crc_received = (frame[5] << 8) | frame[6]
            if crc_calculated != crc_received:
                diag.crc_fail += 1
                pos += 1  # resync
                continue

            slave_addr = frame[0]
            if slave_addr not in expected_addrs:
                diag.unexpected_slave += 1
                pos += 7
                continue

            # 解析寄存器值
            joint_index = slave_addr - 0x01
            if 0 <= joint_index < self.joint_count:
                register_value = (frame[3] << 8) | frame[4]
                angle = (register_value / 65536.0) * 360.0
                # 规整到 [0, 360)
                angle = angle % 360.0
                angle = round(angle, 2)

                key = f"J{joint_index+1}"
                # 更新历史，用于跳变/陈旧检查
                self._last_angle[key] = self.encoder_data.get(key, angle)
                self.encoder_data[key] = angle
                self._last_update_ts[key] = time.time()

                if slave_addr in ok_seen:
                    diag.duplicate_ok += 1
                else:
                    ok_seen.add(slave_addr)

                ok_addrs.append(slave_addr)
                diag.ok = len(ok_seen)

            pos += 7

        return ok_addrs


class ParallelEncoderReader(OptimizedEncoderReader):
    """
    兼容参数的“并行”读取器占位实现：
    单串口无法真正并行，这里复用流水线读取，但保留接口以避免 NameError。
    """

    def read_all_encoders_parallel(self) -> Dict[str, float]:
        return self.read_all_encoders_optimized()


class AdaptiveEncoderReader(OptimizedEncoderReader):
    """自适应读取：根据最近缺包率适当延长等待/读取超时（提升准确性）"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._adaptive_extra_wait = 0.0

    def read_all_encoders_adaptive(self) -> Dict[str, float]:
        # 先跑一次读取（复用优化版），再根据 last_cycle 的缺包率调整下次等待
        data = self.read_all_encoders_optimized()

        rep = self.get_diagnostics_report()
        last = rep["last_cycle"]
        expected = int(last["expected"])
        missing = int(last["missing"])
        loss_rate = (missing / expected) if expected > 0 else 0.0

        # 简单自适应：缺包越高，下一轮额外等待越多（上限 5ms）
        target = min(0.005, max(0.0, (loss_rate - 0.02) * 0.02))
        self._adaptive_extra_wait = 0.7 * self._adaptive_extra_wait + 0.3 * target
        if self._adaptive_extra_wait > 0:
            time.sleep(self._adaptive_extra_wait)

        return data


class HighPerformanceEncoderROSNode(Node):
    """高性能编码器ROS节点（带串口丢包/准确性诊断日志）"""

    def __init__(self):
        super().__init__("high_performance_encoder_ros_node")

        # 声明参数
        self.declare_parameter("publish_frequency", 30.0)
        self.declare_parameter("joint_count", 16)  # 支持1-16个编码器站号
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 230400)
        self.declare_parameter("read_method", "optimized")  # optimized, parallel, adaptive
        self.declare_parameter(
            "joint_names",
            [
                "joint1",
                "joint2",
                "joint3",
                "joint4",
                "joint5",
                "joint6",
                "joint7",
                "joint8",
                "joint9",
                "joint10",
                "joint11",
                "joint12",
                "joint13",
                "joint14",
                "joint15",
                "joint16",
            ],
        )

        # 诊断相关参数
        self.declare_parameter("diagnostics_window", 200)
        self.declare_parameter("stale_warn_s", 1.0)
        self.declare_parameter("max_angle_jump_deg", 120.0)
        self.declare_parameter("packet_loss_warn_rate", 0.10)  # 最近窗口平均缺包率超过该值则WARN
        self.declare_parameter("crc_warn_rate", 0.02)  # 最近窗口平均CRC失败率超过该值则WARN

        # 获取参数
        publish_frequency = float(self.get_parameter("publish_frequency").value)
        joint_count = int(self.get_parameter("joint_count").value)
        serial_port = str(self.get_parameter("serial_port").value)
        baudrate = int(self.get_parameter("baudrate").value)
        read_method = str(self.get_parameter("read_method").value)
        joint_names = list(self.get_parameter("joint_names").value)

        diagnostics_window = int(self.get_parameter("diagnostics_window").value)
        stale_warn_s = float(self.get_parameter("stale_warn_s").value)
        max_angle_jump_deg = float(self.get_parameter("max_angle_jump_deg").value)
        self.packet_loss_warn_rate = float(
            self.get_parameter("packet_loss_warn_rate").value
        )
        self.crc_warn_rate = float(self.get_parameter("crc_warn_rate").value)

        # 验证参数
        if len(joint_names) != joint_count:
            self.get_logger().warning(
                f"关节名称数量({len(joint_names)})与关节数量({joint_count})不匹配!"
            )
            joint_names = [f"joint{i+1}" for i in range(joint_count)]

        self.joint_names = joint_names

        # 根据配置选择读取器
        reader_kwargs = dict(
            max_angle_jump_deg=max_angle_jump_deg,
            stale_warn_s=stale_warn_s,
            diagnostics_window=diagnostics_window,
        )
        if read_method == "parallel":
            self.encoder_reader = ParallelEncoderReader(
                serial_port, baudrate, joint_count, self.get_logger(), **reader_kwargs
            )
            self.read_func = self.encoder_reader.read_all_encoders_parallel
        elif read_method == "adaptive":
            self.encoder_reader = AdaptiveEncoderReader(
                serial_port, baudrate, joint_count, self.get_logger(), **reader_kwargs
            )
            self.read_func = self.encoder_reader.read_all_encoders_adaptive
        else:
            self.encoder_reader = OptimizedEncoderReader(
                serial_port, baudrate, joint_count, self.get_logger(), **reader_kwargs
            )
            self.read_func = self.encoder_reader.read_all_encoders_optimized

        # 创建发布者
        self.joint_pub = self.create_publisher(JointState, "body_states", 10)
        timer_period = 1.0 / publish_frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 统计变量
        self.publish_count = 0
        self.start_time = self.get_clock().now()
        self.last_log_time = self.start_time
        self.frequency_history: List[float] = []

        self.get_logger().info(
            "高性能编码器ROS节点已启动:\n"
            f"  发布频率: {publish_frequency}Hz\n"
            f"  关节数量: {joint_count}\n"
            f"  串口: {serial_port}\n"
            f"  波特率: {baudrate}\n"
            f"  读取方式: {read_method}\n"
            f"  关节名称: {joint_names}\n"
            f"  诊断窗口: {diagnostics_window}\n"
            f"  陈旧告警: {stale_warn_s}s\n"
            f"  跳变阈值: {max_angle_jump_deg}°"
        )

    def timer_callback(self):
        """定时器回调函数"""
        read_start_time = time.time()
        encoder_data = self.read_func()
        read_duration = time.time() - read_start_time

        # 创建JointState消息
        joint_msg = JointState()
        joint_msg.header = Header()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = "encoder_base"

        # 填充关节数据
        joint_positions = []
        for i in range(len(self.joint_names)):
            encoder_key = f"J{i+1}"
            angle_degrees = float(encoder_data.get(encoder_key, 0.0))
            joint_positions.append(math.radians(angle_degrees))

        joint_msg.name = self.joint_names
        joint_msg.position = joint_positions
        joint_msg.velocity = [0.0] * len(self.joint_names)
        joint_msg.effort = [0.0] * len(self.joint_names)

        self.joint_pub.publish(joint_msg)

        # 统计发布频率与诊断（每3秒输出一次）
        self.publish_count += 1
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.last_log_time).nanoseconds / 1e9

        if elapsed_time >= 3.0:
            actual_freq = self.publish_count / elapsed_time
            self.frequency_history.append(actual_freq)
            if len(self.frequency_history) > 10:
                self.frequency_history = self.frequency_history[-10:]

            avg_freq = float(np.mean(self.frequency_history))
            max_freq = float(np.max(self.frequency_history))
            min_freq = float(np.min(self.frequency_history))

            diag = self.encoder_reader.get_diagnostics_report()
            rolling = diag["rolling"]
            last = diag["last_cycle"]

            # 根据窗口平均丢包率/CRC率决定日志级别
            loss_avg = float(rolling.get("loss_rate_avg", 0.0))
            crc_avg = float(rolling.get("crc_rate_avg", 0.0))
            needs_warn = (loss_avg >= self.packet_loss_warn_rate) or (
                crc_avg >= self.crc_warn_rate
            )

            quality_warnings = self.encoder_reader.get_data_quality_warnings()

            msg = (
                f"频率统计 - 当前: {actual_freq:.2f}Hz, 平均: {avg_freq:.2f}Hz, "
                f"最小: {min_freq:.2f}Hz, 最大: {max_freq:.2f}Hz, "
                f"读取耗时: {read_duration*1000:.1f}ms\n"
                f"串口诊断(最近周期) - 期望:{int(last['expected'])}, OK:{int(last['ok'])}, "
                f"缺失:{int(last['missing'])}, CRC:{int(last['crc_fail'])}, "
                f"格式错:{int(last['malformed'])}, 重复:{int(last['duplicate_ok'])}, "
                f"字节:{int(last['raw_bytes'])}, 缺失从站:{last['missing_slaves']}\n"
                f"串口诊断(滚动窗口) - 缺包率avg:{loss_avg*100:.1f}%, max:{float(rolling.get('loss_rate_max',0.0))*100:.1f}%, "
                f"CRC率avg:{crc_avg*100:.2f}%, malformed:{int(rolling.get('malformed',0))}, reconnects:{int(diag.get('reconnects',0))}\n"
                f"最新数据: {encoder_data}"
            )

            if quality_warnings:
                msg += "\n数据质量告警: " + " | ".join(quality_warnings)

            if needs_warn:
                self.get_logger().warning(msg)
            else:
                self.get_logger().info(msg)

            self.publish_count = 0
            self.last_log_time = current_time

    def destroy_node(self):
        """节点销毁时的清理工作"""
        if hasattr(self, "encoder_reader"):
            self.encoder_reader.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node: Optional[HighPerformanceEncoderROSNode] = None
    try:
        node = HighPerformanceEncoderROSNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("节点被用户中断")
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"节点运行错误: {e}")
        else:
            print(f"节点初始化错误: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

