#!/usr/bin/env python3
"""
RS-485 Modbus RTU encoder polling benchmark (1..16).

目标：
- 通过串口连接485总线
- 轮询16个编码器（站号0x01~0x10），读取一个16bit寄存器
- 统计“有效编码器数值”的最大获取频率（有效帧/秒、轮询周期Hz）

说明：
- 默认请求：功能码0x03，起始寄存器0x0001，数量0x0001
- 默认响应长度：7字节（addr, 0x03, 0x02, hi, lo, crc_lo, crc_hi）
- CRC：Modbus RTU CRC16，小端附加（低字节在前）
"""

from __future__ import annotations

import argparse
import statistics
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import serial


def crc16_modbus(data: bytes) -> int:
    """标准Modbus RTU CRC16 (poly 0xA001)，返回0~0xFFFF。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_read_holding_registers_request(
    slave_addr: int, start_reg: int, reg_count: int
) -> bytes:
    pdu = bytes(
        [
            slave_addr & 0xFF,
            0x03,
            (start_reg >> 8) & 0xFF,
            start_reg & 0xFF,
            (reg_count >> 8) & 0xFF,
            reg_count & 0xFF,
        ]
    )
    crc = crc16_modbus(pdu)
    return pdu + bytes([crc & 0xFF, (crc >> 8) & 0xFF])  # CRC低字节在前


@dataclass(frozen=True)
class BenchmarkResult:
    duration_s: float
    loops: int
    valid_frames: int
    valid_loops: int
    full_loops: int
    loops_hz: float
    valid_frames_per_s: float
    full_loops_hz: float
    per_loop_valid_frames_mean: float
    per_loop_valid_frames_p50: float
    per_loop_valid_frames_p95: float


class BaseEncoderReader:
    def __init__(
        self,
        port: str,
        baudrate: int,
        joint_count: int = 16,
        start_reg: int = 0x0001,
        reg_count: int = 0x0001,
        parity: str = "N",
        stopbits: int = 1,
        bytesize: int = 8,
        timeout_s: float = 0.0,
        logger=None,
        rs485: bool = False,
        rts_level_for_tx: bool = True,
        rts_level_for_rx: bool = False,
        delay_before_tx: float = 0.0,
        delay_before_rx: float = 0.0,
    ):
        self.port = port
        self.baudrate = baudrate
        self.joint_count = joint_count
        self.start_reg = start_reg
        self.reg_count = reg_count
        self.parity = parity
        self.stopbits = stopbits
        self.bytesize = bytesize
        self.timeout_s = timeout_s
        self.logger = logger

        self.rs485 = rs485
        self.rts_level_for_tx = rts_level_for_tx
        self.rts_level_for_rx = rts_level_for_rx
        self.delay_before_tx = delay_before_tx
        self.delay_before_rx = delay_before_rx

        self.connection: Optional[serial.Serial] = None
        self.lock = threading.Lock()
        self.encoder_data: Dict[str, float] = {
            f"J{i + 1}": 0.0 for i in range(self.joint_count)
        }

    def connect(self) -> bool:
        try:
            self.connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=self.bytesize,
                parity=self.parity,
                stopbits=self.stopbits,
                timeout=self.timeout_s,
                write_timeout=0.2,
            )

            # 可选：开启pyserial的RS485模式（如硬件/驱动支持）
            if self.rs485:
                try:
                    from serial.rs485 import RS485Settings

                    self.connection.rs485_mode = RS485Settings(
                        rts_level_for_tx=self.rts_level_for_tx,
                        rts_level_for_rx=self.rts_level_for_rx,
                        delay_before_tx=self.delay_before_tx,
                        delay_before_rx=self.delay_before_rx,
                    )
                except Exception as e:
                    if self.logger:
                        self.logger.warning(f"RS485模式设置失败，将继续普通串口模式: {e}")

            self.connection.reset_input_buffer()
            self.connection.reset_output_buffer()
            return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"串口连接失败: {e}")
            self.connection = None
            return False

    def disconnect(self) -> None:
        try:
            if self.connection:
                self.connection.close()
        finally:
            self.connection = None


class OptimizedEncoderReader(BaseEncoderReader):
    """
    优化策略：
    - 第一阶段：快速发送所有从站请求
    - 第二阶段：短窗口批量读取所有返回字节
    - 第三阶段：从拼接缓冲区里扫描/校验/解析多个7字节响应帧
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.connect()

    def read_all_encoders_optimized(
        self,
        tx_gap_s: float = 0.0005,
        pre_rx_wait_s: float = 0.0010,
        rx_window_s: float = 0.0030,
        rx_poll_s: float = 0.0003,
    ) -> Tuple[Dict[str, float], int]:
        """
        返回：(encoder_data_copy, valid_frames_count)
        """
        if not self.connection or not self.connection.is_open:
            if not self.connect():
                return self.encoder_data.copy(), 0

        with self.lock:
            try:
                requests: List[Tuple[int, bytes]] = []
                for i in range(self.joint_count):
                    slave_addr = 0x01 + i
                    frame = build_read_holding_registers_request(
                        slave_addr, self.start_reg, self.reg_count
                    )
                    requests.append((slave_addr, frame))

                self.connection.reset_input_buffer()

                # 第一阶段：批量发送
                for _, frame in requests:
                    self.connection.write(frame)
                    if tx_gap_s > 0:
                        time.sleep(tx_gap_s)

                # 第二阶段：等待设备响应并批量读取
                if pre_rx_wait_s > 0:
                    time.sleep(pre_rx_wait_s)

                total_response = b""
                start = time.perf_counter()
                while (time.perf_counter() - start) < rx_window_s:
                    n = self.connection.in_waiting if self.connection else 0
                    if n and self.connection:
                        total_response += self.connection.read(n)
                    if rx_poll_s > 0:
                        time.sleep(rx_poll_s)

                # 第三阶段：解析
                valid = self._parse_responses(total_response, requests)
                return self.encoder_data.copy(), valid

            except Exception as e:
                if self.logger:
                    self.logger.error(f"读取编码器错误: {e}")
                self.disconnect()
                self.connect()
                return self.encoder_data.copy(), 0

    def _parse_responses(self, response_data: bytes, requests: List[Tuple[int, bytes]]) -> int:
        slave_addrs = {addr for addr, _ in requests}
        valid_frames = 0

        pos = 0
        while pos < len(response_data):
            b0 = response_data[pos]
            if b0 not in slave_addrs:
                pos += 1
                continue

            # 期望响应帧 7字节：addr, 0x03, 0x02, hi, lo, crc_lo, crc_hi
            if pos + 7 > len(response_data):
                break

            frame = response_data[pos : pos + 7]
            if frame[1] != 0x03 or frame[2] != 0x02:
                pos += 1
                continue

            crc_calc = crc16_modbus(frame[:5])
            crc_recv = frame[5] | (frame[6] << 8)
            if crc_calc != crc_recv:
                pos += 1
                continue

            slave_addr = frame[0]
            joint_index = slave_addr - 0x01
            if 0 <= joint_index < self.joint_count:
                reg_val = (frame[3] << 8) | frame[4]
                angle = round((reg_val / 65536.0) * 360.0, 2)
                self.encoder_data[f"J{joint_index + 1}"] = angle
                valid_frames += 1

            pos += 7

        return valid_frames


def run_benchmark(
    reader: OptimizedEncoderReader,
    duration_s: float,
    tx_gap_s: float,
    pre_rx_wait_s: float,
    rx_window_s: float,
    rx_poll_s: float,
    print_every_s: float = 1.0,
) -> BenchmarkResult:
    t0 = time.perf_counter()
    next_print = t0 + print_every_s if print_every_s > 0 else float("inf")

    loops = 0
    valid_frames_total = 0
    valid_loops = 0
    full_loops = 0
    per_loop_valid: List[int] = []

    while True:
        now = time.perf_counter()
        if now - t0 >= duration_s:
            break

        _, valid = reader.read_all_encoders_optimized(
            tx_gap_s=tx_gap_s,
            pre_rx_wait_s=pre_rx_wait_s,
            rx_window_s=rx_window_s,
            rx_poll_s=rx_poll_s,
        )

        loops += 1
        valid_frames_total += valid
        per_loop_valid.append(valid)
        if valid > 0:
            valid_loops += 1
        if valid >= reader.joint_count:
            full_loops += 1

        if now >= next_print:
            elapsed = now - t0
            loops_hz = loops / elapsed if elapsed > 0 else 0.0
            vfps = valid_frames_total / elapsed if elapsed > 0 else 0.0
            print(
                f"[{elapsed:6.2f}s] loops={loops} ({loops_hz:7.1f} Hz) "
                f"valid_frames={valid_frames_total} ({vfps:7.1f}/s) "
                f"full_loops={full_loops}"
            )
            next_print = now + print_every_s

    t1 = time.perf_counter()
    elapsed = max(1e-9, t1 - t0)
    loops_hz = loops / elapsed
    vfps = valid_frames_total / elapsed
    full_hz = full_loops / elapsed

    if per_loop_valid:
        mean_v = statistics.fmean(per_loop_valid)
        p50 = statistics.median(per_loop_valid)
        p95 = statistics.quantiles(per_loop_valid, n=20)[18]  # 95%近似
    else:
        mean_v = p50 = p95 = 0.0

    return BenchmarkResult(
        duration_s=elapsed,
        loops=loops,
        valid_frames=valid_frames_total,
        valid_loops=valid_loops,
        full_loops=full_loops,
        loops_hz=loops_hz,
        valid_frames_per_s=vfps,
        full_loops_hz=full_hz,
        per_loop_valid_frames_mean=float(mean_v),
        per_loop_valid_frames_p50=float(p50),
        per_loop_valid_frames_p95=float(p95),
    )


def main() -> int:
    ap = argparse.ArgumentParser(
        description="RS-485(Modbus RTU) 16编码器轮询最大有效频率测试"
    )
    ap.add_argument("--port", required=True, help="串口设备，如 /dev/ttyUSB0")
    ap.add_argument("--baudrate", type=int, default=115200, help="波特率")
    ap.add_argument("--joints", type=int, default=16, help="编码器数量(站号从1递增)")
    ap.add_argument("--start-reg", type=lambda x: int(x, 0), default="0x0001", help="起始寄存器(支持0x前缀)")
    ap.add_argument("--reg-count", type=lambda x: int(x, 0), default="0x0001", help="寄存器数量(支持0x前缀)")

    ap.add_argument("--duration", type=float, default=10.0, help="测试时长(秒)")
    ap.add_argument("--tx-gap", type=float, default=0.0005, help="连续发送请求间隔(秒)")
    ap.add_argument("--pre-rx-wait", type=float, default=0.0010, help="发送完后等待响应(秒)")
    ap.add_argument("--rx-window", type=float, default=0.0030, help="读响应窗口(秒)")
    ap.add_argument("--rx-poll", type=float, default=0.0003, help="窗口内轮询睡眠(秒)")
    ap.add_argument("--print-every", type=float, default=1.0, help="每隔多少秒打印一次实时统计(0表示不打印)")

    ap.add_argument("--parity", default="N", choices=["N", "E", "O"], help="校验位")
    ap.add_argument("--stopbits", type=int, default=1, choices=[1, 2], help="停止位")
    ap.add_argument("--bytesize", type=int, default=8, choices=[7, 8], help="数据位")
    ap.add_argument("--timeout", type=float, default=0.0, help="串口读超时(秒)，0表示非阻塞")

    ap.add_argument("--rs485", action="store_true", help="尝试启用pyserial RS485模式(若驱动支持)")
    ap.add_argument("--rts-tx", action="store_true", help="RS485模式下TX时RTS拉高(默认False)")
    ap.add_argument("--rts-rx", action="store_true", help="RS485模式下RX时RTS拉高(默认False)")
    ap.add_argument("--delay-before-tx", type=float, default=0.0, help="RS485模式下TX前延时(秒)")
    ap.add_argument("--delay-before-rx", type=float, default=0.0, help="RS485模式下RX前延时(秒)")

    args = ap.parse_args()

    reader = OptimizedEncoderReader(
        port=args.port,
        baudrate=args.baudrate,
        joint_count=args.joints,
        start_reg=args.start_reg,
        reg_count=args.reg_count,
        parity=args.parity,
        stopbits=args.stopbits,
        bytesize=args.bytesize,
        timeout_s=args.timeout,
        rs485=args.rs485,
        rts_level_for_tx=args.rts_tx,
        rts_level_for_rx=args.rts_rx,
        delay_before_tx=args.delay_before_tx,
        delay_before_rx=args.delay_before_rx,
    )

    if not reader.connection or not reader.connection.is_open:
        print("串口未能打开，请检查port/权限/线缆/转换器。")
        return 2

    try:
        print(
            "开始测试："
            f"port={args.port} baudrate={args.baudrate} joints={args.joints} "
            f"tx_gap={args.tx_gap}s pre_rx_wait={args.pre_rx_wait}s "
            f"rx_window={args.rx_window}s rx_poll={args.rx_poll}s duration={args.duration}s"
        )

        result = run_benchmark(
            reader=reader,
            duration_s=args.duration,
            tx_gap_s=args.tx_gap,
            pre_rx_wait_s=args.pre_rx_wait,
            rx_window_s=args.rx_window,
            rx_poll_s=args.rx_poll,
            print_every_s=args.print_every,
        )

        print("\n===== 结果汇总 =====")
        print(f"duration_s:              {result.duration_s:.3f}")
        print(f"loops:                   {result.loops}")
        print(f"loops_hz:                {result.loops_hz:.2f}")
        print(f"valid_frames:            {result.valid_frames}")
        print(f"valid_frames_per_s:      {result.valid_frames_per_s:.2f}")
        print(f"valid_loops:             {result.valid_loops}")
        print(f"full_loops(=16帧):        {result.full_loops}")
        print(f"full_loops_hz:           {result.full_loops_hz:.2f}")
        print(f"per_loop_valid_mean:     {result.per_loop_valid_frames_mean:.2f}")
        print(f"per_loop_valid_p50:      {result.per_loop_valid_frames_p50:.2f}")
        print(f"per_loop_valid_p95:      {result.per_loop_valid_frames_p95:.2f}")

        # 打印最后一次缓存值（便于快速验证角度在变）
        latest = reader.encoder_data.copy()
        print("\n最后一次解析到的角度示例(前8个)：")
        for k in list(latest.keys())[: min(8, len(latest))]:
            print(f"  {k}: {latest[k]}")

        return 0
    finally:
        reader.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())

