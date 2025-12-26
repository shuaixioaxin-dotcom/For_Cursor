#!/usr/bin/env python3
"""
Optimized encoder reader test script (Modbus RTU-like).

默认参数符合你的要求：
- 端口：COM35
- 波特率：230400
- 站号：0x01 ~ 0x10 (1~16)

可调参数：
- 相邻站号发送间隔（--send-gap-ms）
- 批量发送后等待（--pre-read-wait-ms）
- 批量读取窗口（--read-window-ms）
- 轮询串口缓存间隔（--poll-sleep-ms）
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Tuple

try:
    import serial  # type: ignore
except Exception as e:  # pragma: no cover
    print(
        "缺少依赖 pyserial，先安装：pip install -r requirements.txt\n"
        f"导入错误：{e}",
        file=sys.stderr,
    )
    raise


def crc16_modbus(data: bytes) -> int:
    """Modbus CRC16: poly=0xA001, init=0xFFFF, little-endian on wire (Lo then Hi).

    这里返回 16bit 整数，后续拼帧时按你给的逻辑：先高字节再低字节。
    若你的设备严格按 Modbus RTU 低字节在前，可以用 --crc-order=lohi 切换。
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


@dataclass(frozen=True)
class ReaderTiming:
    send_gap_s: float
    pre_read_wait_s: float
    read_window_s: float
    poll_sleep_s: float


class OptimizedEncoderReader:
    def __init__(
        self,
        port: str,
        baudrate: int,
        joint_count: int,
        *,
        crc_order: str = "hilo",
        serial_timeout_s: float = 0.0,
        write_timeout_s: float = 0.2,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.joint_count = joint_count
        self.crc_order = crc_order
        self.serial_timeout_s = serial_timeout_s
        self.write_timeout_s = write_timeout_s

        self.connection: "serial.Serial | None" = None
        self.encoder_data: Dict[str, float] = {f"J{i+1}": 0.0 for i in range(joint_count)}

    def connect(self) -> bool:
        try:
            if self.connection and self.connection.is_open:
                return True
            self.connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.serial_timeout_s,  # 0=非阻塞
                write_timeout=self.write_timeout_s,
            )
            return True
        except Exception as e:
            print(f"[connect] 打开串口失败: {e}", file=sys.stderr)
            self.connection = None
            return False

    def disconnect(self) -> None:
        try:
            if self.connection:
                self.connection.close()
        finally:
            self.connection = None

    def _build_request_frame(self, slave_addr: int) -> bytes:
        frame = bytes([slave_addr, 0x03, 0x00, 0x01, 0x00, 0x01])
        crc = crc16_modbus(frame)
        hi = (crc >> 8) & 0xFF
        lo = crc & 0xFF
        if self.crc_order.lower() == "hilo":
            frame += bytes([hi, lo])
        elif self.crc_order.lower() == "lohi":
            frame += bytes([lo, hi])
        else:
            raise ValueError("crc_order 只能是 hilo 或 lohi")
        return frame

    def read_all_encoders_optimized(
        self,
        *,
        start_addr: int = 0x01,
        timings: ReaderTiming,
        debug: bool = False,
        dump_raw: bool = False,
    ) -> Tuple[Dict[str, float], Dict[str, int]]:
        """返回 (角度字典, 统计信息)."""
        if not self.connection or not self.connection.is_open:
            if not self.connect():
                return self.encoder_data.copy(), {"ok_frames": 0, "bad_crc": 0, "garbage": 0, "bytes": 0}

        # 第一阶段：快速发送所有请求
        requests: List[Tuple[int, bytes]] = []
        for i in range(self.joint_count):
            slave_addr = start_addr + i
            requests.append((slave_addr, self._build_request_frame(slave_addr)))

        try:
            assert self.connection is not None
            self.connection.reset_input_buffer()

            for slave_addr, frame in requests:
                self.connection.write(frame)
                if debug:
                    print(f"[tx] addr=0x{slave_addr:02X} frame={frame.hex(' ')}")
                if timings.send_gap_s > 0:
                    time.sleep(timings.send_gap_s)

            # 第二阶段：等待所有设备响应
            if timings.pre_read_wait_s > 0:
                time.sleep(timings.pre_read_wait_s)

            # 第三阶段：批量读取响应
            total_response = b""
            start_time = time.time()
            while time.time() - start_time < timings.read_window_s:
                waiting = self.connection.in_waiting
                if waiting and waiting > 0:
                    chunk = self.connection.read(waiting)
                    total_response += chunk
                if timings.poll_sleep_s > 0:
                    time.sleep(timings.poll_sleep_s)

            if dump_raw:
                print(f"[rx] bytes={len(total_response)} raw={total_response.hex(' ')}")

            stats = self._parse_responses(
                response_data=total_response,
                requests=requests,
                start_addr=start_addr,
                debug=debug,
            )
            stats["bytes"] = len(total_response)
            return self.encoder_data.copy(), stats
        except Exception as e:
            print(f"[read] 读取编码器错误: {e}", file=sys.stderr)
            # 发生错误时重新连接
            self.disconnect()
            self.connect()
            return self.encoder_data.copy(), {"ok_frames": 0, "bad_crc": 0, "garbage": 0, "bytes": 0}

    def _parse_responses(
        self,
        *,
        response_data: bytes,
        requests: List[Tuple[int, bytes]],
        start_addr: int,
        debug: bool,
    ) -> Dict[str, int]:
        """按你给的逻辑：从 blob 中找 7 字节响应帧并验 CRC。"""
        slave_addrs = {addr for addr, _ in requests}
        response_frames: List[bytes] = []

        bad_crc = 0
        garbage = 0
        current_pos = 0
        while current_pos < len(response_data):
            b0 = response_data[current_pos]
            if b0 in slave_addrs:
                if current_pos + 7 <= len(response_data):
                    frame = response_data[current_pos : current_pos + 7]
                    if len(frame) == 7 and frame[1] == 0x03:
                        # frame: [addr, 0x03, byte_count(=0x02), hi, lo, crc?, crc?]
                        crc_calc = crc16_modbus(frame[:5])
                        crc_hi = frame[5]
                        crc_lo = frame[6]
                        if self.crc_order.lower() == "hilo":
                            crc_recv = (crc_hi << 8) | crc_lo
                        else:
                            crc_recv = (crc_lo << 8) | crc_hi
                        if crc_calc == crc_recv:
                            response_frames.append(frame)
                            if debug:
                                print(f"[rx-ok] frame={frame.hex(' ')}")
                            current_pos += 7
                            continue
                        else:
                            bad_crc += 1
                            if debug:
                                print(
                                    "[rx-bad-crc] "
                                    f"frame={frame.hex(' ')} calc=0x{crc_calc:04X} recv=0x{crc_recv:04X}"
                                )
                    # 未通过：按 1 字节滑窗继续找
                    current_pos += 1
                else:
                    break
            else:
                garbage += 1
                current_pos += 1

        # 解析每个响应帧
        for frame in response_frames:
            slave_addr = frame[0]
            joint_index = slave_addr - start_addr
            if 0 <= joint_index < self.joint_count:
                register_value = (frame[3] << 8) | frame[4]
                angle = round((register_value / 65536.0) * 360.0, 2)
                self.encoder_data[f"J{joint_index+1}"] = angle

        return {"ok_frames": len(response_frames), "bad_crc": bad_crc, "garbage": garbage}


def _parse_args(argv: List[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Optimized encoder reader test")
    p.add_argument("--port", default="COM35", help="串口端口（Windows: COM35；Linux: /dev/ttyUSB0 等）")
    p.add_argument("--baudrate", type=int, default=230400, help="波特率")
    p.add_argument("--start-addr", default="0x01", help="起始站号（hex 或 dec），默认 0x01")
    p.add_argument("--count", type=int, default=16, help="站号数量（默认 16 -> 0x01~0x10）")

    # 等待时间（毫秒）
    p.add_argument("--send-gap-ms", type=float, default=1.0, help="相邻站号发送间隔（ms）")
    p.add_argument("--pre-read-wait-ms", type=float, default=1.0, help="批量发送后等待（ms）")
    p.add_argument("--read-window-ms", type=float, default=1.0, help="读取窗口总时长（ms）")
    p.add_argument("--poll-sleep-ms", type=float, default=1.0, help="读取窗口内轮询 sleep（ms）")

    p.add_argument("--interval-ms", type=float, default=50.0, help="循环读取间隔（ms）")
    p.add_argument("--loops", type=int, default=0, help="循环次数（0=无限循环）")

    p.add_argument("--crc-order", choices=["hilo", "lohi"], default="hilo", help="CRC 拼接字节序")
    p.add_argument("--debug", action="store_true", help="打印发送帧/解析帧等调试信息")
    p.add_argument("--dump-raw", action="store_true", help="打印收到的原始字节流（hex）")
    return p.parse_args(argv)


def _parse_int_auto(s: str) -> int:
    s = s.strip().lower()
    if s.startswith("0x"):
        return int(s, 16)
    return int(s, 10)


def main(argv: List[str]) -> int:
    args = _parse_args(argv)
    start_addr = _parse_int_auto(args.start_addr)
    joint_count = int(args.count)

    reader = OptimizedEncoderReader(
        port=args.port,
        baudrate=int(args.baudrate),
        joint_count=joint_count,
        crc_order=args.crc_order,
    )

    timings = ReaderTiming(
        send_gap_s=max(args.send_gap_ms, 0.0) / 1000.0,
        pre_read_wait_s=max(args.pre_read_wait_ms, 0.0) / 1000.0,
        read_window_s=max(args.read_window_ms, 0.0) / 1000.0,
        poll_sleep_s=max(args.poll_sleep_ms, 0.0) / 1000.0,
    )

    if not reader.connect():
        return 2

    loops = int(args.loops)
    interval_s = max(args.interval_ms, 0.0) / 1000.0

    i = 0
    while True:
        i += 1
        data, stats = reader.read_all_encoders_optimized(
            start_addr=start_addr,
            timings=timings,
            debug=bool(args.debug),
            dump_raw=bool(args.dump_raw),
        )

        # 输出格式：每次一行，便于你复制到日志/表格
        pairs = " ".join([f"J{idx+1}={data[f'J{idx+1}']:.2f}" for idx in range(joint_count)])
        print(
            f"#{i} start=0x{start_addr:02X} count={joint_count} "
            f"ok={stats.get('ok_frames', 0)} bad_crc={stats.get('bad_crc', 0)} bytes={stats.get('bytes', 0)} | "
            f"{pairs}"
        )

        if loops > 0 and i >= loops:
            break
        if interval_s > 0:
            time.sleep(interval_s)

    reader.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

