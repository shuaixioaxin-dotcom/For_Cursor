#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Modbus RTU 批量采集测试工具（多从站轮询）"""

import argparse
import struct
import time
from typing import List, Optional, Tuple

import serial

# 预计算常量
ACC_SCALE = 0.0048828  # 加速度转换系数
QUAT_SCALE = 0.0001    # 四元数转换系数
QUAT_OFFSET = 41       # 四元数数据偏移（3 + 38）
START_ADDR = 0x0034
QUANTITY = 0x0016
RESPONSE_LEN = 49      # 读0x16个寄存器的响应长度


def crc16_modbus(data: bytes) -> int:
    """计算Modbus RTU CRC16校验值（按单片机逻辑直接计算）"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def validate_crc(frame: bytes) -> bool:
    if len(frame) < 3:
        return False
    calc = crc16_modbus(frame[:-2])
    return frame[-2] == (calc & 0xFF) and frame[-1] == ((calc >> 8) & 0xFF)


def build_request(slave_id: int) -> bytes:
    """构建读保持寄存器请求帧（地址/数量固定，仅ID变化）"""
    payload = bytes([
        slave_id,
        0x03,
        (START_ADDR >> 8) & 0xFF,
        START_ADDR & 0xFF,
        (QUANTITY >> 8) & 0xFF,
        QUANTITY & 0xFF,
    ])
    crc = crc16_modbus(payload)
    return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def parse_response(
    response: bytes,
    slave_id: int,
) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]]:
    """解析Modbus响应帧，返回加速度和四元数"""
    if len(response) != RESPONSE_LEN:
        return None
    if response[0] != slave_id or response[1] != 0x03:
        return None
    if response[2] != QUANTITY * 2:
        return None
    if not validate_crc(response):
        return None
    accx_raw, accy_raw, accz_raw = struct.unpack_from(">3H", response, 3)
    accx_ms2 = (accx_raw if accx_raw < 32768 else accx_raw - 65536) * ACC_SCALE
    accy_ms2 = (accy_raw if accy_raw < 32768 else accy_raw - 65536) * ACC_SCALE
    accz_ms2 = (accz_raw if accz_raw < 32768 else accz_raw - 65536) * ACC_SCALE

    qw, qx, qy, qz = struct.unpack_from(">4h", response, QUAT_OFFSET)
    return (
        (accx_ms2, accy_ms2, accz_ms2),
        (qw * QUAT_SCALE, qx * QUAT_SCALE, qy * QUAT_SCALE, qz * QUAT_SCALE),
    )


def parse_id_list(value: str) -> List[int]:
    ids = []
    for item in value.split(","):
        item = item.strip()
        if item:
            ids.append(int(item, 0))
    return ids


def main() -> None:
    parser = argparse.ArgumentParser(description="Modbus RTU 批量采集测试工具")
    parser.add_argument("-p", "--port", type=str, default="COM66", help="串口端口（默认COM66）")
    parser.add_argument("-b", "--baudrate", type=int, default=921600, help="波特率（默认921600）")
    parser.add_argument("-t", "--timeout", type=float, default=0.02, help="单个响应超时秒（默认0.02）")
    parser.add_argument(
        "--ids",
        type=str,
        default="0x01,0x02",
        help="多个从站ID，逗号分隔，如: 0x01,0x02",
    )
    parser.add_argument("-f", "--freq", type=float, default=0, help="循环频率Hz（0=最高频率）")
    parser.add_argument("-c", "--count", type=int, default=0, help="循环次数（0=无限循环）")

    args = parser.parse_args()

    ids = parse_id_list(args.ids)
    if not ids:
        raise ValueError("从站ID列表为空")

    requests = [build_request(slave_id) for slave_id in ids]

    print(
        f"串口: {args.port} | 波特率: {args.baudrate} | 从站ID: {ids} | "
        f"寄存器: 0x{START_ADDR:04X} 数量: {QUANTITY} | "
        f"频率: {'最高' if args.freq == 0 else f'{args.freq}Hz'} | "
        f"次数: {'无限' if args.count == 0 else args.count}"
    )
    print("按 Ctrl+C 退出\n")

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=args.timeout,
        )

        interval = 1.0 / args.freq if args.freq > 0 else 0
        cycle_count = 0
        success_count = 0
        fail_count = 0
        total_time = 0.0
        acc_values = [(0.0, 0.0, 0.0) for _ in ids]
        quat_values = [(0.0, 0.0, 0.0, 0.0) for _ in ids]
        sensor_status = [False for _ in ids]

        try:
            while args.count == 0 or cycle_count < args.count:
                cycle_start = time.perf_counter()

                for index, (slave_id, request) in enumerate(zip(ids, requests)):
                    ser.write(request)
                    ser.flush()
                    response = ser.read(RESPONSE_LEN)

                    result = parse_response(response, slave_id)
                    if result is not None:
                        acc_values[index], quat_values[index] = result
                        sensor_status[index] = True
                        success_count += 1
                    else:
                        sensor_status[index] = False
                        fail_count += 1
                        if ser.in_waiting:
                            ser.read(ser.in_waiting)

                cycle_count += 1
                cycle_time = time.perf_counter() - cycle_start
                total_time += cycle_time

                all_ok = all(sensor_status)
                status_parts = []
                for idx, slave_id in enumerate(ids):
                    if sensor_status[idx]:
                        accx_ms2, accy_ms2, accz_ms2 = acc_values[idx]
                        qw, qx, qy, qz = quat_values[idx]
                        status_parts.append(
                            f"ID{slave_id} "
                            f"ACC:({accx_ms2:7.3f},{accy_ms2:7.3f},{accz_ms2:7.3f})m/s² "
                            f"QUAT:({qw:6.4f},{qx:6.4f},{qy:6.4f},{qz:6.4f})"
                        )
                    else:
                        status_parts.append(f"ID{slave_id}: FAIL")
                status_line = " | ".join(status_parts)
                print(f"{'OK' if all_ok else 'ERR'} | {status_line}")

                if args.freq > 0:
                    sleep_time = interval - cycle_time
                    if sleep_time > 0:
                        time.sleep(sleep_time)

        except KeyboardInterrupt:
            pass

        if cycle_count > 0:
            avg_time = (total_time / cycle_count) * 1000
            max_freq = 1000 / avg_time if avg_time > 0 else 0
            total_responses = cycle_count * len(ids)
            success_rate = (success_count / total_responses * 100) if total_responses > 0 else 0
            print(
                f"\n统计: 循环={cycle_count} 响应总数={total_responses} "
                f"成功={success_count} 失败={fail_count} "
                f"成功率={success_rate:.1f}% 平均周期={avg_time:.2f}ms 实际频率={max_freq:.1f}Hz"
            )

        ser.close()

    except Exception as exc:
        print(f"错误: {exc}")


if __name__ == "__main__":
    main()
