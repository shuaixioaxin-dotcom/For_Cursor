#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Modbus RTU 批量采集测试工具（多从站轮询）"""

import argparse
import struct
import time
from typing import List, Optional, Tuple

import serial

# Modbus RTU CRC16查表
CRC16_TABLE = [
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
]

# 预计算常量
ACC_SCALE = 0.0048828  # 加速度转换系数
QUAT_SCALE = 0.0001    # 四元数转换系数
QUAT_OFFSET = 41       # 四元数数据偏移（3 + 38）
START_ADDR = 0x0034
QUANTITY = 0x0016
RESPONSE_LEN = 49      # 读0x16个寄存器的响应长度


def crc16_modbus(data: bytes) -> int:
    """计算Modbus RTU CRC16校验值"""
    crc = 0xFFFF
    for byte in data:
        crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ byte) & 0xFF]
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


def read_response(ser: serial.Serial, expected_len: int, timeout_s: float) -> bytes:
    deadline = time.perf_counter() + timeout_s
    buffer = bytearray()
    while len(buffer) < expected_len and time.perf_counter() < deadline:
        chunk = ser.read(expected_len - len(buffer))
        if chunk:
            buffer.extend(chunk)
        else:
            time.sleep(0.0005)
    return bytes(buffer)


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
            timeout=0,
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
                    ser.reset_input_buffer()
                    ser.write(request)
                    ser.flush()
                    response = read_response(ser, RESPONSE_LEN, args.timeout)

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
