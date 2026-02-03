#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Modbus RTU 帧解析测试工具 - 支持多从站循环读取"""

import serial
import struct
import time
import argparse
from typing import Optional, Tuple, List, Dict

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
RESPONSE_LEN = 49      # 响应长度


def crc16_modbus(data: bytes) -> int:
    """计算Modbus RTU CRC16校验值"""
    crc = 0xFFFF
    for byte in data:
        crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ byte) & 0xFF]
    return crc & 0xFFFF


def build_request(slave_id: int) -> bytes:
    """构建Modbus RTU请求帧"""
    request_data = bytes([slave_id, 0x03, 0x00, 0x34, 0x00, 0x16])
    crc = crc16_modbus(request_data)
    return request_data + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def verify_crc(data: bytes) -> bool:
    """验证Modbus RTU帧的CRC"""
    if len(data) < 3:
        return False
    # 数据部分（不含CRC）
    payload = data[:-2]
    # 接收到的CRC（小端序）
    received_crc = data[-2] | (data[-1] << 8)
    # 计算CRC
    calculated_crc = crc16_modbus(payload)
    return received_crc == calculated_crc


def read_modbus_response(ser: serial.Serial, expected_slave_id: int, 
                          expected_len: int, timeout_ms: float) -> Tuple[bytes, str]:
    """
    智能读取Modbus响应帧
    
    返回: (响应数据, 错误信息)
    - 成功: (完整帧, "")
    - 失败: (已读数据, 错误原因)
    """
    buffer = bytearray()
    start_time = time.perf_counter()
    timeout_sec = timeout_ms / 1000.0
    
    while (time.perf_counter() - start_time) < timeout_sec:
        # 检查可用数据
        available = ser.in_waiting
        if available > 0:
            chunk = ser.read(available)
            buffer.extend(chunk)
            
            # 检查是否收到足够数据
            if len(buffer) >= expected_len:
                break
        else:
            # 短暂等待，避免CPU空转
            time.sleep(0.0001)  # 100μs
    
    # 数据不足
    if len(buffer) < expected_len:
        return bytes(buffer), f"数据不足({len(buffer)}/{expected_len}字节)"
    
    # 取前 expected_len 字节作为响应帧
    response = bytes(buffer[:expected_len])
    
    # 验证从站ID
    if response[0] != expected_slave_id:
        return response, f"从站ID不匹配(期望0x{expected_slave_id:02X},收到0x{response[0]:02X})"
    
    # 验证功能码（0x03 或 异常响应 0x83）
    if response[1] == 0x83:
        return response, f"从站异常响应(错误码:0x{response[2]:02X})"
    if response[1] != 0x03:
        return response, f"功能码错误(期望0x03,收到0x{response[1]:02X})"
    
    # 验证CRC
    if not verify_crc(response):
        return response, "CRC校验失败"
    
    return response, ""


def parse_response(response: bytes) -> Optional[Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]]:
    """解析Modbus响应帧，返回加速度和四元数"""
    if len(response) < RESPONSE_LEN:
        return None
    
    # 解析加速度（偏移3-8，3个16位无符号整数）
    accx_raw, accy_raw, accz_raw = struct.unpack_from('>3H', response, 3)
    
    # 转换为有符号并计算m/s²
    accx_ms2 = (accx_raw if accx_raw < 32768 else accx_raw - 65536) * ACC_SCALE
    accy_ms2 = (accy_raw if accy_raw < 32768 else accy_raw - 65536) * ACC_SCALE
    accz_ms2 = (accz_raw if accz_raw < 32768 else accz_raw - 65536) * ACC_SCALE
    
    # 解析四元数（偏移41-48，4个16位有符号整数）
    qw, qx, qy, qz = struct.unpack_from('>4h', response, QUAT_OFFSET)
    
    # 转换为浮点数
    return ((accx_ms2, accy_ms2, accz_ms2), (qw * QUAT_SCALE, qx * QUAT_SCALE, qy * QUAT_SCALE, qz * QUAT_SCALE))


def parse_slave_ids(ids_str: str) -> List[int]:
    """解析从站ID字符串，支持逗号分隔和范围表示
    
    例如: "1,2" -> [1, 2]
          "1-3" -> [1, 2, 3]
          "1,3-5" -> [1, 3, 4, 5]
    """
    slave_ids = []
    parts = ids_str.split(',')
    for part in parts:
        part = part.strip()
        if '-' in part:
            start, end = part.split('-', 1)
            start = int(start.strip(), 0)  # 支持0x前缀
            end = int(end.strip(), 0)
            slave_ids.extend(range(start, end + 1))
        else:
            slave_ids.append(int(part.strip(), 0))
    return slave_ids


def main():
    parser = argparse.ArgumentParser(description='Modbus RTU 帧解析测试工具 - 支持多从站循环读取')
    parser.add_argument('-p', '--port', type=str, default='COM66', help='串口端口（默认COM66）')
    parser.add_argument('-b', '--baudrate', type=int, default=921600, help='波特率（默认921600）')
    parser.add_argument('-i', '--ids', type=str, default='1,2', 
                        help='从站ID列表，支持逗号分隔和范围（默认"1,2"，例如"1,2"或"1-3"或"0x01,0x02"）')
    parser.add_argument('-f', '--freq', type=float, default=0, help='读取频率Hz（0=最高频率）')
    parser.add_argument('-c', '--count', type=int, default=0, help='读取循环次数（0=无限循环，每次循环读取所有从站）')
    parser.add_argument('-d', '--delay', type=float, default=0, help='从站间延时ms（默认0ms，按需设置）')
    parser.add_argument('-t', '--timeout', type=float, default=5, help='响应超时ms（默认5ms）')
    parser.add_argument('-w', '--tx-wait', type=float, default=0.5, help='发送后等待ms，让驱动器释放总线（默认0.5ms）')
    parser.add_argument('-v', '--verbose', action='store_true', help='显示详细错误信息和丢弃的数据')
    
    args = parser.parse_args()
    
    # 解析从站ID列表
    slave_ids = parse_slave_ids(args.ids)
    if not slave_ids:
        print("错误: 未指定有效的从站ID")
        return
    
    # 预构建所有从站的请求帧
    requests = {slave_id: build_request(slave_id) for slave_id in slave_ids}
    
    slave_ids_str = ','.join([f'0x{sid:02X}' for sid in slave_ids])
    print(f"串口: {args.port} | 波特率: {args.baudrate} | 从站ID: [{slave_ids_str}]")
    print(f"响应超时: {args.timeout}ms | 发送后等待: {args.tx_wait}ms | 从站间延时: {args.delay}ms")
    print(f"频率: {'最高' if args.freq == 0 else f'{args.freq}Hz'} | "
          f"次数: {'无限' if args.count == 0 else args.count}")
    print("按 Ctrl+C 退出\n")
    
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0  # 非阻塞模式，由 read_modbus_response 控制超时
        )
        
        interval = 1.0 / args.freq if args.freq > 0 else 0
        slave_delay = args.delay / 1000.0  # 从站间延时（转换为秒）
        tx_wait = args.tx_wait / 1000.0    # 发送后等待时间（转换为秒）
        
        # 计算请求帧传输时间 (8字节 * 10位/字节 / 波特率)
        # 10位 = 1起始位 + 8数据位 + 1停止位
        tx_time = 8 * 10 / args.baudrate
        
        cycle_count = 0
        
        # 每个从站的统计信息
        stats: Dict[int, Dict] = {
            slave_id: {
                'success': 0, 
                'fail': 0, 
                'total_time': 0.0,
                'errors': {}  # 错误类型统计
            } for slave_id in slave_ids
        }
        total_requests = 0
        
        try:
            while args.count == 0 or cycle_count < args.count:
                cycle_start = time.perf_counter()
                
                # 循环读取所有从站
                for slave_id in slave_ids:
                    request_start = time.perf_counter()
                    
                    # 清空接收缓冲区（丢弃残留数据）
                    if ser.in_waiting > 0:
                        discarded = ser.read(ser.in_waiting)
                        if args.verbose:
                            print(f"[ID:0x{slave_id:02X}] 丢弃残留数据: {discarded.hex()}")
                    
                    # 发送请求
                    ser.write(requests[slave_id])
                    ser.flush()  # 确保数据写入发送缓冲区
                    
                    # 等待发送完成 + 驱动器释放总线
                    # 这对于 RS485 半双工通信非常关键！
                    if tx_wait > 0:
                        time.sleep(tx_wait)
                    
                    # 智能读取响应
                    response, error = read_modbus_response(ser, slave_id, RESPONSE_LEN, args.timeout)
                    
                    request_time = time.perf_counter() - request_start
                    stats[slave_id]['total_time'] += request_time
                    total_requests += 1
                    
                    if error:
                        # 记录错误类型
                        error_key = error.split('(')[0].strip()
                        stats[slave_id]['errors'][error_key] = stats[slave_id]['errors'].get(error_key, 0) + 1
                        stats[slave_id]['fail'] += 1
                        if args.verbose:
                            print(f"[ID:0x{slave_id:02X}] 错误: {error} | 原始数据: {response.hex() if response else 'N/A'}")
                    else:
                        result = parse_response(response)
                        if result:
                            (accx_ms2, accy_ms2, accz_ms2), (qw, qx, qy, qz) = result
                            print(f"[ID:0x{slave_id:02X}] ACC:({accx_ms2:7.3f},{accy_ms2:7.3f},{accz_ms2:7.3f})m/s² | "
                                  f"QUAT:({qw:6.4f},{qx:6.4f},{qy:6.4f},{qz:6.4f}) | {request_time*1000:.2f}ms")
                            stats[slave_id]['success'] += 1
                        else:
                            stats[slave_id]['fail'] += 1
                            stats[slave_id]['errors']['解析失败'] = stats[slave_id]['errors'].get('解析失败', 0) + 1
                    
                    # 从站间延时，确保总线静默时间
                    if slave_delay > 0:
                        time.sleep(slave_delay)
                
                cycle_count += 1
                cycle_time = time.perf_counter() - cycle_start
                
                if args.freq > 0:
                    sleep_time = interval - cycle_time
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                        
        except KeyboardInterrupt:
            pass
        
        # 统计信息
        print("\n" + "=" * 80)
        print("统计信息:")
        print("=" * 80)
        
        total_success = 0
        total_fail = 0
        
        for slave_id in slave_ids:
            s = stats[slave_id]
            total = s['success'] + s['fail']
            total_success += s['success']
            total_fail += s['fail']
            
            if total > 0:
                avg_time = s['total_time'] / total * 1000
                success_rate = s['success'] / total * 100
                max_freq = 1000 / avg_time if avg_time > 0 else 0
                print(f"[ID:0x{slave_id:02X}] 总数={total} 成功={s['success']} 失败={s['fail']} "
                      f"成功率={success_rate:.1f}% 平均耗时={avg_time:.2f}ms 频率={max_freq:.1f}Hz")
                
                # 显示错误类型统计
                if s['errors']:
                    error_str = ', '.join([f"{k}:{v}" for k, v in s['errors'].items()])
                    print(f"         错误分布: {error_str}")
        
        print("-" * 80)
        total_all = total_success + total_fail
        if total_all > 0:
            overall_success_rate = total_success / total_all * 100
            print(f"[总计] 循环次数={cycle_count} 总请求={total_all} 成功={total_success} 失败={total_fail} "
                  f"成功率={overall_success_rate:.1f}%")
        
        ser.close()
        
    except Exception as e:
        print(f"错误: {e}")


if __name__ == "__main__":
    main()
