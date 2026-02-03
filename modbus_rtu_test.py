#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Modbus RTU 帧解析测试工具 - 多从站循环读取版本"""

import serial
import struct
import time
import argparse
from typing import Optional, Tuple

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
    """构建Modbus请求帧"""
    request_data = bytes([slave_id, 0x03, 0x00, 0x34, 0x00, 0x16])
    crc = crc16_modbus(request_data)
    return request_data + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


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


def main():
    parser = argparse.ArgumentParser(description='Modbus RTU 帧解析测试工具 - 多从站循环读取')
    parser.add_argument('-p', '--port', type=str, default='COM35', help='串口端口（默认COM35）')
    parser.add_argument('-b', '--baudrate', type=int, default=921600, help='波特率（默认921600）')
    parser.add_argument('-s', '--slaves', type=str, default='1,2', help='从站ID列表，逗号分隔（默认1,2）')
    parser.add_argument('-f', '--freq', type=float, default=0, help='读取频率Hz（0=最高频率）')
    parser.add_argument('-c', '--count', type=int, default=0, help='每个从站读取次数（0=无限循环）')
    
    args = parser.parse_args()
    
    # 解析从站ID列表
    slave_ids = [int(x.strip()) for x in args.slaves.split(',')]
    
    # 为每个从站预构建请求帧
    requests = {slave_id: build_request(slave_id) for slave_id in slave_ids}
    
    print(f"串口: {args.port} | 波特率: {args.baudrate} | 从站ID: {slave_ids} | "
          f"频率: {'最高' if args.freq == 0 else f'{args.freq}Hz'} | "
          f"每个从站次数: {'无限' if args.count == 0 else args.count}")
    print("按 Ctrl+C 退出\n")
    
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.01  # 减少超时时间以提高频率
        )
        
        interval = 1.0 / args.freq if args.freq > 0 else 0
        
        # 统计信息（每个从站独立统计）
        stats = {slave_id: {'count': 0, 'success': 0, 'fail': 0, 'total_time': 0.0} 
                 for slave_id in slave_ids}
        
        try:
            current_slave_idx = 0
            loop_count = 0
            
            while True:
                # 获取当前从站ID
                slave_id = slave_ids[current_slave_idx]
                
                # 检查是否达到指定次数
                if args.count > 0 and stats[slave_id]['count'] >= args.count:
                    # 检查是否所有从站都完成
                    if all(stats[sid]['count'] >= args.count for sid in slave_ids):
                        break
                    # 切换到下一个从站
                    current_slave_idx = (current_slave_idx + 1) % len(slave_ids)
                    continue
                
                cycle_start = time.perf_counter()
                
                # 发送请求
                ser.write(requests[slave_id])
                ser.flush()
                response = ser.read(RESPONSE_LEN)
                
                # 解析响应
                if len(response) >= RESPONSE_LEN:
                    result = parse_response(response)
                    if result:
                        (accx_ms2, accy_ms2, accz_ms2), (qw, qx, qy, qz) = result
                        print(f"[从站 0x{slave_id:02X}] ACC:({accx_ms2:7.3f},{accy_ms2:7.3f},{accz_ms2:7.3f})m/s² | "
                              f"QUAT:({qw:6.4f},{qx:6.4f},{qy:6.4f},{qz:6.4f})")
                        stats[slave_id]['success'] += 1
                    else:
                        print(f"[从站 0x{slave_id:02X}] 解析失败")
                        stats[slave_id]['fail'] += 1
                else:
                    print(f"[从站 0x{slave_id:02X}] 响应超时或不完整")
                    stats[slave_id]['fail'] += 1
                
                stats[slave_id]['count'] += 1
                cycle_time = time.perf_counter() - cycle_start
                stats[slave_id]['total_time'] += cycle_time
                
                # 切换到下一个从站
                current_slave_idx = (current_slave_idx + 1) % len(slave_ids)
                
                # 频率控制
                if args.freq > 0:
                    sleep_time = interval - cycle_time
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                        
        except KeyboardInterrupt:
            print("\n\n用户中断")
        
        # 打印统计信息
        print("\n" + "="*80)
        print("统计信息:")
        print("="*80)
        
        for slave_id in slave_ids:
            stat = stats[slave_id]
            count = stat['count']
            success = stat['success']
            fail = stat['fail']
            total_time = stat['total_time']
            
            if count > 0:
                avg_time = total_time / count * 1000
                max_freq = 1000 / avg_time if avg_time > 0 else 0
                success_rate = success / count * 100
                
                print(f"从站 0x{slave_id:02X}: 总数={count:5d} 成功={success:5d} 失败={fail:5d} "
                      f"成功率={success_rate:5.1f}% 平均耗时={avg_time:6.2f}ms 实际频率={max_freq:6.1f}Hz")
        
        # 总体统计
        total_count = sum(s['count'] for s in stats.values())
        total_success = sum(s['success'] for s in stats.values())
        total_fail = sum(s['fail'] for s in stats.values())
        total_time_all = sum(s['total_time'] for s in stats.values())
        
        if total_count > 0:
            print("-"*80)
            avg_time_all = total_time_all / total_count * 1000
            max_freq_all = 1000 / avg_time_all if avg_time_all > 0 else 0
            success_rate_all = total_success / total_count * 100
            
            print(f"总体统计: 总数={total_count:5d} 成功={total_success:5d} 失败={total_fail:5d} "
                  f"成功率={success_rate_all:5.1f}% 平均耗时={avg_time_all:6.2f}ms 实际频率={max_freq_all:6.1f}Hz")
        
        print("="*80)
        
        ser.close()
        
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
