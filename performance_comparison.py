#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""性能对比：传统循环模式 vs 批处理模式"""

import serial
import struct
import time
import argparse
from typing import List, Dict

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

RESPONSE_LEN = 49


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


def test_sequential_mode(ser: serial.Serial, slave_ids: List[int], count: int) -> Dict:
    """传统循环模式测试"""
    print(f"\n【传统循环模式】每次发送-等待-接收一个从站")
    
    requests = {slave_id: build_request(slave_id) for slave_id in slave_ids}
    success = 0
    fail = 0
    
    start_time = time.perf_counter()
    
    for _ in range(count):
        for slave_id in slave_ids:
            ser.write(requests[slave_id])
            ser.flush()
            response = ser.read(RESPONSE_LEN)
            
            if len(response) >= RESPONSE_LEN and response[0] == slave_id:
                success += 1
            else:
                fail += 1
    
    total_time = time.perf_counter() - start_time
    
    return {
        'mode': '传统循环模式',
        'total': success + fail,
        'success': success,
        'fail': fail,
        'time': total_time,
        'avg_time': total_time / (success + fail) * 1000 if (success + fail) > 0 else 0,
        'freq': (success + fail) / total_time if total_time > 0 else 0
    }


def test_batch_mode(ser: serial.Serial, slave_ids: List[int], count: int, inter_delay: float) -> Dict:
    """批处理模式测试"""
    print(f"\n【批处理模式】批量发送请求，批量接收响应")
    
    requests = {slave_id: build_request(slave_id) for slave_id in slave_ids}
    num_slaves = len(slave_ids)
    expected_bytes = num_slaves * RESPONSE_LEN
    success = 0
    fail = 0
    
    start_time = time.perf_counter()
    
    for _ in range(count):
        # 批量发送
        for slave_id in slave_ids:
            ser.write(requests[slave_id])
            if inter_delay > 0:
                time.sleep(inter_delay)
        ser.flush()
        
        # 批量接收
        all_responses = ser.read(expected_bytes + 100)
        
        # 批量处理
        for slave_id in slave_ids:
            found = False
            for i in range(len(all_responses) - RESPONSE_LEN + 1):
                if all_responses[i] == slave_id and all_responses[i+1] == 0x03:
                    success += 1
                    found = True
                    break
            
            if not found:
                fail += 1
    
    total_time = time.perf_counter() - start_time
    
    return {
        'mode': '批处理模式',
        'total': success + fail,
        'success': success,
        'fail': fail,
        'time': total_time,
        'avg_time': total_time / (success + fail) * 1000 if (success + fail) > 0 else 0,
        'freq': (success + fail) / total_time if total_time > 0 else 0
    }


def print_results(results: List[Dict]):
    """打印对比结果"""
    print("\n" + "="*100)
    print("性能对比结果")
    print("="*100)
    print(f"{'模式':<15} {'总数':<8} {'成功':<8} {'失败':<8} {'总耗时(s)':<12} {'平均耗时(ms)':<15} {'实际频率(Hz)':<15}")
    print("-"*100)
    
    for result in results:
        print(f"{result['mode']:<15} {result['total']:<8} {result['success']:<8} {result['fail']:<8} "
              f"{result['time']:<12.3f} {result['avg_time']:<15.2f} {result['freq']:<15.1f}")
    
    print("="*100)
    
    # 计算提升百分比
    if len(results) == 2:
        seq_time = results[0]['time']
        batch_time = results[1]['time']
        
        if seq_time > 0:
            improvement = (seq_time - batch_time) / seq_time * 100
            speedup = seq_time / batch_time if batch_time > 0 else 0
            
            print(f"\n性能提升: {improvement:.1f}%")
            print(f"加速比: {speedup:.2f}x")
            print(f"批处理模式比传统模式快 {speedup:.2f} 倍")


def main():
    parser = argparse.ArgumentParser(description='Modbus RTU 性能对比工具')
    parser.add_argument('-p', '--port', type=str, default='COM66', help='串口端口')
    parser.add_argument('-b', '--baudrate', type=int, default=921600, help='波特率')
    parser.add_argument('-s', '--slaves', type=str, default='1,2', help='从站ID列表')
    parser.add_argument('-c', '--count', type=int, default=50, help='测试批次数')
    parser.add_argument('-t', '--inter-delay', type=float, default=0.0001, help='从站间延迟')
    
    args = parser.parse_args()
    
    slave_ids = [int(x.strip()) for x in args.slaves.split(',')]
    
    print("="*100)
    print("Modbus RTU 性能对比测试")
    print("="*100)
    print(f"串口: {args.port}")
    print(f"波特率: {args.baudrate}")
    print(f"从站列表: {slave_ids}")
    print(f"测试批次: {args.count} (每批次读取所有从站)")
    print(f"总读取次数: {args.count * len(slave_ids)}")
    print(f"从站间延迟: {args.inter_delay*1000:.2f}ms")
    
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.02
        )
        
        results = []
        
        # 测试1：传统循环模式
        print("\n开始测试传统循环模式...")
        result1 = test_sequential_mode(ser, slave_ids, args.count)
        results.append(result1)
        print(f"完成: 总耗时 {result1['time']:.3f}s, 频率 {result1['freq']:.1f}Hz")
        
        time.sleep(1)  # 短暂休息
        
        # 测试2：批处理模式
        print("\n开始测试批处理模式...")
        result2 = test_batch_mode(ser, slave_ids, args.count, args.inter_delay)
        results.append(result2)
        print(f"完成: 总耗时 {result2['time']:.3f}s, 频率 {result2['freq']:.1f}Hz")
        
        # 打印对比结果
        print_results(results)
        
        ser.close()
        
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
