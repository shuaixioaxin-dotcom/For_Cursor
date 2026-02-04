#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""优化的IMU读取器 - 调试版本（带详细日志）"""

import serial
import struct
import time
import threading
from typing import Dict, List, Tuple, Optional


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
ACC_SCALE = 0.0048828
QUAT_SCALE = 0.0001
QUAT_OFFSET = 41
RESPONSE_LEN = 49


class OptimizedIMUReaderDebug:
    """优化的IMU读取器 - 调试版本"""
    
    def __init__(self, port: str, baudrate: int, slave_ids: List[int], debug: bool = False):
        self.port = port
        self.baudrate = baudrate
        self.slave_ids = slave_ids
        self.debug = debug
        self.connection: Optional[serial.Serial] = None
        self.lock = threading.Lock()
        
        # 存储每个从站的IMU数据
        self.imu_data: Dict[int, Dict] = {}
        for slave_id in slave_ids:
            self.imu_data[slave_id] = {
                'acc': (0.0, 0.0, 0.0),
                'quat': (0.0, 0.0, 0.0, 0.0)
            }
        
        # 调试统计
        self.stats = {
            'total_reads': 0,
            'successful_frames': {},
            'failed_crc': {},
            'missing_frames': {},
        }
        for slave_id in slave_ids:
            self.stats['successful_frames'][slave_id] = 0
            self.stats['failed_crc'][slave_id] = 0
            self.stats['missing_frames'][slave_id] = 0
    
    def crc16_modbus(self, data: bytes) -> int:
        """计算Modbus RTU CRC16校验值"""
        crc = 0xFFFF
        for byte in data:
            crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ byte) & 0xFF]
        return crc & 0xFFFF
    
    def connect(self) -> bool:
        """连接串口"""
        try:
            if self.connection and self.connection.is_open:
                return True
                
            self.connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01
            )
            
            print(f"✅ 已连接到串口 {self.port}，波特率 {self.baudrate}")
            return True
            
        except Exception as e:
            print(f"❌ 连接串口失败: {e}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        if self.connection and self.connection.is_open:
            self.connection.close()
            print("已断开串口连接")
    
    def read_all_imu_optimized(self) -> Dict[int, Dict]:
        """优化的批量读取所有IMU数据"""
        if not self.connection or not self.connection.is_open:
            if not self.connect():
                return self.imu_data.copy()
        
        with self.lock:
            try:
                self.stats['total_reads'] += 1
                
                # 第一阶段：快速发送所有请求
                requests = []
                for slave_id in self.slave_ids:
                    frame = bytes([slave_id, 0x03, 0x00, 0x34, 0x00, 0x16])
                    crc = self.crc16_modbus(frame)
                    frame += bytes([crc & 0xFF, (crc >> 8) & 0xFF])
                    requests.append((slave_id, frame))
                    
                    if self.debug and self.stats['total_reads'] <= 3:
                        print(f"  发送请求 ID={slave_id}: {frame.hex()}")
                
                # 批量发送请求
                self.connection.reset_input_buffer()
                for slave_id, frame in requests:
                    self.connection.write(frame)
                    time.sleep(0.001)
                
                # 第二阶段：批量读取响应
                time.sleep(0.005)  # 增加等待时间，确保所有设备响应
                
                # 读取所有可用数据
                total_response = b''
                start_time = time.time()
                timeout = 0.02  # 增加超时时间
                
                while time.time() - start_time < timeout:
                    if self.connection.in_waiting > 0:
                        data = self.connection.read(self.connection.in_waiting)
                        total_response += data
                        if self.debug and self.stats['total_reads'] <= 3:
                            print(f"  读取数据块: {len(data)} 字节")
                    time.sleep(0.001)
                
                if self.debug and self.stats['total_reads'] <= 3:
                    print(f"  总接收数据: {len(total_response)} 字节")
                    print(f"  数据内容: {total_response.hex()}")
                
                # 第三阶段：解析响应
                self._parse_responses(total_response, requests)
                
            except Exception as e:
                print(f"❌ 读取IMU数据错误: {e}")
                self.disconnect()
                self.connect()
        
        return self.imu_data.copy()
    
    def _parse_responses(self, response_data: bytes, requests: List[Tuple]):
        """解析响应数据 - 调试版本"""
        # 记录本次读取中哪些设备有响应
        responded_ids = set()
        
        # 查找响应帧起始位置
        response_frames = []
        current_pos = 0
        
        if self.debug and self.stats['total_reads'] <= 3:
            print(f"\n  === 开始解析响应 ===")
        
        while current_pos < len(response_data):
            slave_ids = [slave_id for slave_id, _ in requests]
            
            if current_pos < len(response_data) and response_data[current_pos] in slave_ids:
                slave_id = response_data[current_pos]
                
                if self.debug and self.stats['total_reads'] <= 3:
                    print(f"  位置 {current_pos}: 发现从站地址 {slave_id}")
                
                # 检查是否有完整的响应帧
                if current_pos + RESPONSE_LEN <= len(response_data):
                    frame = response_data[current_pos:current_pos + RESPONSE_LEN]
                    
                    # 验证功能码
                    if len(frame) == RESPONSE_LEN and frame[1] == 0x03:
                        # 验证CRC
                        crc_calculated = self.crc16_modbus(frame[:RESPONSE_LEN-2])
                        crc_received = frame[RESPONSE_LEN-2] | (frame[RESPONSE_LEN-1] << 8)
                        
                        if self.debug and self.stats['total_reads'] <= 3:
                            print(f"    功能码: 0x{frame[1]:02x}")
                            print(f"    CRC计算: 0x{crc_calculated:04x}, CRC接收: 0x{crc_received:04x}")
                        
                        if crc_calculated == crc_received:
                            response_frames.append(frame)
                            responded_ids.add(slave_id)
                            if self.debug and self.stats['total_reads'] <= 3:
                                print(f"    ✅ CRC校验通过")
                        else:
                            self.stats['failed_crc'][slave_id] += 1
                            if self.debug and self.stats['total_reads'] <= 3:
                                print(f"    ❌ CRC校验失败")
                        
                        current_pos += RESPONSE_LEN
                    else:
                        if self.debug and self.stats['total_reads'] <= 3:
                            print(f"    功能码不匹配: 0x{frame[1]:02x}")
                        current_pos += 1
                else:
                    if self.debug and self.stats['total_reads'] <= 3:
                        print(f"  位置 {current_pos}: 数据不足 (剩余 {len(response_data) - current_pos} 字节)")
                    break
            else:
                current_pos += 1
        
        # 记录缺失的设备
        for slave_id in self.slave_ids:
            if slave_id not in responded_ids:
                self.stats['missing_frames'][slave_id] += 1
                if self.debug and self.stats['total_reads'] <= 3:
                    print(f"  ⚠️ 从站 {slave_id} 没有响应")
        
        if self.debug and self.stats['total_reads'] <= 3:
            print(f"  找到 {len(response_frames)} 个有效响应帧\n")
        
        # 解析每个响应帧
        for frame in response_frames:
            slave_id = frame[0]
            if slave_id in self.slave_ids:
                self.stats['successful_frames'][slave_id] += 1
                
                # 解析加速度
                accx_raw, accy_raw, accz_raw = struct.unpack_from('>3H', frame, 3)
                accx_ms2 = (accx_raw if accx_raw < 32768 else accx_raw - 65536) * ACC_SCALE
                accy_ms2 = (accy_raw if accy_raw < 32768 else accy_raw - 65536) * ACC_SCALE
                accz_ms2 = (accz_raw if accz_raw < 32768 else accz_raw - 65536) * ACC_SCALE
                
                # 解析四元数
                qw, qx, qy, qz = struct.unpack_from('>4h', frame, QUAT_OFFSET)
                
                # 更新数据
                self.imu_data[slave_id] = {
                    'acc': (round(accx_ms2, 3), round(accy_ms2, 3), round(accz_ms2, 3)),
                    'quat': (round(qw * QUAT_SCALE, 4), round(qx * QUAT_SCALE, 4), 
                            round(qy * QUAT_SCALE, 4), round(qz * QUAT_SCALE, 4))
                }
                
                if self.debug and self.stats['total_reads'] <= 3:
                    print(f"  ID {slave_id} 数据已更新: ACC={self.imu_data[slave_id]['acc']}")
    
    def print_stats(self):
        """打印统计信息"""
        print("\n" + "="*70)
        print("调试统计信息")
        print("="*70)
        print(f"总读取次数: {self.stats['total_reads']}")
        print("\n各设备统计:")
        for slave_id in self.slave_ids:
            success = self.stats['successful_frames'][slave_id]
            failed_crc = self.stats['failed_crc'][slave_id]
            missing = self.stats['missing_frames'][slave_id]
            total = self.stats['total_reads']
            success_rate = (success / total * 100) if total > 0 else 0
            
            print(f"\n  从站 ID {slave_id}:")
            print(f"    成功帧: {success}/{total} ({success_rate:.1f}%)")
            print(f"    CRC失败: {failed_crc}")
            print(f"    缺失帧: {missing}")
        print("="*70)


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='优化的IMU读取器 - 调试版本')
    parser.add_argument('-p', '--port', type=str, default='COM66', help='串口端口')
    parser.add_argument('-b', '--baudrate', type=int, default=921600, help='波特率')
    parser.add_argument('-i', '--ids', type=str, default='1,2', help='从站ID列表')
    parser.add_argument('-c', '--count', type=int, default=0, help='读取次数（0=无限）')
    parser.add_argument('-d', '--debug', action='store_true', help='启用调试输出')
    
    args = parser.parse_args()
    
    slave_ids = [int(id_str.strip()) for id_str in args.ids.split(',')]
    
    print(f"串口: {args.port} | 波特率: {args.baudrate} | 从站ID: {slave_ids}")
    print(f"调试模式: {'开启' if args.debug else '关闭'}")
    print("按 Ctrl+C 退出\n")
    
    reader = OptimizedIMUReaderDebug(args.port, args.baudrate, slave_ids, debug=args.debug)
    
    if not reader.connect():
        return
    
    try:
        count = 0
        while args.count == 0 or count < args.count:
            imu_data = reader.read_all_imu_optimized()
            
            # 显示数据
            for slave_id in slave_ids:
                data = imu_data[slave_id]
                acc = data['acc']
                quat = data['quat']
                print(f"ID:{slave_id:2d} | ACC:({acc[0]:7.3f},{acc[1]:7.3f},{acc[2]:7.3f})m/s² | "
                      f"QUAT:({quat[0]:6.4f},{quat[1]:6.4f},{quat[2]:6.4f},{quat[3]:6.4f})")
            
            count += 1
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        reader.print_stats()
        reader.disconnect()


if __name__ == "__main__":
    main()
