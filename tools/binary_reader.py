#!/usr/bin/env python3
"""
IMU二进制数据接收器
用于接收和解析ESP32发送的二进制格式IMU数据
"""

import serial
import struct
import time
import sys
import argparse
from datetime import datetime

class IMUBinaryReader:
    def __init__(self, port, baudrate=921600, num_imus=2):
        """
        初始化IMU数据接收器
        
        Args:
            port: 串口设备路径 (例如: /dev/ttyUSB0 或 COM3)
            baudrate: 波特率
            num_imus: IMU设备数量
        """
        self.port = port
        self.baudrate = baudrate
        self.num_imus = num_imus
        self.ser = None
        
        # 统计信息
        self.frame_count = 0
        self.error_count = 0
        self.start_time = time.time()
        
    def connect(self):
        """连接串口"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            print(f"✓ 已连接到 {self.port} @ {self.baudrate} baud")
            time.sleep(2)  # 等待Arduino重启
            return True
        except serial.SerialException as e:
            print(f"✗ 连接失败: {e}")
            return False
    
    def find_frame_header(self):
        """查找帧头 0xFF 0xAA"""
        while True:
            byte1 = self.ser.read(1)
            if not byte1:
                continue
            
            if byte1[0] == 0xFF:
                byte2 = self.ser.read(1)
                if byte2 and byte2[0] == 0xAA:
                    return True
    
    def read_frame(self):
        """
        读取一帧数据
        
        Returns:
            list of dict: 每个IMU的数据，格式为:
                {
                    'imu_id': int,
                    'acc': [ax, ay, az],
                    'quat': [qw, qx, qy, qz],
                    'valid': bool
                }
            None: 读取失败
        """
        try:
            # 查找帧头
            if not self.find_frame_header():
                return None
            
            # 读取批次数量
            count_byte = self.ser.read(1)
            if not count_byte:
                return None
            batch_count = count_byte[0]
            
            if batch_count == 0 or batch_count > 20:
                self.error_count += 1
                return None
            
            # 读取所有样本
            all_samples = []
            for sample_idx in range(batch_count):
                sample_data = []
                for imu_id in range(self.num_imus):
                    # 读取7个float: ax,ay,az,qw,qx,qy,qz
                    data = self.ser.read(28)  # 7 * 4 bytes
                    if len(data) != 28:
                        self.error_count += 1
                        return None
                    
                    values = struct.unpack('7f', data)
                    ax, ay, az, qw, qx, qy, qz = values
                    
                    # 检查数据有效性（全零表示无效）
                    valid = not (ax == 0 and ay == 0 and az == 0 and 
                                qw == 0 and qx == 0 and qy == 0 and qz == 0)
                    
                    sample_data.append({
                        'imu_id': imu_id + 1,
                        'acc': [ax, ay, az],
                        'quat': [qw, qx, qy, qz],
                        'valid': valid
                    })
                
                all_samples.append(sample_data)
            
            # 验证帧尾
            footer = self.ser.read(2)
            if footer != b'\xbb\xcc':
                self.error_count += 1
                return None
            
            self.frame_count += 1
            return all_samples
            
        except Exception as e:
            print(f"✗ 读取错误: {e}")
            self.error_count += 1
            return None
    
    def print_data(self, samples, verbose=True):
        """
        打印IMU数据
        
        Args:
            samples: read_frame()返回的数据
            verbose: 是否详细输出
        """
        if not samples:
            return
        
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        if verbose:
            print(f"\n{'='*60}")
            print(f"时间: {timestamp} | 批次: {len(samples)}组")
            print(f"{'='*60}")
            
            for idx, sample in enumerate(samples):
                print(f"\n样本 #{idx+1}:")
                for imu_data in sample:
                    imu_id = imu_data['imu_id']
                    valid = "✓" if imu_data['valid'] else "✗"
                    ax, ay, az = imu_data['acc']
                    qw, qx, qy, qz = imu_data['quat']
                    
                    print(f"  IMU{imu_id} {valid}")
                    print(f"    加速度: ({ax:7.3f}, {ay:7.3f}, {az:7.3f}) m/s²")
                    print(f"    四元数: ({qw:7.4f}, {qx:7.4f}, {qy:7.4f}, {qz:7.4f})")
        else:
            # 简洁模式：仅显示最后一个样本
            last_sample = samples[-1]
            print(f"[{timestamp}] ", end='')
            for imu_data in last_sample:
                imu_id = imu_data['imu_id']
                valid = "✓" if imu_data['valid'] else "✗"
                ax, ay, az = imu_data['acc']
                print(f"IMU{imu_id}{valid}:({ax:6.2f},{ay:6.2f},{az:6.2f}) ", end='')
            print(f"| 批次:{len(samples)}")
    
    def print_stats(self):
        """打印统计信息"""
        elapsed = time.time() - self.start_time
        if elapsed > 0:
            fps = self.frame_count / elapsed
            error_rate = (self.error_count / max(1, self.frame_count + self.error_count)) * 100
            print(f"\n{'='*60}")
            print(f"统计信息:")
            print(f"  运行时间: {elapsed:.1f}秒")
            print(f"  接收帧数: {self.frame_count}")
            print(f"  错误次数: {self.error_count}")
            print(f"  帧率: {fps:.1f} FPS")
            print(f"  错误率: {error_rate:.2f}%")
            print(f"{'='*60}")
    
    def run(self, verbose=True, max_frames=None):
        """
        运行数据接收循环
        
        Args:
            verbose: 是否详细输出
            max_frames: 最大接收帧数（None=无限）
        """
        if not self.connect():
            return
        
        print(f"\n开始接收数据... (按Ctrl+C停止)\n")
        
        try:
            while True:
                if max_frames and self.frame_count >= max_frames:
                    break
                
                samples = self.read_frame()
                if samples:
                    self.print_data(samples, verbose)
                
                # 每100帧显示一次统计
                if self.frame_count % 100 == 0 and self.frame_count > 0:
                    self.print_stats()
        
        except KeyboardInterrupt:
            print("\n\n收到停止信号...")
        
        finally:
            self.print_stats()
            if self.ser:
                self.ser.close()
            print("✓ 已断开连接")

def main():
    parser = argparse.ArgumentParser(
        description='IMU二进制数据接收器',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # Windows
  python binary_reader.py COM3
  
  # Linux/Mac
  python binary_reader.py /dev/ttyUSB0
  
  # 简洁模式
  python binary_reader.py /dev/ttyUSB0 --quiet
  
  # 接收100帧后停止
  python binary_reader.py /dev/ttyUSB0 --max-frames 100
        """
    )
    
    parser.add_argument('port', help='串口设备路径 (例如: COM3 或 /dev/ttyUSB0)')
    parser.add_argument('-b', '--baudrate', type=int, default=921600,
                       help='波特率 (默认: 921600)')
    parser.add_argument('-n', '--num-imus', type=int, default=2,
                       help='IMU数量 (默认: 2)')
    parser.add_argument('-q', '--quiet', action='store_true',
                       help='简洁输出模式')
    parser.add_argument('-m', '--max-frames', type=int, default=None,
                       help='最大接收帧数')
    
    args = parser.parse_args()
    
    reader = IMUBinaryReader(args.port, args.baudrate, args.num_imus)
    reader.run(verbose=not args.quiet, max_frames=args.max_frames)

if __name__ == '__main__':
    main()
