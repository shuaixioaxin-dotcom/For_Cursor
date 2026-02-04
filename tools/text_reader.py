#!/usr/bin/env python3
"""
IMU文本数据接收器
用于接收和解析ESP32发送的文本格式IMU数据
"""

import serial
import time
import sys
import argparse
from datetime import datetime

class IMUTextReader:
    def __init__(self, port, baudrate=921600, num_imus=2):
        """
        初始化IMU数据接收器
        
        Args:
            port: 串口设备路径
            baudrate: 波特率
            num_imus: IMU设备数量
        """
        self.port = port
        self.baudrate = baudrate
        self.num_imus = num_imus
        self.ser = None
        
        # 统计信息
        self.line_count = 0
        self.error_count = 0
        self.start_time = time.time()
        
    def connect(self):
        """连接串口"""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            print(f"✓ 已连接到 {self.port} @ {self.baudrate} baud")
            time.sleep(2)
            return True
        except serial.SerialException as e:
            print(f"✗ 连接失败: {e}")
            return False
    
    def parse_line(self, line):
        """
        解析一行数据
        
        Args:
            line: CSV格式字符串
        
        Returns:
            list of dict: 每个IMU的数据
        """
        try:
            values = [float(x.strip()) for x in line.split(',')]
            
            # 每个IMU有7个值：ax,ay,az,qw,qx,qy,qz
            expected_count = self.num_imus * 7
            if len(values) != expected_count:
                return None
            
            result = []
            for i in range(self.num_imus):
                offset = i * 7
                imu_data = {
                    'imu_id': i + 1,
                    'acc': values[offset:offset+3],
                    'quat': values[offset+3:offset+7],
                    'valid': not all(v == 0 for v in values[offset:offset+7])
                }
                result.append(imu_data)
            
            return result
            
        except (ValueError, IndexError) as e:
            return None
    
    def print_data(self, imu_data_list, verbose=True):
        """打印IMU数据"""
        if not imu_data_list:
            return
        
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        if verbose:
            print(f"\n[{timestamp}]")
            for imu_data in imu_data_list:
                imu_id = imu_data['imu_id']
                valid = "✓" if imu_data['valid'] else "✗"
                ax, ay, az = imu_data['acc']
                qw, qx, qy, qz = imu_data['quat']
                
                print(f"  IMU{imu_id} {valid} | "
                      f"Acc:({ax:7.3f},{ay:7.3f},{az:7.3f}) | "
                      f"Quat:({qw:7.4f},{qx:7.4f},{qy:7.4f},{qz:7.4f})")
        else:
            # 简洁模式
            print(f"[{timestamp}] ", end='')
            for imu_data in imu_data_list:
                imu_id = imu_data['imu_id']
                valid = "✓" if imu_data['valid'] else "✗"
                ax, ay, az = imu_data['acc']
                print(f"IMU{imu_id}{valid}:({ax:6.2f},{ay:6.2f},{az:6.2f}) ", end='')
            print()
    
    def print_stats(self):
        """打印统计信息"""
        elapsed = time.time() - self.start_time
        if elapsed > 0:
            lps = self.line_count / elapsed
            error_rate = (self.error_count / max(1, self.line_count + self.error_count)) * 100
            print(f"\n{'='*60}")
            print(f"统计信息:")
            print(f"  运行时间: {elapsed:.1f}秒")
            print(f"  接收行数: {self.line_count}")
            print(f"  错误次数: {self.error_count}")
            print(f"  数据率: {lps:.1f} 行/秒")
            print(f"  错误率: {error_rate:.2f}%")
            print(f"{'='*60}")
    
    def run(self, verbose=True, max_lines=None):
        """运行数据接收循环"""
        if not self.connect():
            return
        
        print(f"\n开始接收数据... (按Ctrl+C停止)\n")
        
        try:
            while True:
                if max_lines and self.line_count >= max_lines:
                    break
                
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                
                if not line:
                    continue
                
                # 跳过注释行（统计信息等）
                if line.startswith('#'):
                    print(f"[INFO] {line}")
                    continue
                
                imu_data_list = self.parse_line(line)
                if imu_data_list:
                    self.line_count += 1
                    self.print_data(imu_data_list, verbose)
                else:
                    self.error_count += 1
                
                # 每100行显示一次统计
                if self.line_count % 100 == 0 and self.line_count > 0:
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
        description='IMU文本数据接收器',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # Windows
  python text_reader.py COM3
  
  # Linux/Mac
  python text_reader.py /dev/ttyUSB0
  
  # 简洁模式
  python text_reader.py /dev/ttyUSB0 --quiet
        """
    )
    
    parser.add_argument('port', help='串口设备路径')
    parser.add_argument('-b', '--baudrate', type=int, default=921600,
                       help='波特率 (默认: 921600)')
    parser.add_argument('-n', '--num-imus', type=int, default=2,
                       help='IMU数量 (默认: 2)')
    parser.add_argument('-q', '--quiet', action='store_true',
                       help='简洁输出模式')
    parser.add_argument('-m', '--max-lines', type=int, default=None,
                       help='最大接收行数')
    
    args = parser.parse_args()
    
    reader = IMUTextReader(args.port, args.baudrate, args.num_imus)
    reader.run(verbose=not args.quiet, max_lines=args.max_lines)

if __name__ == '__main__':
    main()
