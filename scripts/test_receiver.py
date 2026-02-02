#!/usr/bin/env python3
"""
ESP32编码器接收端数据分析脚本
用于监控和分析接收到的编码器数据
"""

import serial
import time
import sys
from datetime import datetime

class EncoderDataMonitor:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.packet_count = 0
        self.error_count = 0
        self.last_seq = -1
        self.start_time = time.time()
        self.last_packet_time = 0
        self.intervals = []
        
    def parse_line(self, line):
        """解析串口输出的一行数据"""
        try:
            if '序列号' in line:
                # 提取序列号
                parts = line.split('序列号:')
                if len(parts) > 1:
                    seq_str = parts[1].split(')')[0].strip()
                    seq = int(seq_str)
                    self.check_sequence(seq)
                    
            elif '采样率' in line:
                # 提取采样率
                parts = line.split(':')
                if len(parts) > 1:
                    rate = int(parts[1].strip().split()[0])
                    return rate
                    
        except Exception as e:
            print(f"解析错误: {e}")
            
        return None
        
    def check_sequence(self, seq):
        """检查序列号连续性"""
        current_time = time.time()
        
        if self.last_seq >= 0:
            expected = (self.last_seq + 1) & 0xFFFF  # 16位序列号会回绕
            if seq != expected:
                self.error_count += 1
                print(f"\n⚠️  序列号跳变: {self.last_seq} -> {seq} (期望: {expected})")
                
            # 计算接收间隔
            if self.last_packet_time > 0:
                interval = (current_time - self.last_packet_time) * 1000  # ms
                self.intervals.append(interval)
                if len(self.intervals) > 100:
                    self.intervals.pop(0)
                    
        self.last_seq = seq
        self.last_packet_time = current_time
        self.packet_count += 1
        
    def get_statistics(self):
        """获取统计信息"""
        elapsed = time.time() - self.start_time
        avg_rate = self.packet_count / elapsed if elapsed > 0 else 0
        
        avg_interval = sum(self.intervals) / len(self.intervals) if self.intervals else 0
        min_interval = min(self.intervals) if self.intervals else 0
        max_interval = max(self.intervals) if self.intervals else 0
        
        return {
            'elapsed': elapsed,
            'packet_count': self.packet_count,
            'error_count': self.error_count,
            'avg_rate': avg_rate,
            'success_rate': (1 - self.error_count / max(self.packet_count, 1)) * 100,
            'avg_interval': avg_interval,
            'min_interval': min_interval,
            'max_interval': max_interval
        }
        
    def print_statistics(self):
        """打印统计信息"""
        stats = self.get_statistics()
        
        print("\n" + "="*60)
        print(f"运行时长: {stats['elapsed']:.1f} 秒")
        print(f"接收数据包: {stats['packet_count']}")
        print(f"丢包/错误: {stats['error_count']}")
        print(f"成功率: {stats['success_rate']:.2f}%")
        print(f"平均接收率: {stats['avg_rate']:.1f} Hz")
        
        if self.intervals:
            print(f"接收间隔: 平均={stats['avg_interval']:.1f}ms, "
                  f"最小={stats['min_interval']:.1f}ms, "
                  f"最大={stats['max_interval']:.1f}ms")
        
        print("="*60)
        
    def run(self):
        """主循环"""
        print("ESP32编码器数据监控")
        print("="*60)
        print(f"串口: {self.ser.port}")
        print(f"波特率: {self.ser.baudrate}")
        print("按 Ctrl+C 停止监控")
        print("="*60 + "\n")
        
        last_stats_time = time.time()
        
        try:
            while True:
                if self.ser.in_waiting:
                    try:
                        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            self.parse_line(line)
                            
                            # 简化输出：每100个包显示一个点
                            if self.packet_count % 100 == 0:
                                print(".", end="", flush=True)
                                
                            # 每50个点换行
                            if self.packet_count % 5000 == 0:
                                print()
                                
                    except UnicodeDecodeError:
                        pass
                        
                # 每10秒打印一次统计
                if time.time() - last_stats_time >= 10:
                    self.print_statistics()
                    last_stats_time = time.time()
                    
        except KeyboardInterrupt:
            print("\n\n停止监控")
            self.print_statistics()
            
            # 保存日志
            log_filename = f"test_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
            with open(log_filename, 'w') as f:
                stats = self.get_statistics()
                f.write("ESP32编码器测试报告\n")
                f.write("="*60 + "\n")
                f.write(f"测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"运行时长: {stats['elapsed']:.1f} 秒\n")
                f.write(f"接收数据包: {stats['packet_count']}\n")
                f.write(f"丢包/错误: {stats['error_count']}\n")
                f.write(f"成功率: {stats['success_rate']:.2f}%\n")
                f.write(f"平均接收率: {stats['avg_rate']:.1f} Hz\n")
                
            print(f"\n日志已保存到: {log_filename}")
            
        finally:
            self.ser.close()

if __name__ == "__main__":
    # 从命令行参数获取串口
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    
    try:
        monitor = EncoderDataMonitor(port=port)
        monitor.run()
    except serial.SerialException as e:
        print(f"串口错误: {e}")
        print(f"请检查串口 {port} 是否可用")
        sys.exit(1)
