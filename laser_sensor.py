#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
激光传感器数据帧解析脚本

帧格式（9字节）:
0x59 0x59 dis_L dis_H Amp_L Amp_H Temp_L Temp_H Check_sum

距离值 = dis_H * 256 + dis_L
AMP值 = Amp_H * 256 + Amp_L

AMP取值范围：
- > 400 且 < 65535：最佳
- < 100：不可用
"""

import serial
import time
import argparse
from typing import Optional, Tuple
from dataclasses import dataclass
from enum import Enum


class AmpStatus(Enum):
    """AMP信号强度状态"""
    OPTIMAL = "最佳"          # 400 < AMP < 65535
    ACCEPTABLE = "可接受"     # 100 <= AMP <= 400
    UNUSABLE = "不可用"       # AMP < 100


@dataclass
class SensorData:
    """传感器数据结构"""
    distance: int       # 距离值 (cm/mm 取决于传感器型号)
    amp: int            # 信号强度
    temperature: int    # 温度原始值
    amp_status: AmpStatus
    raw_frame: bytes    # 原始帧数据


# 帧常量
FRAME_HEADER = 0x59
FRAME_LENGTH = 9


def calculate_checksum(data: bytes) -> int:
    """
    计算校验和
    校验和 = 前8字节之和的低8位
    """
    return sum(data[:8]) & 0xFF


def get_amp_status(amp: int) -> AmpStatus:
    """
    判断AMP信号强度状态
    
    Args:
        amp: AMP值
        
    Returns:
        AmpStatus: 信号强度状态
    """
    if amp < 100:
        return AmpStatus.UNUSABLE
    elif amp > 400:
        return AmpStatus.OPTIMAL
    else:
        return AmpStatus.ACCEPTABLE


def parse_frame(frame: bytes) -> Optional[SensorData]:
    """
    解析数据帧
    
    Args:
        frame: 9字节的原始数据帧
        
    Returns:
        SensorData: 解析后的传感器数据，解析失败返回None
    """
    if len(frame) != FRAME_LENGTH:
        return None
    
    # 验证帧头
    if frame[0] != FRAME_HEADER or frame[1] != FRAME_HEADER:
        return None
    
    # 验证校验和
    expected_checksum = calculate_checksum(frame)
    actual_checksum = frame[8]
    
    if expected_checksum != actual_checksum:
        print(f"校验和错误: 期望={expected_checksum:#04x}, 实际={actual_checksum:#04x}")
        return None
    
    # 解析数据
    dis_l = frame[2]
    dis_h = frame[3]
    amp_l = frame[4]
    amp_h = frame[5]
    temp_l = frame[6]
    temp_h = frame[7]
    
    # 计算实际值
    distance = dis_h * 256 + dis_l
    amp = amp_h * 256 + amp_l
    temperature = temp_h * 256 + temp_l
    
    # 获取AMP状态
    amp_status = get_amp_status(amp)
    
    return SensorData(
        distance=distance,
        amp=amp,
        temperature=temperature,
        amp_status=amp_status,
        raw_frame=frame
    )


class LaserSensor:
    """激光传感器类"""
    
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.1):
        """
        初始化激光传感器
        
        Args:
            port: 串口号 (例如: '/dev/ttyUSB0', 'COM37')
            baudrate: 波特率，默认115200
            timeout: 读取超时时间（秒）
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        self._buffer = bytearray()
    
    def connect(self) -> bool:
        """
        连接串口
        
        Returns:
            bool: 连接是否成功
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            print(f"已连接到串口: {self.port}, 波特率: {self.baudrate}")
            return True
        except serial.SerialException as e:
            print(f"串口连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("串口已断开")
    
    def read_frame(self) -> Optional[SensorData]:
        """
        读取并解析一帧数据
        
        Returns:
            SensorData: 解析后的传感器数据，失败返回None
        """
        if not self.serial or not self.serial.is_open:
            return None
        
        # 读取可用数据到缓冲区
        if self.serial.in_waiting > 0:
            self._buffer.extend(self.serial.read(self.serial.in_waiting))
        
        # 搜索帧头并解析
        while len(self._buffer) >= FRAME_LENGTH:
            # 查找帧头 0x59 0x59
            header_index = -1
            for i in range(len(self._buffer) - 1):
                if self._buffer[i] == FRAME_HEADER and self._buffer[i + 1] == FRAME_HEADER:
                    header_index = i
                    break
            
            if header_index == -1:
                # 没找到帧头，保留最后一个字节（可能是帧头的开始）
                self._buffer = self._buffer[-1:]
                return None
            
            # 丢弃帧头之前的数据
            if header_index > 0:
                self._buffer = self._buffer[header_index:]
            
            # 检查是否有完整帧
            if len(self._buffer) < FRAME_LENGTH:
                return None
            
            # 提取一帧数据
            frame = bytes(self._buffer[:FRAME_LENGTH])
            self._buffer = self._buffer[FRAME_LENGTH:]
            
            # 解析帧
            data = parse_frame(frame)
            if data:
                return data
        
        return None
    
    def run(self, show_raw: bool = False, interval: float = 0):
        """
        持续读取并显示传感器数据
        
        Args:
            show_raw: 是否显示原始帧数据
            interval: 显示间隔（秒），0表示尽可能快
        """
        if not self.connect():
            return
        
        print("\n开始读取传感器数据... (按 Ctrl+C 停止)")
        print("注意: AMP < 100 的数据帧将被丢弃\n")
        print("-" * 70)
        print(f"{'时间戳':<12} {'距离':<10} {'AMP':<10} {'AMP状态':<12} {'温度原始值':<10}")
        print("-" * 70)
        
        valid_frame_count = 0
        discarded_frame_count = 0
        last_print_time = 0
        
        try:
            while True:
                data = self.read_frame()
                
                if data:
                    # AMP低于100时丢弃该帧，不进行处理
                    if data.amp < 100:
                        discarded_frame_count += 1
                        continue
                    
                    valid_frame_count += 1
                    current_time = time.time()
                    
                    # 控制输出频率
                    if interval == 0 or (current_time - last_print_time) >= interval:
                        timestamp = time.strftime("%H:%M:%S")
                        
                        # 根据AMP状态设置标记
                        status_marker = " ✓" if data.amp_status == AmpStatus.OPTIMAL else ""
                        
                        print(f"{timestamp:<12} {data.distance:<10} {data.amp:<10} "
                              f"{data.amp_status.value:<10}{status_marker} {data.temperature:<10}", end="")
                        
                        if show_raw:
                            raw_hex = ' '.join(f'{b:02X}' for b in data.raw_frame)
                            print(f"  [{raw_hex}]", end="")
                        
                        print()
                        last_print_time = current_time
                
                # 短暂休眠以降低CPU占用
                time.sleep(0.001)
                
        except KeyboardInterrupt:
            print("\n" + "-" * 70)
            print(f"\n停止读取. 有效帧数: {valid_frame_count}, 丢弃帧数(AMP<100): {discarded_frame_count}")
        finally:
            self.disconnect()


def main():
    parser = argparse.ArgumentParser(description='激光传感器数据帧解析工具')
    parser.add_argument('-p', '--port', type=str, default='/dev/ttyUSB0',
                        help='串口号 (默认: /dev/ttyUSB0, Windows下可用COM37等)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200,
                        help='波特率 (默认: 115200)')
    parser.add_argument('-r', '--raw', action='store_true',
                        help='显示原始帧数据')
    parser.add_argument('-i', '--interval', type=float, default=0,
                        help='显示间隔（秒），0表示尽可能快 (默认: 0)')
    parser.add_argument('-t', '--test', action='store_true',
                        help='运行测试模式（不需要实际硬件）')
    
    args = parser.parse_args()
    
    if args.test:
        run_test()
    else:
        sensor = LaserSensor(port=args.port, baudrate=args.baudrate)
        sensor.run(show_raw=args.raw, interval=args.interval)


def run_test():
    """测试模式：使用模拟数据测试解析功能"""
    print("=" * 70)
    print("测试模式 - 测试帧解析功能")
    print("=" * 70)
    print("注意: AMP < 100 的数据帧将被丢弃，不进行计算")
    
    # 测试用例
    test_cases = [
        # 正常帧 - 距离=1000, AMP=500 (最佳)
        {
            "name": "正常帧 (AMP=500, 最佳)",
            "frame": bytes([0x59, 0x59, 0xE8, 0x03, 0xF4, 0x01, 0x00, 0x00, 0x00]),
            "should_discard": False,
        },
        # AMP=50 (不可用 - 应丢弃)
        {
            "name": "低AMP帧 (AMP=50, 应丢弃)",
            "frame": bytes([0x59, 0x59, 0x64, 0x00, 0x32, 0x00, 0x00, 0x00, 0x00]),
            "should_discard": True,
        },
        # AMP=200 (可接受)
        {
            "name": "中等AMP帧 (AMP=200, 可接受)",
            "frame": bytes([0x59, 0x59, 0xC8, 0x00, 0xC8, 0x00, 0x00, 0x00, 0x00]),
            "should_discard": False,
        },
        # AMP=99 (边界值 - 应丢弃)
        {
            "name": "边界值帧 (AMP=99, 应丢弃)",
            "frame": bytes([0x59, 0x59, 0x64, 0x00, 0x63, 0x00, 0x00, 0x00, 0x00]),
            "should_discard": True,
        },
        # AMP=100 (边界值 - 保留)
        {
            "name": "边界值帧 (AMP=100, 保留)",
            "frame": bytes([0x59, 0x59, 0x64, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00]),
            "should_discard": False,
        },
    ]
    
    for tc in test_cases:
        # 计算正确的校验和
        frame_list = list(tc["frame"])
        checksum = sum(frame_list[:8]) & 0xFF
        frame_list[8] = checksum
        frame = bytes(frame_list)
        
        print(f"\n测试: {tc['name']}")
        print(f"原始帧: {' '.join(f'{b:02X}' for b in frame)}")
        
        data = parse_frame(frame)
        if data:
            # 检查是否应该丢弃
            if data.amp < 100:
                print(f"  ⚠️ 丢弃: AMP={data.amp} < 100，数据无效")
                if tc["should_discard"]:
                    print("  ✓ 测试通过: 正确识别为应丢弃")
                else:
                    print("  ✗ 测试失败: 不应该被丢弃")
            else:
                print(f"  距离: {data.distance}")
                print(f"  AMP: {data.amp}")
                print(f"  AMP状态: {data.amp_status.value}")
                print(f"  温度原始值: {data.temperature}")
                if not tc["should_discard"]:
                    print("  ✓ 测试通过: 正确保留有效数据")
                else:
                    print("  ✗ 测试失败: 应该被丢弃")
        else:
            print("  解析失败!")
    
    # 测试错误帧
    print("\n" + "-" * 70)
    print("测试错误帧...")
    
    # 错误帧头
    bad_header = bytes([0x58, 0x59, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    result = parse_frame(bad_header)
    print(f"\n错误帧头测试: {'通过 (正确拒绝)' if result is None else '失败'}")
    
    # 错误校验和
    bad_checksum = bytes([0x59, 0x59, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF])
    result = parse_frame(bad_checksum)
    print(f"错误校验和测试: {'通过 (正确拒绝)' if result is None else '失败'}")
    
    print("\n" + "=" * 70)
    print("测试完成!")
    print("=" * 70)


if __name__ == '__main__':
    main()
