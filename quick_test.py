#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Modbus RTU 快速测试脚本"""

import subprocess
import sys
import argparse

def run_test(test_name, args):
    """运行测试并打印结果"""
    print(f"\n{'='*80}")
    print(f"测试: {test_name}")
    print(f"{'='*80}")
    
    cmd = ["python", "modbus_rtu_test.py"] + args
    print(f"命令: {' '.join(cmd)}\n")
    
    try:
        subprocess.run(cmd, timeout=10)
    except subprocess.TimeoutExpired:
        print("\n[测试超时，已自动终止]")
    except KeyboardInterrupt:
        print("\n[用户中断测试]")
    
    input("\n按回车继续下一个测试...")


def main():
    parser = argparse.ArgumentParser(description='Modbus RTU 快速测试脚本')
    parser.add_argument('-p', '--port', type=str, default='COM66', help='串口端口')
    parser.add_argument('-b', '--baudrate', type=int, default=921600, help='波特率')
    args = parser.parse_args()
    
    base_args = ['-p', args.port, '-b', str(args.baudrate)]
    
    print("="*80)
    print("Modbus RTU 批处理工具 - 快速测试")
    print("="*80)
    print(f"使用串口: {args.port}")
    print(f"波特率: {args.baudrate}")
    print("\n将运行以下测试（每个测试10秒后自动停止）：")
    print("1. 默认配置测试（2个从站，最高频率）")
    print("2. 单从站测试（从站1）")
    print("3. 4从站批处理测试")
    print("4. 低频监控测试（1Hz）")
    print("5. 无延迟高速测试")
    
    input("\n按回车开始测试...")
    
    # 测试1：默认配置
    run_test(
        "默认配置（从站1,2，最高频率）",
        base_args + ['-c', '10']
    )
    
    # 测试2：单从站
    run_test(
        "单从站测试（从站1）",
        base_args + ['-s', '1', '-c', '10']
    )
    
    # 测试3：4从站批处理
    run_test(
        "4从站批处理（从站1,2,3,4）",
        base_args + ['-s', '1,2,3,4', '-c', '5']
    )
    
    # 测试4：低频监控
    run_test(
        "低频监控（1Hz）",
        base_args + ['-f', '1', '-c', '5']
    )
    
    # 测试5：无延迟高速
    run_test(
        "无延迟高速测试",
        base_args + ['-t', '0', '-c', '10']
    )
    
    print("\n" + "="*80)
    print("所有测试完成！")
    print("="*80)
    print("\n提示：")
    print("- 如需修改串口，使用: python quick_test.py -p COMX")
    print("- 如需修改波特率，使用: python quick_test.py -b 115200")
    print("- 查看完整帮助: python modbus_rtu_test.py -h")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n测试已中断")
        sys.exit(0)
