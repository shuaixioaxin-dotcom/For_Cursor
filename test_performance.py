#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""性能测试脚本：对比优化前后的性能差异"""

import time
import argparse
from optimized_imu_reader import OptimizedIMUReader


def performance_test(port: str, baudrate: int, slave_ids: list, test_count: int = 100):
    """
    性能测试函数
    
    Args:
        port: 串口端口
        baudrate: 波特率
        slave_ids: 从站ID列表
        test_count: 测试次数
    """
    print("=" * 70)
    print("IMU Modbus RTU 性能测试")
    print("=" * 70)
    print(f"串口: {port}")
    print(f"波特率: {baudrate}")
    print(f"从站ID: {slave_ids}")
    print(f"测试次数: {test_count}")
    print("=" * 70)
    
    # 创建读取器
    reader = OptimizedIMUReader(port, baudrate, slave_ids)
    
    if not reader.connect():
        print("❌ 无法连接到串口")
        return
    
    print("✅ 串口连接成功\n")
    
    # 预热
    print("⏳ 预热中...")
    for _ in range(10):
        reader.read_all_imu_optimized()
    print("✅ 预热完成\n")
    
    # 性能测试
    print(f"🚀 开始性能测试（{test_count}次读取）...")
    
    success_count = 0
    fail_count = 0
    min_time = float('inf')
    max_time = 0
    total_time = 0
    times = []
    
    for i in range(test_count):
        start_time = time.perf_counter()
        
        try:
            imu_data = reader.read_all_imu_optimized()
            
            # 验证数据有效性
            valid = True
            for slave_id in slave_ids:
                if slave_id in imu_data:
                    acc = imu_data[slave_id]['acc']
                    quat = imu_data[slave_id]['quat']
                    # 简单验证数据是否合理
                    if all(isinstance(x, (int, float)) for x in acc + quat):
                        continue
                valid = False
                break
            
            if valid:
                success_count += 1
            else:
                fail_count += 1
                
        except Exception as e:
            fail_count += 1
            print(f"❌ 读取失败: {e}")
        
        end_time = time.perf_counter()
        cycle_time = end_time - start_time
        times.append(cycle_time)
        
        min_time = min(min_time, cycle_time)
        max_time = max(max_time, cycle_time)
        total_time += cycle_time
        
        # 显示进度
        if (i + 1) % 20 == 0:
            print(f"  进度: {i+1}/{test_count} ({(i+1)/test_count*100:.1f}%)")
    
    # 断开连接
    reader.disconnect()
    
    # 计算统计数据
    avg_time = total_time / test_count
    avg_time_ms = avg_time * 1000
    min_time_ms = min_time * 1000
    max_time_ms = max_time * 1000
    
    # 计算频率
    max_freq = 1.0 / avg_time if avg_time > 0 else 0
    
    # 计算方差和标准差
    variance = sum((t - avg_time) ** 2 for t in times) / test_count
    std_dev = variance ** 0.5
    std_dev_ms = std_dev * 1000
    
    # 显示结果
    print("\n" + "=" * 70)
    print("测试结果")
    print("=" * 70)
    
    print(f"\n📊 基本统计:")
    print(f"  总测试次数: {test_count}")
    print(f"  成功次数:   {success_count} ({success_count/test_count*100:.2f}%)")
    print(f"  失败次数:   {fail_count} ({fail_count/test_count*100:.2f}%)")
    
    print(f"\n⏱️  时间统计:")
    print(f"  平均耗时:   {avg_time_ms:.3f} ms")
    print(f"  最小耗时:   {min_time_ms:.3f} ms")
    print(f"  最大耗时:   {max_time_ms:.3f} ms")
    print(f"  标准差:     {std_dev_ms:.3f} ms")
    
    print(f"\n🚀 性能指标:")
    print(f"  最大频率:   {max_freq:.2f} Hz")
    print(f"  设备数量:   {len(slave_ids)}")
    print(f"  每秒读取:   {max_freq * len(slave_ids):.2f} 设备次/秒")
    
    # 性能评级
    print(f"\n⭐ 性能评级:")
    if avg_time_ms < 5:
        rating = "优秀 (Excellent)"
        emoji = "🌟🌟🌟🌟🌟"
    elif avg_time_ms < 10:
        rating = "良好 (Good)"
        emoji = "🌟🌟🌟🌟"
    elif avg_time_ms < 20:
        rating = "一般 (Fair)"
        emoji = "🌟🌟🌟"
    elif avg_time_ms < 50:
        rating = "较差 (Poor)"
        emoji = "🌟🌟"
    else:
        rating = "很差 (Very Poor)"
        emoji = "🌟"
    
    print(f"  {emoji} {rating}")
    
    # 优化建议
    print(f"\n💡 优化建议:")
    if avg_time_ms > 20:
        print("  • 考虑提高波特率以减少传输时间")
        print("  • 检查串口线缆质量和长度")
        print("  • 减少设备数量或分批读取")
    elif avg_time_ms > 10:
        print("  • 性能良好，可考虑进一步调优等待时间参数")
    else:
        print("  • 性能优秀，无需额外优化")
    
    if fail_count > test_count * 0.01:  # 失败率超过1%
        print("  • 失败率较高，检查:")
        print("    - 从站ID是否正确")
        print("    - 设备是否在线")
        print("    - CRC校验是否通过")
        print("    - 串口配置是否正确")
    
    print("\n" + "=" * 70)
    
    # 返回统计数据
    return {
        'total': test_count,
        'success': success_count,
        'fail': fail_count,
        'avg_time_ms': avg_time_ms,
        'min_time_ms': min_time_ms,
        'max_time_ms': max_time_ms,
        'std_dev_ms': std_dev_ms,
        'max_freq': max_freq,
        'device_count': len(slave_ids)
    }


def compare_configurations(port: str, baudrate: int, test_count: int = 100):
    """
    对比不同配置的性能
    
    Args:
        port: 串口端口
        baudrate: 波特率
        test_count: 每个配置的测试次数
    """
    print("\n" + "=" * 70)
    print("多配置性能对比测试")
    print("=" * 70)
    
    configurations = [
        ([2], "单设备"),
        ([2, 3], "双设备"),
        ([2, 3, 4], "三设备"),
        ([2, 3, 4, 5], "四设备"),
    ]
    
    results = []
    
    for slave_ids, desc in configurations:
        print(f"\n{'='*70}")
        print(f"测试配置: {desc} (ID: {slave_ids})")
        print(f"{'='*70}")
        
        try:
            result = performance_test(port, baudrate, slave_ids, test_count)
            if result:
                result['desc'] = desc
                result['ids'] = slave_ids
                results.append(result)
        except Exception as e:
            print(f"❌ 测试失败: {e}")
        
        # 间隔一下，让设备缓冲
        time.sleep(0.5)
    
    # 显示对比结果
    if len(results) > 1:
        print("\n" + "=" * 70)
        print("性能对比总结")
        print("=" * 70)
        print(f"\n{'配置':<12} {'设备数':<8} {'平均耗时(ms)':<15} {'最大频率(Hz)':<15} {'吞吐量':<15}")
        print("-" * 70)
        
        for result in results:
            throughput = result['max_freq'] * result['device_count']
            print(f"{result['desc']:<12} {result['device_count']:<8} "
                  f"{result['avg_time_ms']:<15.3f} {result['max_freq']:<15.2f} "
                  f"{throughput:<15.2f}")
        
        # 计算性能提升
        if len(results) >= 2:
            baseline = results[0]
            print(f"\n📈 相对{baseline['desc']}的性能提升:")
            for result in results[1:]:
                time_ratio = baseline['avg_time_ms'] / result['avg_time_ms']
                throughput_ratio = (result['max_freq'] * result['device_count']) / \
                                 (baseline['max_freq'] * baseline['device_count'])
                print(f"  {result['desc']}: "
                      f"速度比={time_ratio:.2f}x, "
                      f"吞吐量比={throughput_ratio:.2f}x")
        
        print("=" * 70)


def main():
    parser = argparse.ArgumentParser(description='IMU Modbus RTU 性能测试工具')
    parser.add_argument('-p', '--port', type=str, default='COM66', 
                       help='串口端口（默认COM66）')
    parser.add_argument('-b', '--baudrate', type=int, default=921600, 
                       help='波特率（默认921600）')
    parser.add_argument('-i', '--ids', type=str, default='2', 
                       help='从站ID列表，逗号分隔（默认2）')
    parser.add_argument('-c', '--count', type=int, default=100, 
                       help='测试次数（默认100）')
    parser.add_argument('--compare', action='store_true', 
                       help='对比不同设备数量的性能')
    
    args = parser.parse_args()
    
    try:
        if args.compare:
            # 对比模式
            compare_configurations(args.port, args.baudrate, args.count)
        else:
            # 单一配置测试
            slave_ids = [int(id_str.strip()) for id_str in args.ids.split(',')]
            performance_test(args.port, args.baudrate, slave_ids, args.count)
            
    except KeyboardInterrupt:
        print("\n\n⚠️  测试被用户中断")
    except Exception as e:
        print(f"\n❌ 发生错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
