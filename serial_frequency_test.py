#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
串口响应频率测试脚本

用于测试Modbus RTU设备的最高响应频率
发送帧：01 03 00 01 00 01 D5 CA
预期接收：01 03 02 XX XX CRC CRC

用法：
    python serial_frequency_test.py                    # 使用默认参数持续发送
    python serial_frequency_test.py --continuous       # 持续高频发送模式
    python serial_frequency_test.py --rounds 5         # 固定轮数测试模式
"""

import serial
import time
import argparse
import statistics
from typing import Optional, Tuple, List
from datetime import datetime
import sys


class SerialFrequencyTester:
    """串口响应频率测试类"""

    # Modbus RTU 请求帧: 读取从站地址01的保持寄存器，起始地址0001，读取1个寄存器
    # 01 03 00 01 00 01 D5 CA
    REQUEST_FRAME = bytes([0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0xD5, 0xCA])

    # 预期响应帧长度 (从站地址 + 功能码 + 字节数 + 数据 + CRC) = 1 + 1 + 1 + 2 + 2 = 7字节
    EXPECTED_RESPONSE_LENGTH = 7

    def __init__(
        self,
        port: str,
        baudrate: int = 38400,
        timeout: float = 0.05,
        bytesize: int = 8,
        parity: str = "N",
        stopbits: int = 1,
    ):
        """
        初始化串口测试器

        Args:
            port: 串口端口号 (如 /dev/ttyUSB0 或 COM3)
            baudrate: 波特率
            timeout: 读取超时时间(秒)
            bytesize: 数据位
            parity: 校验位 (N=无, E=偶, O=奇)
            stopbits: 停止位
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.serial_conn: Optional[serial.Serial] = None

    def open(self) -> bool:
        """打开串口连接"""
        try:
            parity_map = {"N": serial.PARITY_NONE, "E": serial.PARITY_EVEN, "O": serial.PARITY_ODD}
            stopbits_map = {1: serial.STOPBITS_ONE, 2: serial.STOPBITS_TWO}

            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=self.bytesize,
                parity=parity_map.get(self.parity, serial.PARITY_NONE),
                stopbits=stopbits_map.get(self.stopbits, serial.STOPBITS_ONE),
                timeout=self.timeout,
            )
            print(f"✓ 串口已打开: {self.port} @ {self.baudrate} bps")
            return True
        except serial.SerialException as e:
            print(f"✗ 串口打开失败: {e}")
            return False

    def close(self):
        """关闭串口连接"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print(f"✓ 串口已关闭: {self.port}")

    def send_and_receive(self) -> Tuple[bool, float, bytes]:
        """
        发送请求帧并接收响应

        Returns:
            Tuple[bool, float, bytes]: (成功标志, 响应时间(ms), 接收数据)
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return False, 0.0, b""

        # 清空输入缓冲区
        self.serial_conn.reset_input_buffer()

        # 记录发送时间
        start_time = time.perf_counter()

        # 发送请求帧
        self.serial_conn.write(self.REQUEST_FRAME)
        self.serial_conn.flush()

        # 接收响应
        response = self.serial_conn.read(self.EXPECTED_RESPONSE_LENGTH)

        # 记录接收时间
        end_time = time.perf_counter()

        # 计算响应时间(毫秒)
        response_time_ms = (end_time - start_time) * 1000

        # 验证响应
        success = len(response) >= self.EXPECTED_RESPONSE_LENGTH

        return success, response_time_ms, response

    def verify_response(self, response: bytes) -> bool:
        """
        验证响应帧格式

        Args:
            response: 接收到的响应数据

        Returns:
            bool: 响应是否有效
        """
        if len(response) < self.EXPECTED_RESPONSE_LENGTH:
            return False

        # 检查从站地址和功能码
        if response[0] != 0x01 or response[1] != 0x03:
            return False

        # 检查字节数字段
        if response[2] != 0x02:
            return False

        return True

    def run_continuous_test(self, stats_interval: int = 100):
        """
        持续高频发送测试（无限循环，直到Ctrl+C中断）

        Args:
            stats_interval: 每隔多少次请求打印一次统计信息
        """
        print("\n" + "#" * 60)
        print("# 持续高频发送模式")
        print("#" * 60)
        print(f"串口: {self.port}")
        print(f"波特率: {self.baudrate}")
        print(f"发送帧: {self.REQUEST_FRAME.hex(' ').upper()}")
        print(f"统计间隔: 每 {stats_interval} 次请求")
        print("-" * 60)
        print("按 Ctrl+C 停止测试...")
        print("-" * 60)

        # 统计变量
        total_requests = 0
        total_success = 0
        total_fail = 0
        total_invalid = 0
        response_times: List[float] = []
        interval_times: List[float] = []

        test_start = time.perf_counter()
        interval_start = test_start
        last_response = b""

        try:
            while True:
                success, response_time, response = self.send_and_receive()
                total_requests += 1

                if success:
                    if self.verify_response(response):
                        total_success += 1
                        response_times.append(response_time)
                        interval_times.append(response_time)
                        last_response = response
                    else:
                        total_invalid += 1
                else:
                    total_fail += 1

                # 每隔 stats_interval 次打印统计信息
                if total_requests % stats_interval == 0:
                    now = time.perf_counter()
                    interval_elapsed = (now - interval_start) * 1000  # ms
                    total_elapsed = (now - test_start)  # seconds

                    # 计算区间统计
                    if interval_times:
                        interval_avg = statistics.mean(interval_times)
                        interval_min = min(interval_times)
                        interval_max = max(interval_times)
                        interval_hz = len(interval_times) / (interval_elapsed / 1000) if interval_elapsed > 0 else 0
                    else:
                        interval_avg = interval_min = interval_max = 0
                        interval_hz = 0

                    # 计算总体统计
                    overall_hz = total_requests / total_elapsed if total_elapsed > 0 else 0
                    success_rate = (total_success / total_requests * 100) if total_requests > 0 else 0

                    # 打印实时统计
                    print(
                        f"[{total_requests:8d}] "
                        f"成功率: {success_rate:5.1f}% | "
                        f"响应: {interval_avg:5.2f}ms (min:{interval_min:5.2f} max:{interval_max:5.2f}) | "
                        f"频率: {interval_hz:6.1f} Hz | "
                        f"总频率: {overall_hz:6.1f} Hz | "
                        f"数据: {last_response.hex(' ').upper() if last_response else 'N/A'}"
                    )

                    # 重置区间统计
                    interval_times = []
                    interval_start = now

        except KeyboardInterrupt:
            pass

        # 打印最终统计
        test_end = time.perf_counter()
        total_time = test_end - test_start

        print("\n" + "=" * 60)
        print("最终统计结果")
        print("=" * 60)
        print(f"  总请求数: {total_requests}")
        print(f"  成功次数: {total_success}")
        print(f"  失败次数: {total_fail}")
        print(f"  无效响应: {total_invalid}")
        print(f"  成功率: {total_success/total_requests*100:.2f}%" if total_requests > 0 else "  成功率: N/A")
        print("-" * 60)
        print(f"  总运行时间: {total_time:.2f} 秒")
        print(f"  平均频率: {total_requests/total_time:.2f} Hz" if total_time > 0 else "  平均频率: N/A")

        if response_times:
            print("-" * 60)
            print(f"  最小响应时间: {min(response_times):.2f} ms")
            print(f"  最大响应时间: {max(response_times):.2f} ms")
            print(f"  平均响应时间: {statistics.mean(response_times):.2f} ms")
            if len(response_times) > 1:
                print(f"  响应时间标准差: {statistics.stdev(response_times):.2f} ms")
            theoretical_max_hz = 1000 / statistics.mean(response_times)
            print("-" * 60)
            print(f"  ★ 理论最高响应频率: {theoretical_max_hz:.2f} Hz")

        print("=" * 60)

        return {
            "total_requests": total_requests,
            "total_success": total_success,
            "total_fail": total_fail,
            "total_invalid": total_invalid,
            "total_time": total_time,
            "response_times": response_times,
        }

    def run_single_test(
        self, num_requests: int, interval_ms: float = 0
    ) -> dict:
        """
        运行单次测试

        Args:
            num_requests: 请求次数
            interval_ms: 请求间隔(毫秒)

        Returns:
            dict: 测试结果统计
        """
        response_times: List[float] = []
        success_count = 0
        fail_count = 0
        invalid_count = 0

        print(f"\n开始测试: {num_requests}次请求, 间隔{interval_ms:.2f}ms")
        print("-" * 50)

        test_start = time.perf_counter()

        for i in range(num_requests):
            success, response_time, response = self.send_and_receive()

            if success:
                if self.verify_response(response):
                    success_count += 1
                    response_times.append(response_time)
                    if i < 5 or i >= num_requests - 3:  # 显示前5个和最后3个
                        print(
                            f"  [{i+1:4d}] ✓ 响应时间: {response_time:.2f}ms, "
                            f"数据: {response.hex(' ').upper()}"
                        )
                else:
                    invalid_count += 1
                    print(
                        f"  [{i+1:4d}] ⚠ 无效响应: {response.hex(' ').upper()}"
                    )
            else:
                fail_count += 1
                if fail_count <= 5:
                    print(f"  [{i+1:4d}] ✗ 超时或无响应")

            # 请求间隔
            if interval_ms > 0:
                time.sleep(interval_ms / 1000.0)

        test_end = time.perf_counter()
        total_time = (test_end - test_start) * 1000  # 总耗时(ms)

        # 计算统计数据
        results = {
            "total_requests": num_requests,
            "success_count": success_count,
            "fail_count": fail_count,
            "invalid_count": invalid_count,
            "total_time_ms": total_time,
            "actual_frequency_hz": (num_requests / total_time) * 1000 if total_time > 0 else 0,
        }

        if response_times:
            results["min_response_ms"] = min(response_times)
            results["max_response_ms"] = max(response_times)
            results["avg_response_ms"] = statistics.mean(response_times)
            results["std_response_ms"] = (
                statistics.stdev(response_times) if len(response_times) > 1 else 0
            )
            # 理论最高频率 = 1000 / 平均响应时间
            results["theoretical_max_hz"] = 1000 / results["avg_response_ms"]
        else:
            results["min_response_ms"] = 0
            results["max_response_ms"] = 0
            results["avg_response_ms"] = 0
            results["std_response_ms"] = 0
            results["theoretical_max_hz"] = 0

        return results

    def print_results(self, results: dict):
        """打印测试结果"""
        print("\n" + "=" * 50)
        print("测试结果统计")
        print("=" * 50)
        print(f"  总请求数: {results['total_requests']}")
        print(f"  成功次数: {results['success_count']}")
        print(f"  失败次数: {results['fail_count']}")
        print(f"  无效响应: {results['invalid_count']}")
        print(
            f"  成功率: {results['success_count']/results['total_requests']*100:.1f}%"
        )
        print("-" * 50)
        print(f"  总耗时: {results['total_time_ms']:.2f} ms")
        print(f"  实际测试频率: {results['actual_frequency_hz']:.2f} Hz")
        print("-" * 50)
        print(f"  最小响应时间: {results['min_response_ms']:.2f} ms")
        print(f"  最大响应时间: {results['max_response_ms']:.2f} ms")
        print(f"  平均响应时间: {results['avg_response_ms']:.2f} ms")
        print(f"  响应时间标准差: {results['std_response_ms']:.2f} ms")
        print("-" * 50)
        print(f"  ★ 理论最高响应频率: {results['theoretical_max_hz']:.2f} Hz")
        print("=" * 50)

    def find_max_frequency(
        self, test_rounds: int = 5, requests_per_round: int = 100
    ) -> dict:
        """
        寻找最高响应频率

        Args:
            test_rounds: 测试轮数
            requests_per_round: 每轮请求次数

        Returns:
            dict: 最终测试结果
        """
        all_results = []

        print("\n" + "#" * 60)
        print("# 串口响应频率测试")
        print("#" * 60)
        print(f"串口: {self.port}")
        print(f"波特率: {self.baudrate}")
        print(f"发送帧: {self.REQUEST_FRAME.hex(' ').upper()}")
        print(f"测试轮数: {test_rounds}")
        print(f"每轮请求: {requests_per_round}")

        for round_num in range(test_rounds):
            print(f"\n{'='*20} 第 {round_num + 1} 轮 {'='*20}")
            results = self.run_single_test(requests_per_round, interval_ms=0)
            self.print_results(results)
            all_results.append(results)

            # 轮次间短暂休息
            if round_num < test_rounds - 1:
                time.sleep(0.5)

        # 汇总结果
        print("\n" + "#" * 60)
        print("# 最终汇总")
        print("#" * 60)

        total_success = sum(r["success_count"] for r in all_results)
        total_requests = sum(r["total_requests"] for r in all_results)
        all_avg_times = [r["avg_response_ms"] for r in all_results if r["avg_response_ms"] > 0]
        all_max_hz = [r["theoretical_max_hz"] for r in all_results if r["theoretical_max_hz"] > 0]

        if all_avg_times:
            overall_avg = statistics.mean(all_avg_times)
            overall_max_hz = statistics.mean(all_max_hz)
            best_hz = max(all_max_hz)
            worst_hz = min(all_max_hz)
        else:
            overall_avg = 0
            overall_max_hz = 0
            best_hz = 0
            worst_hz = 0

        print(f"  总成功率: {total_success}/{total_requests} ({total_success/total_requests*100:.1f}%)")
        print(f"  平均响应时间: {overall_avg:.2f} ms")
        print(f"  平均最高频率: {overall_max_hz:.2f} Hz")
        print(f"  最佳最高频率: {best_hz:.2f} Hz")
        print(f"  最差最高频率: {worst_hz:.2f} Hz")
        print("#" * 60)

        return {
            "all_results": all_results,
            "total_success_rate": total_success / total_requests if total_requests > 0 else 0,
            "overall_avg_response_ms": overall_avg,
            "overall_max_hz": overall_max_hz,
            "best_hz": best_hz,
            "worst_hz": worst_hz,
        }


def list_serial_ports():
    """列出可用的串口"""
    import serial.tools.list_ports

    ports = serial.tools.list_ports.comports()
    if ports:
        print("可用串口列表:")
        for port in ports:
            print(f"  - {port.device}: {port.description}")
    else:
        print("未检测到可用串口")
    return [port.device for port in ports]


def main():
    parser = argparse.ArgumentParser(
        description="串口响应频率测试脚本 - 测试Modbus RTU设备最高响应频率",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python serial_frequency_test.py                          # 默认COM65持续高频发送
  python serial_frequency_test.py --continuous             # 持续高频发送模式
  python serial_frequency_test.py --rounds 5               # 固定5轮测试模式
  python serial_frequency_test.py --port COM3 --baudrate 115200
  python serial_frequency_test.py --list                   # 列出可用串口
        """,
    )

    parser.add_argument("--port", "-p", type=str, default="COM65", help="串口端口号 (默认: COM65)")
    parser.add_argument("--baudrate", "-b", type=int, default=38400, help="波特率 (默认: 38400)")
    parser.add_argument("--timeout", "-t", type=float, default=0.05, help="读取超时(秒) (默认: 0.05)")
    parser.add_argument("--rounds", "-r", type=int, default=0, help="测试轮数 (默认: 0=持续模式)")
    parser.add_argument("--requests", "-n", type=int, default=100, help="每轮请求次数 (默认: 100)")
    parser.add_argument("--parity", type=str, default="N", choices=["N", "E", "O"], help="校验位 (默认: N)")
    parser.add_argument("--stopbits", type=int, default=1, choices=[1, 2], help="停止位 (默认: 1)")
    parser.add_argument("--continuous", "-c", action="store_true", help="持续高频发送模式 (默认)")
    parser.add_argument("--stats-interval", "-s", type=int, default=100, help="统计打印间隔 (默认: 100)")
    parser.add_argument("--list", "-l", action="store_true", help="列出可用串口")

    args = parser.parse_args()

    # 列出串口
    if args.list:
        list_serial_ports()
        return

    # 创建测试器
    tester = SerialFrequencyTester(
        port=args.port,
        baudrate=args.baudrate,
        timeout=args.timeout,
        parity=args.parity,
        stopbits=args.stopbits,
    )

    # 打开串口
    if not tester.open():
        return

    try:
        # 判断运行模式
        if args.rounds > 0 and not args.continuous:
            # 固定轮数测试模式
            results = tester.find_max_frequency(
                test_rounds=args.rounds,
                requests_per_round=args.requests,
            )

            # 保存结果到文件
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            result_file = f"serial_test_result_{timestamp}.txt"
            with open(result_file, "w", encoding="utf-8") as f:
                f.write(f"串口响应频率测试结果\n")
                f.write(f"测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"串口: {args.port}\n")
                f.write(f"波特率: {args.baudrate}\n")
                f.write(f"总成功率: {results['total_success_rate']*100:.1f}%\n")
                f.write(f"平均响应时间: {results['overall_avg_response_ms']:.2f} ms\n")
                f.write(f"理论最高频率: {results['best_hz']:.2f} Hz\n")
            print(f"\n结果已保存到: {result_file}")
        else:
            # 持续高频发送模式（默认）
            results = tester.run_continuous_test(stats_interval=args.stats_interval)

            # 保存结果到文件
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            result_file = f"serial_test_result_{timestamp}.txt"
            with open(result_file, "w", encoding="utf-8") as f:
                f.write(f"串口响应频率测试结果 (持续模式)\n")
                f.write(f"测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"串口: {args.port}\n")
                f.write(f"波特率: {args.baudrate}\n")
                f.write(f"总请求数: {results['total_requests']}\n")
                f.write(f"成功次数: {results['total_success']}\n")
                f.write(f"失败次数: {results['total_fail']}\n")
                f.write(f"总运行时间: {results['total_time']:.2f} 秒\n")
                if results['response_times']:
                    f.write(f"平均响应时间: {statistics.mean(results['response_times']):.2f} ms\n")
                    f.write(f"平均频率: {results['total_requests']/results['total_time']:.2f} Hz\n")
            print(f"\n结果已保存到: {result_file}")

    except KeyboardInterrupt:
        print("\n\n测试被用户中断")
    finally:
        tester.close()


if __name__ == "__main__":
    main()
