#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3
import serial


class LimbIMUNode(Node):
    def __init__(self):
        super().__init__('limb_imu_node')

        # 声明参数
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('verbose', True)
        self.declare_parameter('num_imus', 4)
        self.declare_parameter('read_period', 0.001)
        self.declare_parameter('log_interval', 3.0)
        self.declare_parameter('serial_timeout', 0.0)

        # 获取参数值
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        self.num_imus = self.get_parameter('num_imus').get_parameter_value().integer_value
        self.read_period = self.get_parameter('read_period').get_parameter_value().double_value
        self.log_interval = self.get_parameter('log_interval').get_parameter_value().double_value
        self.serial_timeout = self.get_parameter('serial_timeout').get_parameter_value().double_value

        self.get_logger().info(f'Connecting to serial port: {port}, baudrate: {baudrate}')
        self.get_logger().info(f'Number of IMUs: {self.num_imus}')
        self.get_logger().info(f'Read period: {self.read_period}s, log interval: {self.log_interval}s')

        # 创建发布者
        self.publishers_imu_limb = {}
        for i in range(1, self.num_imus + 1):
            pub_name = f'limb_imu/imu{i}'
            self.publishers_imu_limb[i] = self.create_publisher(Imu, pub_name, 10)
            self.get_logger().info(f'Created publisher: {pub_name}')

        # 存储每个IMU的最新四元数
        self.latest_quaternions = {
            i: [0.0, 0.0, 0.0, 1.0] for i in range(1, self.num_imus + 1)
        }

        self.buffer = ""

        # 初始化串口
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=self.serial_timeout,
            )
            self.get_logger().info(f'Successfully opened serial port: {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        # 创建定时器读取串口数据
        self.timer = self.create_timer(self.read_period, self.read_serial_data)

        # 用于统计发布频率
        self.counters = {i: 0 for i in range(1, self.num_imus + 1)}
        self.last_log_time = time.time()

    def parse_imu_data(self, data):
        """解析IMU数据"""
        lines = []

        # 添加新数据到缓冲区
        self.buffer += data

        # 查找完整的数据行
        while True:
            # 查找[IMU开头
            start = self.buffer.find('[IMU')
            if start == -1:
                # 没有IMU数据，清空缓冲区
                self.buffer = ""
                break

            # 查找换行符
            end = self.buffer.find('\n', start)
            if end == -1:
                # 数据不完整，保留在缓冲区中
                self.buffer = self.buffer[start:]
                break

            # 提取一行数据
            line = self.buffer[start:end].strip()
            self.buffer = self.buffer[end + 1:]

            if line:
                lines.append(line)

        if len(self.buffer) > 4096:
            self.buffer = self.buffer[-4096:]

        return lines

    def process_line(self, line):
        """处理单行IMU数据并发布"""
        if ']:' in line:
            parts = line.split(']:', 1)
            if len(parts) == 2:
                imu_id = parts[0]
                data_str = parts[1]

                # 提取IMU编号
                if imu_id.startswith('[IMU') and imu_id[4:].isdigit():
                    imu_num = int(imu_id[4:])

                    # 只处理有效的IMU编号
                    if 1 <= imu_num <= self.num_imus:
                        # 解析四元数
                        try:
                            # 去掉可能的空格并分割
                            data_str = data_str.strip()
                            values = [float(x) for x in data_str.split(',')]
                            if len(values) == 4:
                                # 存储最新的四元数
                                self.latest_quaternions[imu_num] = values
                                return imu_num, values
                        except ValueError as e:
                            self.get_logger().warning(
                                f'Failed to parse IMU data: {e}, line: {line}'
                            )
        return None, None

    def create_imu_msg(self, imu_num, quaternion):
        """创建IMU消息"""
        msg = Imu()

        # 设置header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.frame_id}_imu{imu_num}'

        # 设置四元数
        # 输入格式为 w, x, y, z
        # ROS格式为 x, y, z, w
        msg.orientation = Quaternion()
        msg.orientation.x = quaternion[1]  # x
        msg.orientation.y = quaternion[2]  # y
        msg.orientation.z = quaternion[3]  # z
        msg.orientation.w = quaternion[0]  # w

        # 设置角速度（未知）
        msg.angular_velocity = Vector3()
        msg.angular_velocity_covariance[0] = -1

        # 设置线加速度（未知）
        msg.linear_acceleration = Vector3()
        msg.linear_acceleration_covariance[0] = -1

        # 设置方向协方差矩阵（这里设置为未知）
        msg.orientation_covariance[0] = -1

        return msg

    def read_serial_data(self):
        """读取并处理串口数据"""
        try:
            if not (self.serial and self.serial.is_open):
                return

            data_chunks = []
            while True:
                waiting = self.serial.in_waiting
                if not waiting:
                    break
                data_chunks.append(self.serial.read(waiting))

            if not data_chunks:
                return

            ascii_data = b''.join(data_chunks).decode('ascii', errors='ignore')

            # 解析数据行
            lines = self.parse_imu_data(ascii_data)

            # 处理每一行并发布
            for line in lines:
                imu_num, quat = self.process_line(line)
                if imu_num and quat:
                    # 创建并发布消息
                    msg = self.create_imu_msg(imu_num, quat)
                    self.publishers_imu_limb[imu_num].publish(msg)

                    # 更新计数器
                    self.counters[imu_num] += 1

                    # 如果verbose为True，打印每个IMU的四元数
                    if self.verbose:
                        self.get_logger().debug(
                            f'IMU{imu_num}: '
                            f'w={quat[0]:.3f}, x={quat[1]:.3f}, '
                            f'y={quat[2]:.3f}, z={quat[3]:.3f}'
                        )

            # 定期输出频率统计和四元数信息
            current_time = time.time()
            if current_time - self.last_log_time > self.log_interval:
                self.last_log_time = current_time

                # 输出频率统计
                log_msg = "Publish frequencies: "
                for i in range(1, self.num_imus + 1):
                    freq = self.counters[i] / self.log_interval
                    log_msg += f'IMU{i}: {freq:.1f}Hz '
                    self.counters[i] = 0
                self.get_logger().info(log_msg)

                # 输出IMU的四元数信息
                quat_msg = "Latest quaternions:\n"
                for i in range(1, self.num_imus + 1):
                    q = self.latest_quaternions[i]
                    quat_msg += (
                        f'  IMU{i}: '
                        f'w={q[0]:.3f}, x={q[1]:.3f}, '
                        f'y={q[2]:.3f}, z={q[3]:.3f}\n'
                    )
                self.get_logger().info(quat_msg)

        except serial.SerialException as e:
            self.get_logger().error(f'Serial communication error: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error in read_serial_data: {e}')

    def destroy_node(self):
        """清理资源"""
        if hasattr(self, 'serial') and self.serial and self.serial.is_open:
            self.serial.close()
            self.get_logger().info('Serial port closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = LimbIMUNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Node interrupted by user')
    except Exception as e:
        if node:
            node.get_logger().error(f'Node initialization failed: {e}')
        else:
            print(f'Node initialization failed: {e}')
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
