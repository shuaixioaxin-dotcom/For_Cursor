#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3, Vector3Stamped
import serial


class LegIMUNode(Node):
    def __init__(self):
        super().__init__('leg_imu_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('verbose', True)
        self.declare_parameter('num_imus', 2)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        self.num_imus = self.get_parameter('num_imus').get_parameter_value().integer_value

        self.get_logger().info(f'Connecting to serial port: {port}, baudrate: {baudrate}')
        self.get_logger().info(f'Number of IMUs: {self.num_imus}')

        self.imu_publishers = {}
        self.acc_publishers = {}
        for i in range(1, self.num_imus + 1):
            imu_pub_name = f'leg_imu/imu{i}'
            acc_pub_name = f'leg_imu/acc{i}'
            self.imu_publishers[i] = self.create_publisher(Imu, imu_pub_name, 10)
            self.acc_publishers[i] = self.create_publisher(Vector3Stamped, acc_pub_name, 10)
            self.get_logger().info(f'Created publisher: {imu_pub_name}')
            self.get_logger().info(f'Created publisher: {acc_pub_name}')

        self.latest_quaternions = {
            i: [0.0, 0.0, 0.0, 1.0] for i in range(1, self.num_imus + 1)
        }
        self.latest_accels = {
            i: [0.0, 0.0, 0.0] for i in range(1, self.num_imus + 1)
        }

        self.buffer = ""
        self.current_imu = None

        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.1,
            )
            self.get_logger().info(f'Successfully opened serial port: {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        self.timer = self.create_timer(0.001, self.read_serial_data)

        self.imu_counters = {i: 0 for i in range(1, self.num_imus + 1)}
        self.acc_counters = {i: 0 for i in range(1, self.num_imus + 1)}
        self.last_log_time = time.time()

    def parse_imu_data(self, data):
        lines = []
        self.buffer += data

        while True:
            newline_index = self.buffer.find('\n')
            if newline_index == -1:
                break

            line = self.buffer[:newline_index].strip()
            self.buffer = self.buffer[newline_index + 1:]
            if line:
                lines.append(line)

        if len(self.buffer) > 4096:
            self.buffer = self.buffer[-4096:]

        return lines

    def parse_vector(self, data_str, expected_len):
        tokens = [token.strip() for token in data_str.split(',') if token.strip()]
        if len(tokens) != expected_len:
            raise ValueError(f'Expected {expected_len} values, got {len(tokens)}')
        return [float(token) for token in tokens]

    def process_payload(self, imu_num, payload, line):
        if payload.startswith('acc:'):
            data_str = payload[4:].strip()
            try:
                accel = self.parse_vector(data_str, 3)
            except ValueError as e:
                self.get_logger().warning(f'Failed to parse acc: {e}, line: {line}')
                return None, None, None
            self.latest_accels[imu_num] = accel
            return imu_num, None, accel

        if payload.startswith('quat:'):
            data_str = payload[5:].strip()
            try:
                quat = self.parse_vector(data_str, 4)
            except ValueError as e:
                self.get_logger().warning(f'Failed to parse quat: {e}, line: {line}')
                return None, None, None
            self.latest_quaternions[imu_num] = quat
            return imu_num, quat, None

        try:
            quat = self.parse_vector(payload, 4)
        except ValueError:
            return None, None, None
        self.latest_quaternions[imu_num] = quat
        return imu_num, quat, None

    def process_line(self, line):
        if line.startswith('[IMU'):
            end_bracket = line.find(']')
            if end_bracket == -1:
                return None, None, None
            imu_id = line[4:end_bracket]
            if not imu_id.isdigit():
                return None, None, None
            imu_num = int(imu_id)
            if not (1 <= imu_num <= self.num_imus):
                return None, None, None

            self.current_imu = imu_num
            payload = line[end_bracket + 1:].lstrip(':').strip()
            if not payload:
                return None, None, None
            return self.process_payload(imu_num, payload, line)

        if self.current_imu is None:
            return None, None, None

        return self.process_payload(self.current_imu, line, line)

    def create_imu_msg(self, imu_num, quaternion, accel=None):
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.frame_id}_imu{imu_num}'

        msg.orientation = Quaternion()
        msg.orientation.x = quaternion[1]
        msg.orientation.y = quaternion[2]
        msg.orientation.z = quaternion[3]
        msg.orientation.w = quaternion[0]

        msg.angular_velocity = Vector3()
        msg.angular_velocity_covariance[0] = -1

        msg.linear_acceleration = Vector3()
        if accel is not None:
            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]
        msg.linear_acceleration_covariance[0] = -1

        msg.orientation_covariance[0] = -1

        return msg

    def create_acc_msg(self, imu_num, accel):
        msg = Vector3Stamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.frame_id}_imu{imu_num}'

        msg.vector = Vector3()
        msg.vector.x = accel[0]
        msg.vector.y = accel[1]
        msg.vector.z = accel[2]

        return msg

    def read_serial_data(self):
        try:
            if self.serial and self.serial.in_waiting:
                data = self.serial.read(self.serial.in_waiting)
                ascii_data = data.decode('ascii', errors='ignore')

                lines = self.parse_imu_data(ascii_data)

                for line in lines:
                    imu_num, quat, accel = self.process_line(line)
                    if not imu_num:
                        continue

                    if accel is not None:
                        acc_msg = self.create_acc_msg(imu_num, accel)
                        self.acc_publishers[imu_num].publish(acc_msg)
                        self.acc_counters[imu_num] += 1
                        if self.verbose:
                            self.get_logger().debug(
                                f'IMU{imu_num} acc: '
                                f'x={accel[0]:.3f}, y={accel[1]:.3f}, z={accel[2]:.3f}'
                            )

                    if quat is not None:
                        acc_for_imu = self.latest_accels.get(imu_num)
                        imu_msg = self.create_imu_msg(imu_num, quat, acc_for_imu)
                        self.imu_publishers[imu_num].publish(imu_msg)
                        self.imu_counters[imu_num] += 1
                        if self.verbose:
                            self.get_logger().debug(
                                f'IMU{imu_num} quat: '
                                f'w={quat[0]:.3f}, x={quat[1]:.3f}, '
                                f'y={quat[2]:.3f}, z={quat[3]:.3f}'
                            )

                current_time = time.time()
                if current_time - self.last_log_time > 3.0:
                    self.last_log_time = current_time

                    log_msg = "Publish frequencies (imu/acc): "
                    for i in range(1, self.num_imus + 1):
                        imu_freq = self.imu_counters[i] / 3.0
                        acc_freq = self.acc_counters[i] / 3.0
                        log_msg += f'IMU{i}: {imu_freq:.1f}Hz/{acc_freq:.1f}Hz '
                        self.imu_counters[i] = 0
                        self.acc_counters[i] = 0
                    self.get_logger().info(log_msg)

                    quat_msg = "Latest quaternions:\n"
                    for i in range(1, self.num_imus + 1):
                        q = self.latest_quaternions[i]
                        quat_msg += (
                            f'  IMU{i}: '
                            f'w={q[0]:.3f}, x={q[1]:.3f}, '
                            f'y={q[2]:.3f}, z={q[3]:.3f}\n'
                        )
                    self.get_logger().info(quat_msg)

                    acc_msg = "Latest accelerations:\n"
                    for i in range(1, self.num_imus + 1):
                        a = self.latest_accels[i]
                        acc_msg += (
                            f'  IMU{i}: '
                            f'x={a[0]:.3f}, y={a[1]:.3f}, z={a[2]:.3f}\n'
                        )
                    self.get_logger().info(acc_msg)

        except serial.SerialException as e:
            self.get_logger().error(f'Serial communication error: {e}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error in read_serial_data: {e}')

    def destroy_node(self):
        if hasattr(self, 'serial') and self.serial and self.serial.is_open:
            self.serial.close()
            self.get_logger().info('Serial port closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = LegIMUNode()
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
