import serial
import time
import threading
import collections
import numpy as np

class KalmanFilter:
    """
    一个简单的一维卡尔曼滤波器，用于平滑高度测量数据。
    """
    def __init__(self, process_variance=1e-5, measurement_variance=1e-1, estimate_error=1.0, initial_value=0):
        self.process_variance = process_variance     # 过程噪声协方差 Q
        self.measurement_variance = measurement_variance # 测量噪声协方差 R
        self.estimate_error = estimate_error         # 估计误差协方差 P
        self.current_estimate = initial_value        # 当前估计值
        self.kalman_gain = 0                         # 卡尔曼增益 K

    def update(self, measurement):
        # 预测更新
        # 假设物体高度变化是平滑的，预测值等于上一次的估计值
        prediction = self.current_estimate
        prediction_error = self.estimate_error + self.process_variance

        # 测量更新
        self.kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.current_estimate = prediction + self.kalman_gain * (measurement - prediction)
        self.estimate_error = (1 - self.kalman_gain) * prediction_error

        return self.current_estimate

class LaserSensorReader:
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.running = False
        self.latest_raw_height = 0.0
        self.latest_filtered_height = 0.0
        self.lock = threading.Lock()
        
        # 初始化滤波器
        # process_variance: 调小表示我们相信模型（高度变化平滑），调大表示相信系统状态变化快
        # measurement_variance: 调大表示传感器噪声大，滤波器会更平滑但滞后
        self.kf = KalmanFilter(process_variance=1e-4, measurement_variance=0.1)

    def connect(self):
        try:
            # timeout=0.1 允许我们在循环中检查退出条件
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=0.1)
            print(f"已连接到传感器: {self.port}")
            return True
        except serial.SerialException as e:
            print(f"无法连接串口: {e}")
            return False

    def _parse_data(self, raw_line):
        """
        解析传感器数据。这里假设传感器发送的是类似 "D: 123.45\n" 的 ASCII 格式。
        根据实际传感器协议（如 Modbus, Hex 等）需要修改此函数。
        """
        try:
            # 示例：假设数据是 "123.45" 格式的字符串
            line = raw_line.decode('utf-8').strip()
            # 简单的模拟解析，移除可能的非数字字符
            # 实际情况请根据传感器手册编写
            if not line:
                return None
            return float(line)
        except ValueError:
            return None
        except Exception as e:
            # print(f"解析错误: {e}")
            return None

    def start_reading(self):
        if not self.serial_conn:
            if not self.connect():
                return

        self.running = True
        self.thread = threading.Thread(target=self._read_loop)
        self.thread.daemon = True
        self.thread.start()

    def _read_loop(self):
        while self.running and self.serial_conn.is_open:
            try:
                line = self.serial_conn.readline()
                if line:
                    height = self._parse_data(line)
                    if height is not None:
                        # 应用滤波
                        filtered = self.kf.update(height)
                        
                        with self.lock:
                            self.latest_raw_height = height
                            self.latest_filtered_height = filtered
                            
                        # 可以在这里打印，或者在主线程获取
                        # print(f"Raw: {height:.3f}, Filtered: {filtered:.3f}")
            except Exception as e:
                print(f"读取循环错误: {e}")
                time.sleep(0.1)

    def get_height(self):
        with self.lock:
            return self.latest_raw_height, self.latest_filtered_height

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
        if self.serial_conn:
            self.serial_conn.close()
        print("传感器读取已停止")

# 模拟数据生成器（用于没有真实硬件时的测试）
def mock_sensor_simulation():
    print("注意：正在运行模拟模式（无真实硬件连接）")
    reader = LaserSensorReader()
    # 覆盖 _read_loop 进行模拟
    
    t = 0
    try:
        while True:
            # 模拟真实高度：正弦波运动
            true_height = 100 + 10 * np.sin(t)
            # 模拟噪声：添加随机干扰
            noise = np.random.normal(0, 2.0) # 标准差为2的噪声
            measured_height = true_height + noise
            
            # 滤波
            filtered = reader.kf.update(measured_height)
            
            # 简单的可视化输出
            bar_len = int(filtered) // 2
            print(f"Time: {t:.2f} | Raw: {measured_height:6.2f} | Filtered: {filtered:6.2f} | Error: {abs(filtered - true_height):.2f}")
            
            t += 0.1
            time.sleep(0.05) # 20Hz 模拟频率
    except KeyboardInterrupt:
        print("\n模拟结束")

if __name__ == "__main__":
    # 如果有真实硬件，请取消注释下面几行并修改端口号
    # sensor = LaserSensorReader(port='/dev/ttyUSB0', baudrate=115200)
    # sensor.start_reading()
    # try:
    #     while True:
    #         raw, filt = sensor.get_height()
    #         print(f"高度: {filt:.4f} mm")
    #         time.sleep(0.01)
    # except KeyboardInterrupt:
    #     sensor.stop()

    # 运行模拟演示
    mock_sensor_simulation()
