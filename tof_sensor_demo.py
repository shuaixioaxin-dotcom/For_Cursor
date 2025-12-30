import serial
import time
import struct
import threading
import numpy as np

class KalmanFilter:
    """
    简易卡尔曼滤波器（同前），用于进一步平滑数据
    """
    def __init__(self, process_variance=1e-4, measurement_variance=1e-2):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate_error = 1.0
        self.current_estimate = 0
        self.kalman_gain = 0
        self.first_run = True

    def update(self, measurement):
        if self.first_run:
            self.current_estimate = measurement
            self.first_run = False
        
        prediction = self.current_estimate
        prediction_error = self.estimate_error + self.process_variance
        
        self.kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.current_estimate = prediction + self.kalman_gain * (measurement - prediction)
        self.estimate_error = (1 - self.kalman_gain) * prediction_error
        
        return self.current_estimate

class TFLidarReader:
    """
    读取 TFmini / TF-Luna 等 ToF 激光雷达数据的类
    标准协议帧 (9 bytes):
    Byte0-1: 0x59 0x59 (Frame Header)
    Byte2: Dist_L
    Byte3: Dist_H (Distance = Dist_H*256 + Dist_L)
    Byte4: Strength_L
    Byte5: Strength_H
    Byte6: Temp_L
    Byte7: Temp_H
    Byte8: Checksum
    """
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.running = False
        self.distance = 0
        self.strength = 0
        self.lock = threading.Lock()
        self.kf = KalmanFilter(process_variance=0.01, measurement_variance=1.0) # 针对cm级跳动调整参数

    def connect(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"已连接 ToF 雷达: {self.port}")
            return True
        except serial.SerialException as e:
            print(f"串口连接失败: {e}")
            return False

    def start(self):
        if not self.serial_conn:
            if not self.connect():
                return
        self.running = True
        self.thread = threading.Thread(target=self._read_loop)
        self.thread.daemon = True
        self.thread.start()

    def _read_loop(self):
        # 缓冲区
        buffer = bytearray()
        
        while self.running and self.serial_conn.is_open:
            try:
                # 读取数据
                if self.serial_conn.in_waiting > 0:
                    chunk = self.serial_conn.read(self.serial_conn.in_waiting)
                    buffer.extend(chunk)
                
                # 处理缓冲区中的数据
                while len(buffer) >= 9:
                    # 寻找帧头 0x59 0x59
                    if buffer[0] != 0x59 or buffer[1] != 0x59:
                        # 如果不是帧头，弹出一个字节继续找
                        buffer.pop(0)
                        continue
                    
                    # 校验和检查
                    # Checksum is lower 8 bits of sum of first 8 bytes
                    checksum = sum(buffer[:8]) & 0xFF
                    if checksum != buffer[8]:
                        # 校验失败，可能是误判的帧头，丢弃第一个字节重新找
                        buffer.pop(0)
                        continue
                    
                    # 解析数据
                    dist = buffer[2] + buffer[3] * 256
                    strength = buffer[4] + buffer[5] * 256
                    # temp = buffer[6] + buffer[7] * 256 # 温度通常除以8-256...视具体型号而定，这里主要关注距离
                    
                    # 移除已处理的9个字节
                    del buffer[:9]
                    
                    # 过滤异常值 (0通常表示无效或超出量程)
                    if dist > 0 and strength > 0: # Strength过低通常意味着不可靠
                        with self.lock:
                            self.distance = dist # 单位通常是 cm
                            self.strength = strength
            except Exception as e:
                print(f"读取错误: {e}")
                time.sleep(0.1)
            
            time.sleep(0.001) # 极短休眠避免占满CPU

    def get_data(self):
        with self.lock:
            # 原始距离
            raw_dist = self.distance
            
        # 应用卡尔曼滤波
        filtered_dist = self.kf.update(raw_dist)
        return raw_dist, filtered_dist

    def stop(self):
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()

def mock_tof_simulation():
    print("=== ToF 激光雷达模拟模式 ===")
    print("模拟场景：物体在 100cm 处上下波动，模拟 50Hz 采样")
    
    kf = KalmanFilter(process_variance=0.1, measurement_variance=2.0)
    t = 0
    try:
        while True:
            # 模拟真实距离：100cm 基准 + 20cm 正弦波动
            true_dist = 100 + 20 * np.sin(t)
            
            # 模拟传感器噪声：ToF 传感器通常有 +/- 1cm 的跳动
            # 偶尔会有较大的突变（环境干扰）
            noise = np.random.normal(0, 1.0) 
            if np.random.random() > 0.95: # 5% 概率出现较大干扰
                noise += np.random.choice([-5, 5])
            
            measured_dist = true_dist + noise
            
            # 滤波
            filtered_dist = kf.update(measured_dist)
            
            # 可视化
            print(f"Raw: {measured_dist:6.1f} cm | Filtered: {filtered_dist:6.1f} cm | Signal: {'#'*int(filtered_dist/5)}")
            
            t += 0.2 # 模拟时间步进
            time.sleep(0.02) # 50Hz = 20ms
    except KeyboardInterrupt:
        print("停止模拟")

if __name__ == "__main__":
    # 使用示例：
    # lidar = TFLidarReader(port='/dev/ttyUSB0')
    # lidar.start()
    # while True:
    #     dist, filt = lidar.get_data()
    #     print(f"Distance: {dist} cm")
    #     time.sleep(0.02)
    
    # 运行模拟
    mock_tof_simulation()
