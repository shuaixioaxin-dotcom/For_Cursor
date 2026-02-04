# IMU (ACC/QUAT) Modbus RTU 优化读取器

## 概述

这是一个优化的IMU数据读取器，采用批量处理方式读取多个Modbus RTU从站的加速度(ACC)和四元数(QUAT)数据。

## 优化策略

参照 `OptimizedEncoderReader` 的三阶段处理逻辑：

### 第一阶段：批量发送请求
- 为所有从站构建Modbus请求帧
- 快速连续发送所有请求
- 发送间隔仅0.001秒

### 第二阶段：批量读取响应
- 等待所有设备响应（0.001秒）
- 一次性读取串口缓冲区中的所有数据
- 避免多次读取的开销

### 第三阶段：解析响应
- 在接收到的数据流中定位每个响应帧
- 验证从站地址、功能码和CRC校验
- 解析ACC和QUAT数据并更新缓存

## 主要特性

1. **批量处理**：同时处理多个从站的数据请求
2. **线程安全**：使用锁机制保护并发访问
3. **自动重连**：发生错误时自动重新连接
4. **数据缓存**：保存最新的IMU数据
5. **CRC校验**：确保数据完整性

## 使用方法

### 基本用法

```bash
# 读取单个从站
python optimized_imu_reader.py -p COM66 -b 921600 -i 2

# 读取多个从站
python optimized_imu_reader.py -p COM66 -b 921600 -i "2,3,4,5"

# 指定读取频率
python optimized_imu_reader.py -p COM66 -b 921600 -i 2 -f 100

# 读取指定次数
python optimized_imu_reader.py -p COM66 -b 921600 -i 2 -c 100
```

### 参数说明

- `-p, --port`: 串口端口（默认COM66）
- `-b, --baudrate`: 波特率（默认921600）
- `-i, --ids`: 从站ID列表，逗号分隔（默认2）
- `-f, --freq`: 读取频率Hz（0=最高频率）
- `-c, --count`: 读取次数（0=无限循环）

### 在代码中使用

```python
from optimized_imu_reader import OptimizedIMUReader

# 创建读取器
reader = OptimizedIMUReader(
    port='COM66',
    baudrate=921600,
    slave_ids=[2, 3, 4]
)

# 连接
reader.connect()

# 批量读取所有IMU数据
imu_data = reader.read_all_imu_optimized()

# 访问特定从站的数据
acc = imu_data[2]['acc']      # (x, y, z) in m/s²
quat = imu_data[2]['quat']    # (w, x, y, z)

# 断开连接
reader.disconnect()
```

## 数据格式

### Modbus请求帧
- 从站地址: 1字节
- 功能码: 0x03（读保持寄存器）
- 起始地址: 0x0034
- 寄存器数量: 0x0016（22个寄存器）
- CRC校验: 2字节

### Modbus响应帧（49字节）
- 从站地址: 1字节
- 功能码: 0x03
- 字节数: 1字节（0x2C = 44字节数据）
- 数据: 44字节
  - 加速度数据（偏移3-8）: 3个16位无符号整数
  - 四元数数据（偏移41-48）: 4个16位有符号整数
- CRC校验: 2字节

## 性能优势

相比原始的单次请求-响应模式：

1. **减少通信开销**：批量发送和接收减少了串口读写次数
2. **提高响应速度**：并行处理多个设备的请求
3. **降低延迟**：减少等待时间，提高整体吞吐量
4. **更好的可扩展性**：轻松支持更多从站设备

## 注意事项

1. 确保所有从站ID在Modbus网络中唯一
2. 根据实际设备数量和网络状况调整等待时间
3. 高波特率下需要更短的等待时间
4. 建议在实际环境中测试并调优参数

## 与原始代码的对比

### 原始代码（单次模式）
```python
# 每次读取一个设备
ser.write(request)
response = ser.read(RESPONSE_LEN)
parse_response(response)
```

### 优化代码（批量模式）
```python
# 批量发送所有请求
for frame in requests:
    ser.write(frame)

# 批量读取所有响应
total_response = ser.read(ser.in_waiting)

# 批量解析所有响应
parse_responses(total_response)
```

## 技术细节

### CRC16计算
使用查表法计算Modbus RTU CRC16校验值，提高计算效率。

### 数据转换
- **加速度**：16位无符号整数 → 有符号值 → m/s²（比例因子：0.0048828）
- **四元数**：16位有符号整数 → 归一化浮点数（比例因子：0.0001）

### 响应帧解析
1. 搜索有效的从站地址
2. 验证响应帧长度（49字节）
3. 检查功能码（0x03）
4. 验证CRC校验
5. 提取并转换ACC和QUAT数据

## 许可证

本代码遵循原项目许可证。
