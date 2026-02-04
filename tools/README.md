# IMU数据接收工具

本目录包含用于接收和解析ESP32发送的IMU数据的Python工具。

## 工具列表

### 1. text_reader.py
用于接收文本格式的IMU数据。

**使用方法**：
```bash
# 基本使用
python text_reader.py /dev/ttyUSB0

# 简洁模式
python text_reader.py /dev/ttyUSB0 --quiet

# 接收100行后停止
python text_reader.py /dev/ttyUSB0 --max-lines 100
```

### 2. binary_reader.py
用于接收二进制格式的IMU数据（更高效）。

**使用方法**：
```bash
# 基本使用
python binary_reader.py /dev/ttyUSB0

# 简洁模式
python binary_reader.py /dev/ttyUSB0 --quiet

# 接收100帧后停止
python binary_reader.py /dev/ttyUSB0 --max-frames 100
```

## 安装依赖

```bash
pip install -r requirements.txt
```

或手动安装：
```bash
pip install pyserial
```

## 示例输出

### 文本模式
```
[12:34:56.789]
  IMU1 ✓ | Acc:(  1.234,  5.678,  9.012) | Quat:( 0.9998, 0.0123, 0.0456, 0.0789)
  IMU2 ✓ | Acc:(  2.345,  6.789,  0.123) | Quat:( 0.9997, 0.0234, 0.0567, 0.0890)
```

### 二进制模式（详细）
```
============================================================
时间: 12:34:56.789 | 批次: 4组
============================================================

样本 #1:
  IMU1 ✓
    加速度: (  1.234,   5.678,   9.012) m/s²
    四元数: ( 0.9998,  0.0123,  0.0456,  0.0789)
  IMU2 ✓
    加速度: (  2.345,   6.789,   0.123) m/s²
    四元数: ( 0.9997,  0.0234,  0.0567,  0.0890)
...
```

## 参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| port | 串口设备路径 | 必需 |
| -b, --baudrate | 波特率 | 921600 |
| -n, --num-imus | IMU数量 | 2 |
| -q, --quiet | 简洁输出模式 | false |
| -m, --max-lines/frames | 最大接收数 | 无限 |

## 性能测试

使用这些工具可以验证实际采集频率：

```bash
# 接收1000行后显示统计
python text_reader.py /dev/ttyUSB0 --max-lines 1000 --quiet
```

输出示例：
```
============================================================
统计信息:
  运行时间: 5.0秒
  接收行数: 1000
  错误次数: 0
  数据率: 200.0 行/秒
  错误率: 0.00%
============================================================
```

## 故障排查

### 问题：找不到串口设备

**Linux**：
```bash
# 查看可用串口
ls /dev/ttyUSB* /dev/ttyACM*

# 添加用户到dialout组
sudo usermod -a -G dialout $USER
# 注销后重新登录
```

**Windows**：
```bash
# 在设备管理器中查看COM端口号
```

### 问题：权限拒绝

```bash
# Linux
sudo chmod 666 /dev/ttyUSB0
```

### 问题：数据解析错误

1. 检查ESP32代码中的 `OUTPUT_BINARY` 配置与使用的工具是否匹配
2. 确认波特率设置正确（默认921600）
3. 检查IMU数量配置是否一致
