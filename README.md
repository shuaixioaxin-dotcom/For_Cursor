# For_Cursor
For testing of Cursor

## 串口响应频率测试脚本

### 功能说明
用于测试 Modbus RTU 设备的最高响应频率。

**发送帧格式：**
```
01 03 00 01 00 01 D5 CA
```
- `01`: 从站地址
- `03`: 功能码（读取保持寄存器）
- `00 01`: 起始寄存器地址
- `00 01`: 读取寄存器数量
- `D5 CA`: CRC校验

**接收帧示例：**
```
01 03 02 00 00 B8 44
```
- `01`: 从站地址
- `03`: 功能码
- `02`: 数据字节数
- `00 00`: 寄存器数据
- `B8 44`: CRC校验

### 安装依赖
```bash
pip install -r requirements.txt
```

### 使用方法

**列出可用串口：**
```bash
python serial_frequency_test.py --list
```

**基本测试：**
```bash
python serial_frequency_test.py --port /dev/ttyUSB0
python serial_frequency_test.py --port COM3  # Windows
```

**自定义参数：**
```bash
# 指定波特率
python serial_frequency_test.py --port /dev/ttyUSB0 --baudrate 115200

# 增加测试轮数和每轮请求数
python serial_frequency_test.py --port /dev/ttyUSB0 --rounds 10 --requests 200

# 完整参数
python serial_frequency_test.py --port /dev/ttyUSB0 --baudrate 9600 --timeout 0.1 --rounds 5 --requests 100 --parity N --stopbits 1
```

### 参数说明
| 参数 | 简写 | 默认值 | 说明 |
|------|------|--------|------|
| --port | -p | - | 串口端口号（必填） |
| --baudrate | -b | 9600 | 波特率 |
| --timeout | -t | 0.1 | 读取超时(秒) |
| --rounds | -r | 5 | 测试轮数 |
| --requests | -n | 100 | 每轮请求次数 |
| --parity | - | N | 校验位(N/E/O) |
| --stopbits | - | 1 | 停止位(1/2) |
| --list | -l | - | 列出可用串口 |

### 测试结果
脚本会输出：
- 每次请求的响应时间
- 成功/失败统计
- 最小/最大/平均响应时间
- **理论最高响应频率 (Hz)**

测试结果会自动保存到 `serial_test_result_YYYYMMDD_HHMMSS.txt` 文件中。
