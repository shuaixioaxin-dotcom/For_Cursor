# IMU数据读取频率优化指南

## 目标
将IMU数据读取频率从当前水平优化到 **200Hz+**

## 理论分析

### 时间预算计算（200Hz = 5ms/周期）
- 2个IMU，每个IMU需要在 **2.5ms** 内完成一次请求-响应

### RS485通信时间（921600波特率）
| 项目 | 字节数 | 时间 |
|------|--------|------|
| 请求帧 | 8字节 | 87μs |
| 响应帧 | 49字节 | 532μs |
| **单个IMU总计** | 57字节 | **619μs** |
| **2个IMU总计** | 114字节 | **1.24ms** |

### 理论极限
- 纯通信时间：1.24ms
- 剩余时间预算：5ms - 1.24ms = **3.76ms**
- 理论上可以达到 **400Hz+**

## 原代码主要瓶颈

### 1. Serial.print() 输出瓶颈 ⚠️ **最大问题**
```cpp
// 原代码：每个周期都调用Serial.print()输出CSV
Serial.print(imu_data[i].acc[0], 3);  // 浮点数转字符串非常慢！
```
**问题**：`Serial.print(float)` 内部调用 `dtostrf()`，转换一个浮点数约需 100-200μs

**影响**：每周期输出约14个浮点数 → **2-3ms 仅用于串口输出！**

### 2. 总线静默时间过长
```cpp
static const uint32_t BUS_SILENCE_US = 500;  // 原代码500μs
```
**问题**：Modbus规范仅要求3.5字符时间（921600波特率下约38μs）

### 3. RS485方向切换延迟过长
```cpp
static const uint32_t TX_ENABLE_DELAY_US = 30;
static const uint32_t TX_DISABLE_DELAY_US = 60;
```
**问题**：MAX485等芯片典型切换时间 < 10μs

### 4. 超时时间过长
```cpp
static const uint16_t RESPONSE_TIMEOUT_MS = 30;  // 30ms太长
```
**问题**：实际响应通常在1-2ms内完成，30ms超时浪费时间

### 5. CRC计算效率低
原代码使用逐位计算，每字节需要8次循环

## 优化方案

### 优化1：使用二进制输出替代CSV
```cpp
// 二进制包结构
struct BinaryPacket {
  uint8_t header;      // 0xAA
  uint8_t imu_count;
  uint32_t timestamp;
  struct { ... } imu[NUM_IMUS];
  uint8_t checksum;
  uint8_t footer;      // 0x55
};

// 直接写入二进制数据
Serial.write((uint8_t*)&pkt, sizeof(pkt));
```
**效果**：输出时间从 2-3ms 降低到 < 100μs

### 优化2：降低总线静默时间
```cpp
static const uint32_t BUS_SILENCE_US = 50;  // 50μs足够
```

### 优化3：降低RS485方向切换延迟
```cpp
static const uint32_t TX_ENABLE_DELAY_US = 5;
static const uint32_t TX_DISABLE_DELAY_US = 10;
```

### 优化4：降低超时时间
```cpp
static const uint16_t RESPONSE_TIMEOUT_US = 2000;  // 2ms
```

### 优化5：使用CRC查表法
```cpp
// 256字节查找表，每字节仅需1次查表
static inline uint16_t crc16_modbus_fast(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc = (crc >> 8) ^ pgm_read_word(&crc16_table[(crc ^ *data++) & 0xFF]);
  }
  return crc;
}
```

### 优化6：预缓存请求帧
```cpp
static uint8_t cached_requests[NUM_IMUS][MODBUS_REQUEST_LEN];

// setup()中预构建
static void buildAllRequests() {
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    // 构建并缓存每个IMU的请求帧
  }
}
```

### 优化7：输出降频
```cpp
// 每N个周期输出一次数据
static const uint8_t OUTPUT_DECIMATION = 1;  // 设为2则每2个周期输出一次
```

## 预期性能

| 配置 | 预期频率 |
|------|----------|
| 二进制输出 + 全部优化 | **400-500Hz** |
| CSV输出 + sprintf缓冲 | **250-300Hz** |
| CSV输出 + 降频(1/2) | **400Hz采样, 200Hz输出** |

## 硬件优化建议

### 1. 确保RS485芯片性能
- 使用 MAX485 / SP485 等高速芯片
- 检查芯片数据手册中的 DE/RE 切换时间

### 2. 检查接线
- 使用屏蔽双绞线
- 总线两端加120Ω终端电阻
- 保持线缆短（<10m 最佳）

### 3. IMU设备端优化
- 确认IMU固件响应延迟
- 如果IMU支持，使用广播读取（非标准Modbus）

## 使用方法

### 切换输出模式
```cpp
// 二进制输出（最高性能）
static const bool USE_BINARY_OUTPUT = true;

// CSV输出（方便调试）
static const bool USE_BINARY_OUTPUT = false;
```

### 调整输出频率
```cpp
// 每个周期都输出
static const uint8_t OUTPUT_DECIMATION = 1;

// 每2个周期输出一次（采样率不变，输出减半）
static const uint8_t OUTPUT_DECIMATION = 2;
```

### 二进制数据解析示例（Python）
```python
import struct
import serial

# 包格式
HEADER = 0xAA
FOOTER = 0x55
NUM_IMUS = 2
PACKET_SIZE = 1 + 1 + 4 + NUM_IMUS * (1 + 1 + 6 + 8) + 1 + 1  # 38字节

def parse_packet(data):
    if len(data) != PACKET_SIZE:
        return None
    if data[0] != HEADER or data[-1] != FOOTER:
        return None
    
    offset = 2
    timestamp = struct.unpack('<I', data[offset:offset+4])[0]
    offset += 4
    
    imus = []
    for i in range(NUM_IMUS):
        imu_id = data[offset]
        valid = data[offset + 1]
        acc = struct.unpack('>3h', data[offset+2:offset+8])
        quat = struct.unpack('>4h', data[offset+8:offset+16])
        offset += 16
        
        imus.append({
            'id': imu_id,
            'valid': bool(valid),
            'acc': [a * 0.0048828 for a in acc],
            'quat': [q * 0.0001 for q in quat]
        })
    
    return {'timestamp': timestamp, 'imus': imus}

# 使用示例
ser = serial.Serial('/dev/ttyUSB0', 2000000)
buffer = b''

while True:
    buffer += ser.read(ser.in_waiting or 1)
    
    # 查找包头
    start = buffer.find(bytes([HEADER]))
    if start >= 0 and len(buffer) >= start + PACKET_SIZE:
        packet = buffer[start:start + PACKET_SIZE]
        result = parse_packet(packet)
        if result:
            print(result)
        buffer = buffer[start + PACKET_SIZE:]
```

## 故障排除

### 频率仍然低于200Hz
1. 检查串口监视器中的频率报告
2. 查看 `fail` 计数是否过高
3. 尝试增加 `RESPONSE_TIMEOUT_US`
4. 使用示波器检查RS485信号质量

### 数据错误率高
1. 增加 `BUS_SILENCE_US` 到 100-200μs
2. 增加 `TX_DISABLE_DELAY_US`
3. 检查终端电阻和接线

### IMU无响应
1. 确认IMU地址配置正确
2. 检查波特率设置一致
3. 用单个IMU测试

## 总结

达到200Hz的关键优化点：
1. ✅ **二进制输出** - 最重要的优化
2. ✅ **降低延迟参数** - 累计节省数百微秒
3. ✅ **CRC查表法** - 提高10倍以上
4. ✅ **预缓存请求** - 消除重复计算
5. ✅ **微秒级超时** - 减少等待时间
