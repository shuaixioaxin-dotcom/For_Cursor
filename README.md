# Modbus IMU 频率优化方案

## 目标
将2个IMU的Modbus RTU读取频率从当前水平优化到200Hz。

## 时间预算分析

### 目标频率计算
- **目标**: 200Hz = 5ms/周期
- **2个IMU**: 每个IMU需要在 2.5ms 内完成通信

### 原始代码时序分析

在921600波特率下，每字节传输时间约为 10.85μs (10位/字节):

| 项目 | 原始值 | 耗时 |
|------|--------|------|
| 请求帧 (8字节) | - | ~87μs |
| 响应帧 (49字节) | - | ~532μs |
| TX_ENABLE_DELAY | 30μs | 30μs |
| TX_DISABLE_DELAY | 60μs | 60μs |
| **BUS_SILENCE** | **500μs** | **500μs** ← 主要瓶颈 |
| RESPONSE_TIMEOUT | 30ms | (最坏情况) |

**单IMU周期估算**: 30 + 87 + 60 + ~500 + 500 ≈ **1.2ms**
**两个IMU理论频率**: 1000 / (1.2 × 2) ≈ **416Hz** (理论上限)

### 实际瓶颈
1. `BUS_SILENCE_US = 500μs` - 过于保守
2. `Serial.print()` 多次调用 - 串口输出阻塞
3. `RESPONSE_TIMEOUT_MS = 30ms` - 超时过长导致错误恢复慢
4. CRC计算使用循环 - 可用查表法加速
5. 每次请求都重新构建帧 - 可预缓存

## 优化方案

### 1. 时序参数优化

| 参数 | 原始值 | 优化值 | 说明 |
|------|--------|--------|------|
| BUS_SILENCE_US | 500μs | **50μs** | Modbus最小静默时间 @921600 ≈ 38μs |
| TX_ENABLE_DELAY_US | 30μs | **10μs** | RS485芯片典型响应 <1μs |
| TX_DISABLE_DELAY_US | 60μs | **20μs** | 确保最后字节发送完成 |
| RESPONSE_TIMEOUT_MS | 30ms | **5ms** | 49字节@921600仅需~532μs |
| FAIL_COOLDOWN_MS | 20ms | **5ms** | 更快的故障恢复 |
| MAX_BACKOFF_SHIFT | 4 | **2** | 减少最大退避时间 |

### 2. 代码优化

#### CRC16查找表
```cpp
// 使用256字节查找表,比循环计算快约10倍
static const uint16_t crc16_table[256] PROGMEM = { ... };

static uint16_t crc16_modbus_fast(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    uint8_t index = (crc ^ data[i]) & 0xFF;
    crc = (crc >> 8) ^ pgm_read_word(&crc16_table[index]);
  }
  return crc;
}
```

#### 请求帧预缓存
```cpp
static uint8_t request_cache[NUM_IMUS][MODBUS_REQUEST_LEN];

// setup()中预构建所有请求帧
static void buildRequestCache() {
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    // 构建并缓存请求帧...
  }
}
```

#### CSV输出优化
```cpp
// 使用sprintf一次性构建,减少Serial.print调用次数
static char csv_buffer[256];
ptr += sprintf(ptr, "%d,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f", ...);
Serial.println(csv_buffer);

// 可选: 输出分频或完全禁用
static const bool DISABLE_CSV_OUTPUT = false;
static const uint8_t CSV_OUTPUT_DIVIDER = 1;  // 1=每周期, 5=每5周期
```

### 3. 串口缓冲区优化
```cpp
Serial2.setRxBufferSize(256);  // 增加RX缓冲区
```

## 优化后时序估算

| 项目 | 耗时 |
|------|------|
| TX使能延迟 | 10μs |
| 发送请求 | ~87μs |
| TX禁用延迟 | 20μs |
| 等待+接收响应 | ~600μs |
| 总线静默 | 50μs |
| 处理开销 | ~50μs |
| **单IMU总计** | **~820μs** |

**两个IMU周期**: 820μs × 2 ≈ **1.64ms**
**预期频率**: 1000 / 1.64 ≈ **610Hz**

## 配置选项

### 最高性能模式 (禁用CSV输出)
```cpp
static const bool DISABLE_CSV_OUTPUT = true;
```
预期可达 **400-600Hz**

### 平衡模式 (每5周期输出一次)
```cpp
static const bool DISABLE_CSV_OUTPUT = false;
static const uint8_t CSV_OUTPUT_DIVIDER = 5;
```
预期可达 **300-400Hz**

### 默认模式 (每周期输出)
```cpp
static const bool DISABLE_CSV_OUTPUT = false;
static const uint8_t CSV_OUTPUT_DIVIDER = 1;
```
预期可达 **200-300Hz**

## 进一步优化建议

如果仍需要更高频率,可考虑:

1. **使用DMA传输** - 硬件DMA可实现零CPU占用的串口通信
2. **中断驱动接收** - 使用RX中断而非轮询
3. **减少寄存器读取数量** - 如果不需要全部22个寄存器
4. **双缓冲** - 一个IMU通信时处理另一个的数据
5. **提高波特率** - 如果硬件支持更高波特率

## 文件说明

- `modbus_imu_optimized.ino` - 优化后的完整代码
- `README.md` - 本说明文档

## 使用方法

1. 将 `modbus_imu_optimized.ino` 上传到ESP32
2. 根据需要调整配置参数
3. 观察串口输出的频率报告

```
# Freq: 245.3Hz (ok=490 fail=0)
```
