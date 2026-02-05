# IMU 数据读取频率优化指南 - 目标200Hz

## 时间开销分析

### 原始代码瓶颈

| 参数 | 原始值 | 时间开销 |
|------|--------|----------|
| BUS_SILENCE_US | 500μs | 0.5ms/次 |
| TX_ENABLE_DELAY_US | 30μs | 微小 |
| TX_DISABLE_DELAY_US | 60μs | 微小 |
| RESPONSE_TIMEOUT_MS | 30ms | 超时时浪费大量时间 |
| FAIL_COOLDOWN_MS | 20ms | 失败后跳过IMU |
| Serial.print() CSV输出 | ~1-2ms | **主要瓶颈** |

### 串口传输时间计算 (921600 baud)

```
每字节时间 = 10 bits / 921600 baud = 10.85μs

发送请求 (8字节):  8 × 10.85μs = 87μs
接收响应 (49字节): 49 × 10.85μs = 532μs
单次通信理论最小时间: ~620μs
```

### 200Hz目标分析

```
周期 = 1000ms / 200Hz = 5ms
每个IMU可用时间 = 5ms / 2 = 2.5ms

时间预算:
- RS485通信: ~0.7ms
- 数据处理: ~0.1ms  
- 串口输出: ~0.5ms (优化后)
- 裕量: ~1.2ms
```

## 优化策略

### 1. 减少延迟参数

```cpp
// 优化前 -> 优化后
RESPONSE_TIMEOUT_MS:  30ms -> 3ms   // IMU响应通常<1ms
BUS_SILENCE_US:       500μs -> 100μs // Modbus最小3.5字符≈38μs@921600
TX_ENABLE_DELAY_US:   30μs -> 10μs
TX_DISABLE_DELAY_US:  60μs -> 20μs
FAIL_COOLDOWN_MS:     20ms -> 5ms
MAX_BACKOFF_SHIFT:    4 -> 2        // 减少最大退避时间
```

### 2. CRC查表法 (性能提升~4倍)

```cpp
// 原始: 循环计算 ~15μs
// 优化: 查表法 ~4μs
static const uint16_t crc_table[256] PROGMEM = { ... };
static uint16_t crc16_modbus_fast(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc = (crc >> 8) ^ pgm_read_word(&crc_table[(crc ^ data[i]) & 0xFF]);
  }
  return crc;
}
```

### 3. 预计算请求帧

```cpp
// 在setup()中预计算所有IMU的请求帧
static uint8_t request_cache[NUM_IMUS][MODBUS_REQUEST_LEN];

void buildAllRequests() {
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    // 预先构建完整请求帧，包括CRC
  }
}

// loop()中直接发送缓存的请求
Serial2.write(request_cache[imu_idx], MODBUS_REQUEST_LEN);
```

### 4. 优化串口输出

**问题**: 原始代码使用多次 `Serial.print()` 调用，每次都会阻塞。

**方案A**: 使用缓冲区批量输出
```cpp
static char output_buffer[256];
int pos = snprintf(output_buffer, sizeof(output_buffer),
                   "%d,%.3f,%.3f,...", id, ax, ay, ...);
Serial.write(output_buffer, pos);  // 单次写入
```

**方案B**: 使用二进制格式 (更快)
```cpp
#pragma pack(push, 1)
struct BinaryFrame {
  uint8_t header[2];  // 0xAA, 0x55
  uint8_t imu_count;
  uint32_t timestamp_ms;
  // ... 原始int16数据 ...
  uint8_t checksum;
};
#pragma pack(pop)
Serial.write((uint8_t*)&frame, sizeof(frame));
```

### 5. 使用微秒级超时

```cpp
// 原始: 毫秒精度
if (millis() - request_start_ms > RESPONSE_TIMEOUT_MS) { ... }

// 优化: 微秒精度，响应更快
if ((micros() - request_start_us) > (RESPONSE_TIMEOUT_MS * 1000UL)) { ... }
```

### 6. 增大Serial2缓冲区

```cpp
Serial2.setRxBufferSize(256);  // 默认64字节可能不够
Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
```

## 进阶优化 (如果仍不够)

### A. ESP32双核并行处理

```cpp
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Core 0: RS485通信
void rs485Task(void *param) {
  for (;;) {
    // 处理Modbus通信
    vTaskDelay(1);  // 让出CPU
  }
}

// Core 1: 数据输出
void outputTask(void *param) {
  for (;;) {
    // 处理串口输出
    vTaskDelay(1);
  }
}

void setup() {
  xTaskCreatePinnedToCore(rs485Task, "RS485", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(outputTask, "Output", 4096, NULL, 1, NULL, 1);
}
```

### B. DMA传输 (需要ESP-IDF)

```cpp
// 使用ESP-IDF的UART DMA模式
uart_driver_install(UART_NUM_2, 256, 256, 10, &uart_queue, 0);
```

### C. 减少数据量

```cpp
// 如果不需要所有数据，减少寄存器数量
static const uint16_t MODBUS_REG_COUNT = 0x0003;  // 只读3个寄存器(加速度)
```

### D. 提高波特率

```cpp
// 如果IMU支持，可以尝试更高波特率
static const uint32_t RS485_BAUD = 1500000;  // 1.5Mbps
```

## 参数对照表

| 参数 | 原始值 | 优化值 | 效果 |
|------|--------|--------|------|
| RESPONSE_TIMEOUT_MS | 30 | 3 | 超时响应更快 |
| BUS_SILENCE_US | 500 | 100 | 减少空闲等待 |
| TX_ENABLE_DELAY_US | 30 | 10 | 减少切换延迟 |
| TX_DISABLE_DELAY_US | 60 | 20 | 减少切换延迟 |
| FAIL_COOLDOWN_MS | 5 | 5 | 快速重试 |
| MAX_BACKOFF_SHIFT | 4 | 2 | 最大退避40ms→20ms |
| CRC计算 | 循环 | 查表 | 4倍速度提升 |
| 请求帧 | 每次构建 | 预计算缓存 | 消除重复计算 |
| 串口输出 | 多次print | 批量write | 减少阻塞 |

## 预期效果

优化后理论最大频率:

```
单个IMU周期 ≈ 0.1ms(延迟) + 0.6ms(通信) + 0.1ms(处理) + 0.1ms(静默) = 0.9ms
2个IMU周期 ≈ 1.8ms + 0.5ms(输出) = 2.3ms
最大频率 ≈ 1000ms / 2.3ms ≈ 430Hz
```

考虑到实际抖动和裕量，稳定达到 **200Hz** 是可行的。

## 调试建议

1. 先注释掉CSV输出 (`OUTPUT_MODE_NONE`)，确认通信可达到目标频率
2. 逐步启用功能，观察频率变化
3. 使用示波器测量实际RS485波形时序
4. 检查IMU本身的数据更新率是否支持200Hz
