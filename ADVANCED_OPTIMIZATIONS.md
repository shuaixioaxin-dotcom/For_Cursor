# 进阶优化选项

## 方案 1: 最小寄存器读取（推荐用于极高频率）

如果你的应用只需要特定数据，可以减少读取的寄存器数量：

### 仅读取加速度和四元数（11个寄存器）

```cpp
// ================= Modbus RTU Configuration =================
static const uint8_t MODBUS_FUNC_READ_HREG = 0x03;
static const uint16_t MODBUS_REG_START = 0x0034;
static const uint16_t MODBUS_REG_COUNT = 0x000B;  // 11 registers instead of 22
```

**性能提升：**
- 响应长度: 47 字节 → 25 字节
- 传输时间: 510μs → 270μs (节省 240μs)
- 理论频率: 从 769Hz → **1200Hz+**
- 适用于: 只需要基础姿态数据的场景

---

## 方案 2: 2Mbps 高速通信

如果硬件支持（大多数 ESP32 和现代 IMU 支持）：

```cpp
// ================= Serial Configuration =================
static const uint32_t SERIAL_BAUD = 2000000;      // USB串口
static const uint32_t RS485_BAUD = 2000000;       // RS485 提升到 2Mbps

// 相应调整时序
static const uint32_t BUS_SILENCE_US = 18;        // 3.5 字符 @ 2Mbps
```

**性能提升：**
- 通信时间减半: 600μs → 300μs
- 2个IMU周期: 1.3ms → 0.7ms
- 理论频率: **1400Hz+**
- 适用于: 需要更多 IMU 或更高频率的场景

---

## 方案 3: 并行查询（多 RS485 端口）

如果 ESP32 有多个 UART 口连接不同的 RS485 总线：

```cpp
// 使用 Serial2 和 Serial1 同时查询不同的 IMU
// 可以实现真正的并行查询
HardwareSerial Serial1(1);  // UART1 for IMU 1
HardwareSerial Serial2(2);  // UART2 for IMU 2

void loop() {
  // 同时发送请求到两个总线
  sendRequest(Serial1, IMU_IDS[0]);
  sendRequest(Serial2, IMU_IDS[1]);
  
  // 并行接收响应
  // ...
}
```

**性能提升：**
- 查询时间: 1.3ms → 0.65ms（减半）
- 理论频率: **1500Hz+**
- 适用于: 大量 IMU 部署，可以分组到不同总线

---

## 方案 4: 二进制数据输出

替换 CSV 文本输出为二进制格式，减少序列化开销：

```cpp
static void outputCycleBinary() {
  // 输出标记字节
  Serial.write(0xFF);
  Serial.write(0xFE);
  
  // 输出数据（每个 float 4 字节）
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    Serial.write((uint8_t*)&imu_data[i].acc[0], 12);    // 3 floats
    Serial.write((uint8_t*)&imu_data[i].quat[0], 16);   // 4 floats
  }
  
  // 输出结束标记
  Serial.write(0xFD);
}
```

**性能提升：**
- CSV 文本: ~100 字节，需要格式化
- 二进制: 58 字节，无格式化开销
- 节省时间: ~200-500μs/周期
- 适用于: 上位机可以解析二进制的场景

---

## 方案 5: 使用 FreeRTOS 任务分离

将通信和数据输出分离到不同任务：

```cpp
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

QueueHandle_t imu_data_queue;

// 任务1: 高优先级 Modbus 通信
void modbusTask(void* param) {
  while (1) {
    // 仅负责 Modbus 查询和数据接收
    queryAllImus();
    xQueueSend(imu_data_queue, &imu_data, 0);
  }
}

// 任务2: 低优先级数据输出
void outputTask(void* param) {
  while (1) {
    if (xQueueReceive(imu_data_queue, &received_data, portMAX_DELAY)) {
      outputCycleCsv();
    }
  }
}

void setup() {
  // ...
  imu_data_queue = xQueueCreate(10, sizeof(ImuData) * NUM_IMUS);
  
  xTaskCreatePinnedToCore(modbusTask, "Modbus", 4096, NULL, 10, NULL, 1);
  xTaskCreatePinnedToCore(outputTask, "Output", 4096, NULL, 1, NULL, 0);
}
```

**优势：**
- 通信任务不被输出阻塞
- 更稳定的查询周期
- 可以缓冲数据防止丢失
- 适用于: 需要最大稳定性的场景

---

## 方案 6: DMA 传输（高级）

使用 ESP32 的 UART DMA 功能：

```cpp
#include <driver/uart.h>

void setupUartDma() {
  uart_config_t uart_config = {
    .baud_rate = 921600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
  };
  
  uart_param_config(UART_NUM_2, &uart_config);
  uart_set_pin(UART_NUM_2, RS485_TX_PIN, RS485_RX_PIN, -1, -1);
  
  // 安装带 DMA 的 UART 驱动
  uart_driver_install(UART_NUM_2, 1024, 1024, 0, NULL, 0);
}

void sendRequestDma(uint8_t slave_id) {
  uint8_t request[8];
  buildRequest(slave_id, request);
  
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  uart_write_bytes(UART_NUM_2, (const char*)request, 8);
  uart_wait_tx_done(UART_NUM_2, pdMS_TO_TICKS(1));
  digitalWrite(RS485_DE_RE_PIN, LOW);
}
```

**优势：**
- 减少 CPU 占用
- 在高频率下更稳定
- 可以同时处理其他任务
- 适用于: 复杂系统或高负载场景

---

## 方案 7: 减少 IMU 数据更新频率

如果 IMU 支持配置内部输出速率：

```cpp
// 通过 Modbus 配置 IMU 内部更新频率
// 寄存器地址取决于具体 IMU 型号
void configureImuRate(uint8_t slave_id, uint16_t rate_hz) {
  // 例如: 写入配置寄存器设置为 200Hz
  // writeRegister(slave_id, 0x001F, 200);
}
```

**策略：**
- 如果只需要 200Hz，将 IMU 内部配置为 200Hz
- 避免查询频率高于 IMU 更新频率（读取重复数据）
- 平衡功耗和性能

---

## 组合优化方案

### 极致性能配置（1000Hz+）
```cpp
// 组合方案 1 + 2 + 4 + 6
- 2Mbps 波特率
- 减少到 11 个寄存器
- 二进制输出
- DMA 传输
```

**预期结果：**
- 2个IMU: **1000-1500Hz**
- 4个IMU: **500-700Hz**
- 8个IMU: **250-350Hz**

### 平衡配置（200-300Hz）
```cpp
// 优化后的默认配置已足够
- 921600 波特率
- 22 个寄存器（完整数据）
- CSV 输出（便于调试）
- 标准轮询
```

**预期结果：**
- 2个IMU: **250-400Hz**
- 4个IMU: **125-200Hz**
- 6个IMU: **80-130Hz**

### 稳定优先配置（150-200Hz）
```cpp
// 在优化基础上增加容错
static const uint16_t RESPONSE_TIMEOUT_MS = 5;      // 3ms → 5ms
static const uint32_t BUS_SILENCE_US = 80;          // 40us → 80us
static const uint32_t FAIL_COOLDOWN_MS = 5;        // 2ms → 5ms
```

**优势：**
- 更高的可靠性
- 适应较差的总线条件
- 仍然能达到 200Hz 目标

---

## 性能对比表

| 配置 | 波特率 | 寄存器数 | 预期频率 (2 IMU) | CPU占用 | 实现难度 |
|------|--------|----------|------------------|---------|----------|
| 原始 | 921600 | 22 | 50-100Hz | 低 | ⭐ |
| 优化 | 921600 | 22 | 250-400Hz | 低 | ⭐ |
| 方案1 | 921600 | 11 | 400-600Hz | 低 | ⭐⭐ |
| 方案2 | 2000000 | 22 | 500-700Hz | 低 | ⭐⭐ |
| 方案4 | 921600 | 22 | 300-500Hz | 低 | ⭐⭐⭐ |
| 方案5 | 921600 | 22 | 300-500Hz | 中 | ⭐⭐⭐⭐ |
| 方案6 | 921600 | 22 | 400-600Hz | 低 | ⭐⭐⭐⭐ |
| 极致 | 2000000 | 11 | 1000-1500Hz | 中 | ⭐⭐⭐⭐⭐ |

---

## 选择建议

### 你应该使用优化版本（默认），如果：
- ✅ 需要达到 200Hz
- ✅ 希望实现简单
- ✅ 需要完整的 IMU 数据
- ✅ 硬件条件标准

### 你应该使用方案1（减少寄存器），如果：
- ✅ 只需要加速度和四元数
- ✅ 需要 400Hz+ 频率
- ✅ 有更多 IMU 需要查询

### 你应该使用方案2（2Mbps），如果：
- ✅ 硬件明确支持 2Mbps
- ✅ 总线质量良好（短距离、低噪声）
- ✅ 需要最高性能

### 你应该使用方案5（RTOS），如果：
- ✅ 系统还有其他复杂任务
- ✅ 需要最稳定的查询周期
- ✅ 可以接受额外的复杂度

---

## 测试建议

1. **基准测试**: 先使用优化版本，确认能达到 200Hz
2. **逐步提升**: 如果需要更高频率，逐个尝试进阶方案
3. **监控失败率**: 保持 `fail_count` 在总数的 1% 以内
4. **压力测试**: 长时间运行（1小时+）确保稳定性
5. **实际场景**: 在目标应用环境中测试（振动、温度等）

---

## 调试工具

### 监控实时周期时间
```cpp
static uint32_t last_cycle_us = 0;

void advanceImuIndex() {
  current_imu_index++;
  if (current_imu_index >= NUM_IMUS) {
    current_imu_index = 0;
    
    uint32_t now_us = micros();
    if (last_cycle_us > 0) {
      uint32_t cycle_time_us = now_us - last_cycle_us;
      Serial.print("# Cycle time: ");
      Serial.print(cycle_time_us);
      Serial.println(" us");
    }
    last_cycle_us = now_us;
    
    outputCycleCsv();
  }
}
```

### 示波器检查
- 监测 DE/RE 引脚切换时序
- 检查 TX/RX 数据波形
- 验证总线静默时间
- 确认波特率准确性

---

## 总结

对于 200Hz 目标，**优化版本已经足够**（可达 250-400Hz）。如果需要：
- **更多 IMU**: 使用方案1（减少寄存器）或方案2（提升波特率）
- **更高频率**: 组合多个方案
- **最佳稳定性**: 使用方案5（RTOS任务分离）

选择合适的优化方案取决于你的具体需求和硬件条件。建议从优化版本开始，根据实际测试结果决定是否需要进阶优化。
