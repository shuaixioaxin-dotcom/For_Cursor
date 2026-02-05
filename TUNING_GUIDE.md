# 参数调优指南 - 根据硬件优化到最佳性能

## 🎯 调优目标

根据你的实际硬件条件，找到**性能**和**稳定性**之间的最佳平衡点。

## 📊 参数影响矩阵

| 参数 | 影响性能 | 影响稳定性 | 调优难度 |
|------|----------|------------|----------|
| **CSV输出频率** | ⭐⭐⭐⭐⭐ | ✅ 无影响 | ✅ 简单 |
| **响应超时** | ⭐⭐⭐⭐ | ⚠️ 高影响 | ⚠️ 需测试 |
| **总线静默时间** | ⭐⭐⭐ | ⚠️ 高影响 | ⚠️ 需测试 |
| **波特率** | ⭐⭐⭐⭐⭐ | ⚠️ 极高影响 | ⚠️ 硬件限制 |
| **寄存器数量** | ⭐⭐⭐⭐ | ✅ 无影响 | ⚠️ 需理解协议 |
| **TX延迟** | ⭐⭐ | ⚠️ 中等影响 | ⚠️ 硬件相关 |

## 🔧 逐步调优流程

### 第一步：从基准开始

使用**保守版**的默认参数：

```cpp
static const uint32_t RS485_BAUD = 921600;
static const uint16_t RESPONSE_TIMEOUT_MS = 10;
static const uint32_t BUS_SILENCE_US = 200;
static const uint8_t CSV_OUTPUT_DIVIDER = 5;
```

**运行并记录**：
- 频率：`_____ Hz`
- 成功率：`ok=_____, fail=_____`
- 失败率：`_____ %`

### 第二步：优化输出频率（最安全）

**目标**：失败率保持在0%，提升频率

```cpp
// 尝试1：改成10
static const uint8_t CSV_OUTPUT_DIVIDER = 10;
记录频率：_____ Hz，失败率：_____ %

// 尝试2：改成20
static const uint8_t CSV_OUTPUT_DIVIDER = 20;
记录频率：_____ Hz，失败率：_____ %

// 尝试3：改成50（实际应用可能不需要这么高）
static const uint8_t CSV_OUTPUT_DIVIDER = 50;
记录频率：_____ Hz，失败率：_____ %
```

**建议**：
- 失败率=0%：继续增加divider
- 失败率>5%：保持当前值
- 实际应用：根据上位机需要的更新率选择

### 第三步：优化超时时间（需谨慎）

**原理**：超时只影响失败情况，成功时无影响

```cpp
// 当前值：10ms
// 测试当前失败率

如果失败率 < 1%：
  // 尝试降低到8ms
  static const uint16_t RESPONSE_TIMEOUT_MS = 8;
  
  如果失败率仍 < 1%：
    // 尝试降低到5ms
    static const uint16_t RESPONSE_TIMEOUT_MS = 5;

如果失败率 > 5%：
  // 增加到15ms
  static const uint16_t RESPONSE_TIMEOUT_MS = 15;
```

**建议的超时值**：

| 通信质量 | 推荐超时 | 说明 |
|----------|----------|------|
| 优秀（fail<0.1%） | 5ms | 追求极致性能 |
| 良好（fail<1%） | 10ms | 平衡性能和稳定性 |
| 一般（fail<5%） | 15-20ms | 优先保证稳定性 |
| 较差（fail>5%） | 30ms+ | 检查硬件连接 |

### 第四步：优化总线延迟（高级）

**警告**：这个参数需要根据实际硬件测试

```cpp
// 保守版默认：200μs
static const uint32_t BUS_SILENCE_US = 200;

// 测试步骤：
1. 运行24小时，记录失败率
2. 如果 fail=0，尝试降低到150μs
3. 再运行24小时，记录失败率
4. 如果 fail仍=0，尝试100μs
5. 如果出现失败，回退到上一个稳定值
```

**推荐值**：

| 线缆长度 | 推荐值 | 说明 |
|----------|--------|------|
| < 1m | 100μs | 短线高速 |
| 1-3m | 200μs | 平衡值（保守版默认）|
| 3-10m | 300-500μs | 长线路需要更多延迟 |
| > 10m | 500μs+ | 原始值 |

### 第五步：提升波特率（需硬件支持）

**前提条件**：
- ✅ RS485模块规格支持更高波特率
- ✅ IMU设备支持并已配置
- ✅ 线缆质量好，长度短（<2m推荐）

**测试步骤**：

```cpp
// 步骤1：尝试1.5Mbps（如果支持）
static const uint32_t RS485_BAUD = 1500000;
// 运行测试：记录失败率

// 步骤2：尝试2Mbps（激进值）
static const uint32_t RS485_BAUD = 2000000;
// 运行测试：记录失败率
```

**判断标准**：

| 失败率 | 结论 | 操作 |
|--------|------|------|
| = 0% | ✅ 完美 | 可以使用 |
| < 1% | ⚠️ 可接受 | 看应用容忍度 |
| 1-5% | ⚠️ 不稳定 | 不推荐 |
| > 5% | ❌ 失败 | 降低波特率 |

**常见波特率选项**：

```cpp
921600   // 标准值，兼容性最好
1000000  // 1Mbps，部分设备支持
1500000  // 1.5Mbps
2000000  // 2Mbps，需要优质硬件
```

### 第六步：减少寄存器读取（需理解协议）

**前提**：了解IMU寄存器映射

#### 方案A：仅读加速度+四元数

```cpp
// 原始：0x0034开始，22个寄存器
static const uint16_t MODBUS_REG_START = 0x0034;
static const uint16_t MODBUS_REG_COUNT = 0x0016;  // 22

// 优化：只读11个寄存器（加速度3+温度1+角度3+四元数4）
static const uint16_t MODBUS_REG_COUNT = 0x000B;  // 11

// 更新响应长度
static const uint16_t RESPONSE_BYTE_COUNT = MODBUS_REG_COUNT * 2;
static const uint16_t RESPONSE_LEN = 1 + 1 + 1 + RESPONSE_BYTE_COUNT + 2;
```

#### 方案B：仅读加速度

```cpp
// 如果只需要加速度
static const uint16_t MODBUS_REG_COUNT = 0x0003;  // 3个寄存器

// 相应修改parseResponse函数，跳过四元数解析
```

**时间节省**：

| 寄存器数量 | 响应字节 | 传输时间@921600 | 传输时间@2Mbps |
|------------|----------|-----------------|----------------|
| 22（原始） | 47字节 | ~510μs | ~235μs |
| 11（优化） | 27字节 | ~293μs | ~135μs |
| 3（最小） | 11字节 | ~120μs | ~55μs |

### 第七步：优化TX延迟（最后调整）

```cpp
// 保守版默认
static const uint32_t TX_ENABLE_DELAY_US = 20;
static const uint32_t TX_DISABLE_DELAY_US = 40;

// 激进值（需测试）
static const uint32_t TX_ENABLE_DELAY_US = 10;
static const uint32_t TX_DISABLE_DELAY_US = 20;

// 如果出现通信错误，增加延迟
static const uint32_t TX_ENABLE_DELAY_US = 30;
static const uint32_t TX_DISABLE_DELAY_US = 60;
```

**说明**：
- `TX_ENABLE_DELAY_US`：从接收切换到发送的延迟
- `TX_DISABLE_DELAY_US`：发送完成后等待，确保最后字节传输完成

## 📊 性能计算器

### 估算你的目标频率

```
单个IMU查询时间 = 请求时间 + 响应时间 + 总线静默 + TX延迟

请求时间 = (8字节 × 10 bits) / 波特率
响应时间 = (响应字节数 × 10 bits) / 波特率

例如：波特率=921600, 响应47字节
请求时间 = 80 / 921600 = 86.8μs
响应时间 = 470 / 921600 = 510μs
总线静默 = 200μs
TX延迟 = 60μs
单次查询 = 857μs ≈ 0.86ms

2个IMU完整周期 = 0.86ms × 2 = 1.72ms
无输出时频率 = 1000ms / 1.72ms ≈ 581Hz

考虑CSV输出（每N个周期）：
实际周期 = 1.72ms + (2ms / N)
N=5时：实际周期 = 2.12ms，频率 = 472Hz
N=10时：实际周期 = 1.92ms，频率 = 521Hz
```

### 在线计算工具

填入你的参数：

```
波特率（baud）: _______
寄存器数量: _______
IMU数量: _______
总线静默时间（μs）: _______
CSV输出除数: _______

计算结果：
理论频率 = _______ Hz
```

## 🎯 推荐配置方案

### 方案1：稳定型（推荐新手）

```cpp
// 目标：200Hz稳定运行，失败率<0.1%
static const uint32_t RS485_BAUD = 921600;
static const uint16_t MODBUS_REG_COUNT = 0x0016;  // 22寄存器
static const uint16_t RESPONSE_TIMEOUT_MS = 15;
static const uint32_t BUS_SILENCE_US = 300;
static const uint32_t TX_ENABLE_DELAY_US = 20;
static const uint32_t TX_DISABLE_DELAY_US = 40;
static const uint8_t CSV_OUTPUT_DIVIDER = 10;
```

**预期性能**：220-250Hz，fail≈0

### 方案2：平衡型（推荐大多数用户）

```cpp
// 目标：300Hz，失败率<1%
static const uint32_t RS485_BAUD = 921600;
static const uint16_t MODBUS_REG_COUNT = 0x0016;  // 22寄存器
static const uint16_t RESPONSE_TIMEOUT_MS = 10;
static const uint32_t BUS_SILENCE_US = 200;
static const uint32_t TX_ENABLE_DELAY_US = 20;
static const uint32_t TX_DISABLE_DELAY_US = 40;
static const uint8_t CSV_OUTPUT_DIVIDER = 20;
```

**预期性能**：280-320Hz，fail<1%

### 方案3：高性能型（需要好硬件）

```cpp
// 目标：500Hz
static const uint32_t RS485_BAUD = 2000000;
static const uint16_t MODBUS_REG_COUNT = 0x000B;  // 11寄存器
static const uint16_t RESPONSE_TIMEOUT_MS = 8;
static const uint32_t BUS_SILENCE_US = 150;
static const uint32_t TX_ENABLE_DELAY_US = 15;
static const uint32_t TX_DISABLE_DELAY_US = 30;
static const uint8_t CSV_OUTPUT_DIVIDER = 50;
```

**预期性能**：450-550Hz，需要优质硬件

### 方案4：极限型（实验性）

```cpp
// 目标：700Hz+，可能不稳定
static const uint32_t RS485_BAUD = 2000000;
static const uint16_t MODBUS_REG_COUNT = 0x0003;  // 3寄存器（仅加速度）
static const uint16_t RESPONSE_TIMEOUT_MS = 5;
static const uint32_t BUS_SILENCE_US = 100;
static const uint32_t TX_ENABLE_DELAY_US = 10;
static const uint32_t TX_DISABLE_DELAY_US = 20;
static const uint8_t CSV_OUTPUT_DIVIDER = 100;
```

**预期性能**：700-900Hz，仅用于测试极限

## 🔍 调试技巧

### 1. 添加详细日志

```cpp
// 在setup()后添加
Serial.println("# === Configuration ===");
Serial.print("# RS485_BAUD: "); Serial.println(RS485_BAUD);
Serial.print("# MODBUS_REG_COUNT: "); Serial.println(MODBUS_REG_COUNT);
Serial.print("# RESPONSE_TIMEOUT_MS: "); Serial.println(RESPONSE_TIMEOUT_MS);
Serial.print("# BUS_SILENCE_US: "); Serial.println(BUS_SILENCE_US);
Serial.print("# CSV_OUTPUT_DIVIDER: "); Serial.println(CSV_OUTPUT_DIVIDER);
Serial.println("# ====================");
```

### 2. 监控失败模式

```cpp
// 在recordFailure()函数中添加
static uint32_t last_fail_print = 0;
if (millis() - last_fail_print > 1000) {
  Serial.print("# IMU ");
  Serial.print(IMU_IDS[idx]);
  Serial.print(" fail, streak=");
  Serial.println(imu_fail_streak[idx]);
  last_fail_print = millis();
}
```

### 3. 测量实际通信时间

```cpp
// 在sendRequest()前后添加
uint32_t start_us = micros();
sendRequest(IMU_IDS[current_imu_index]);
uint32_t send_duration = micros() - start_us;

// 在接收完成后
uint32_t total_duration = micros() - start_us;
Serial.print("# Query time: ");
Serial.print(total_duration);
Serial.println(" us");
```

## 📝 调优记录表

建议记录每次调优的结果：

```
日期：2026-02-05
波特率：921600
超时：10ms
总线静默：200μs
CSV除数：10
运行时间：1小时
平均频率：287 Hz
成功次数：1,032,120
失败次数：156
失败率：0.015%
结论：✅ 可接受，尝试增加CSV除数到20
---
日期：2026-02-05
波特率：921600
超时：10ms
总线静默：200μs  
CSV除数：20
运行时间：1小时
平均频率：312 Hz
成功次数：1,123,200
失败次数：142
失败率：0.013%
结论：✅ 更好，采用此配置
---
```

## 🎓 高级优化

如果以上优化还不够，考虑：

1. **使用二进制输出替代CSV**
   - 减少90%的输出时间
   - 需要修改上位机解析程序

2. **使用RTOS任务分离**
   - 通信任务和输出任务独立
   - 需要ESP32 FreeRTOS知识

3. **使用DMA传输**
   - 完全异步，零CPU占用
   - 需要深入ESP32 UART驱动

4. **优化数据结构**
   - 使用定点数替代浮点数
   - 使用查表替代CRC计算

5. **减少IMU数量**
   - 单个IMU可达到2倍频率
   - 或考虑使用多个RS485总线

---

**祝调优顺利！记住：稳定性永远比极限性能更重要。**
