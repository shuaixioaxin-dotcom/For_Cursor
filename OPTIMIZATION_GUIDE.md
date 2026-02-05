# IMU Modbus RTU 数据稳定性优化指南

## 问题分析

根据描述，IMU1偶发收不到应答帧，这通常由以下原因导致：

1. **RS485收发切换时序问题** - TX→RX切换时间不足导致应答帧开头丢失
2. **总线静默时间不足** - 连续轮询时设备未完全准备好
3. **串口缓冲区溢出** - 高波特率下数据未及时读取
4. **缺少重试机制** - 一次失败就等待下一个轮询周期

## 优化点说明

### 优化点1: 增加总线时序参数

```cpp
// 原值
static const uint16_t RESPONSE_TIMEOUT_MS = 100;
static const uint32_t BUS_SILENCE_US = 500;
static const uint32_t TX_ENABLE_DELAY_US = 30;
static const uint32_t TX_DISABLE_DELAY_US = 60;

// 优化后
static const uint16_t RESPONSE_TIMEOUT_MS = 150;      // 增加超时容忍度
static const uint32_t BUS_SILENCE_US = 1000;          // 约1个字符时间@921600
static const uint32_t TX_ENABLE_DELAY_US = 50;        // 确保DE信号稳定
static const uint32_t TX_DISABLE_DELAY_US = 100;      // 确保切换完成
static const uint32_t POST_TX_SETTLE_US = 200;        // 新增: 发送完成后等待
static const uint32_t INTER_FRAME_DELAY_MS = 2;       // 新增: 帧间延时
```

**关键点**: 在921600波特率下，一个字节约11μs。增加这些延时可以确保RS485芯片完成收发切换。

### 优化点2: 重试机制

```cpp
static const uint8_t MAX_RETRY_COUNT = 2;   // 单次轮询最大重试2次
static const uint32_t RETRY_DELAY_MS = 5;   // 重试前等待5ms
```

当超时或解析失败时，立即重试而不是等待下一个轮询周期，显著提高数据获取成功率。

### 优化点3: 数据保持

```cpp
// 保存上一次有效数据
float last_valid_acc[3];
float last_valid_quat[4];
uint32_t last_valid_time_ms;
```

当通信失败时，使用上一次有效数据而不是清零，保持数据流连续性。

### 优化点4: 彻底清空接收缓冲区

```cpp
static void clearRxBuffer() {
  delayMicroseconds(100);
  uint32_t start = micros();
  while (Serial2.available() > 0 || (micros() - start) < 200) {
    if (Serial2.available() > 0) {
      Serial2.read();
      start = micros();
    }
  }
}
```

确保在发送新请求前，清除所有残余数据。

### 优化点5: 增加Serial2缓冲区

```cpp
Serial2.setRxBufferSize(512);  // 从默认256增加到512
```

高波特率下，增加缓冲区可防止数据溢出。

### 优化点6: 改进的发送函数

在发送完成后增加额外等待时间，确保数据完全发送出去后再切换到接收模式。

### 优化点7-11: 状态机增强

- 新增 `STATE_PRE_SEND_DELAY` 状态，发送前增加帧间延时
- 新增 `STATE_RETRY_DELAY` 状态，处理重试逻辑
- 改进接收数据的校验逻辑

## 参数调优建议

### 如果仍有问题，依次尝试：

#### 1. 进一步增加时序参数
```cpp
static const uint32_t TX_DISABLE_DELAY_US = 150;      // 从100增加到150
static const uint32_t POST_TX_SETTLE_US = 300;        // 从200增加到300
static const uint32_t INTER_FRAME_DELAY_MS = 5;       // 从2增加到5
```

#### 2. 降低波特率测试
```cpp
static const uint32_t RS485_BAUD = 460800;  // 从921600降到460800
```
如果降低波特率后稳定，说明是时序问题。

#### 3. 增加重试次数
```cpp
static const uint8_t MAX_RETRY_COUNT = 3;  // 从2增加到3
```

#### 4. 调整响应超时
```cpp
static const uint16_t RESPONSE_TIMEOUT_MS = 200;  // 从150增加到200
```

### 硬件检查建议

1. **检查RS485模块**
   - 确认DE/RE引脚连接正确
   - 检查A/B线是否接反
   - 确认终端电阻配置（长线需要120Ω终端电阻）

2. **检查电源**
   - RS485模块供电是否稳定
   - 是否有足够的去耦电容

3. **检查接线**
   - 双绞线质量
   - 接线长度和布线方式
   - 是否有电磁干扰源

## 实时性影响评估

| 优化项 | 延时增加 | 说明 |
|--------|----------|------|
| TX_ENABLE_DELAY | +20μs | 可忽略 |
| TX_DISABLE_DELAY | +40μs | 可忽略 |
| POST_TX_SETTLE | +200μs | 每次发送 |
| INTER_FRAME_DELAY | +2ms | 每帧间 |
| 单次重试 | ~160ms | 仅失败时 |

### 频率估算

假设2个IMU，无重试情况：
- 原版: ~500 Hz 理论最大
- 优化后: ~200-250 Hz (主要受 INTER_FRAME_DELAY 影响)

如需更高频率，可以：
1. 减少 `INTER_FRAME_DELAY_MS` 到 1ms
2. 减少 `RESPONSE_TIMEOUT_MS` 到 100ms
3. 但这会降低稳定性

## 调试方法

### 开启调试输出
```cpp
static const bool DEBUG_RETRY = true;      // 查看重试情况
static const bool DEBUG_RAW_RESPONSES = true;  // 查看原始数据
```

### 观察统计信息
每秒输出的统计信息中：
- `Retry Success` - 如果这个值很高，说明重试机制在起作用
- `Timeout` - 如果超时很多，考虑增加超时时间或检查硬件
- `CRC Error` - 如果CRC错误多，可能是电气干扰或波特率问题

## 示波器调试建议

在示波器上同时观察：
1. RS485 TX线（DI引脚）
2. RS485 RX线（RO引脚）
3. DE/RE控制信号

检查：
- 请求发送后，DE/RE是否及时切换到接收模式
- IMU应答帧的起始位置是否在DE/RE切换后
- 是否有应答帧被截断的情况

如果发现应答帧在DE/RE切换前就开始，需要增加 `POST_TX_SETTLE_US`。
