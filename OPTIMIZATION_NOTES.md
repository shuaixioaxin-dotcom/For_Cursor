# IMU 200Hz 轮询优化说明

## 优化目标
将双IMU系统的轮询频率从原来的约30Hz提升到200Hz

## 关键优化项

### 1. 响应超时优化
- **原值**: `RESPONSE_TIMEOUT_MS = 30ms`
- **新值**: `RESPONSE_TIMEOUT_MS = 3ms`
- **理由**: 在921600波特率下，49字节响应约需0.5ms传输完成，3ms超时已足够且能快速检测失败

### 2. 总线静默时间优化
- **原值**: `BUS_SILENCE_US = 500μs`
- **新值**: `BUS_SILENCE_US = 100μs`
- **理由**: Modbus RTU标准要求3.5个字符时间（约38μs @921600），100μs已提供足够裕量

### 3. TX延迟优化
- **原值**: `TX_ENABLE_DELAY_US = 30μs`, `TX_DISABLE_DELAY_US = 60μs`
- **新值**: `TX_ENABLE_DELAY_US = 10μs`, `TX_DISABLE_DELAY_US = 20μs`
- **理由**: 现代RS485收发器切换时间通常<5μs，减少不必要的延迟

### 4. 失败冷却优化
- **原值**: `FAIL_COOLDOWN_MS = 20ms`, `MAX_BACKOFF_SHIFT = 4`
- **新值**: `FAIL_COOLDOWN_MS = 5ms`, `MAX_BACKOFF_SHIFT = 3`
- **理由**: 更快重试失败的设备，退避时间：5ms → 10ms → 20ms → 40ms

## 性能分析

### 理论计算
- **目标周期**: 1000ms / 200Hz = 5ms
- **双IMU系统**: 每个IMU需在2.5ms内完成

### 单IMU通信时间
1. **请求**: 8字节 × 10位/字节 ÷ 921600 = 0.087ms
2. **响应**: 49字节 × 10位/字节 ÷ 921600 = 0.532ms
3. **延迟总计**: TX_EN(0.01ms) + TX_DIS(0.02ms) + BUS_SILENCE(0.1ms) = 0.13ms
4. **总计**: 0.087 + 0.532 + 0.13 ≈ **0.75ms**

### 实际周期估算
- 理想情况：0.75ms × 2 = **1.5ms** (可达 666Hz)
- 加上处理开销和输出：约 **2-3ms** (可达 200-300Hz)
- 目标200Hz完全可行，有充足余量

## 注意事项

### 硬件要求
1. **RS485收发器**: 需支持快速切换（如MAX485、MAX13487等）
2. **IMU响应速度**: 确保IMU能在1ms内准备好响应
3. **电缆长度**: 高波特率下建议≤50m，使用优质屏蔽双绞线

### 调试建议
1. 如果出现大量超时，可以适当增加`RESPONSE_TIMEOUT_MS`到5ms
2. 如果出现CRC错误，检查：
   - 电缆质量和长度
   - 终端电阻是否正确配置（120Ω）
   - TX延迟是否足够（根据具体收发器调整）
3. 监控失败率：`imu_fail_streak`应该保持在0

### 性能监控
可以添加性能统计代码：

```cpp
// 在全局变量中添加
static uint32_t loop_count = 0;
static uint32_t last_stats_ms = 0;

// 在loop()中添加
loop_count++;
if (millis() - last_stats_ms >= 1000) {
  Serial.print("Rate: ");
  Serial.print(loop_count);
  Serial.println(" Hz");
  loop_count = 0;
  last_stats_ms = millis();
}
```

## 进一步优化方向

如需更高频率，可考虑：

1. **DMA传输**: 使用ESP32的DMA功能减少CPU开销
2. **中断驱动**: 用UART中断替代轮询Serial2.available()
3. **双核并行**: 利用ESP32双核，一个核心专门处理通信
4. **批量读取**: 一次请求读取多个寄存器减少往返次数
5. **固定时序**: 使用硬件定时器确保精确的轮询周期
