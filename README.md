# ESP32 IMU RS485 Modbus RTU 读取器 - 200Hz优化版

## 文件说明

| 文件 | 说明 |
|------|------|
| `imu_reader_optimized.ino` | 单核优化版本，适用于大多数场景 |
| `imu_reader_dual_core.ino` | ESP32双核版本，通信和输出分离，性能最佳 |
| `OPTIMIZATION_GUIDE.md` | 详细的优化策略说明 |

## 快速开始

### 1. 基础优化版 (推荐先尝试)

```cpp
// 使用 imu_reader_optimized.ino
// 主要优化:
// - 减少延迟参数 (超时3ms, 静默100μs)
// - CRC查表法加速
// - 预计算请求帧缓存
// - 批量串口输出
```

### 2. 双核版本 (最高性能)

```cpp
// 使用 imu_reader_dual_core.ino
// Core 0: RS485通信 (高优先级)
// Core 1: 串口输出 (低优先级)
// 通信不会被输出阻塞
```

## 核心优化参数

```cpp
// 优化前 -> 优化后
RESPONSE_TIMEOUT_MS:  30ms  -> 3ms
BUS_SILENCE_US:       500μs -> 100μs
TX_ENABLE_DELAY_US:   30μs  -> 10μs
TX_DISABLE_DELAY_US:  60μs  -> 20μs
FAIL_COOLDOWN_MS:     20ms  -> 5ms
```

## 预期性能

- 单核优化版: 稳定 **200Hz+**
- 双核版本: 稳定 **300Hz+** (取决于IMU响应速度)

## 注意事项

1. 确保IMU本身支持200Hz数据更新率
2. 如果通信不稳定，可适当增加 `RESPONSE_TIMEOUT_MS`
3. 使用二进制输出模式 (`OUTPUT_MODE_BINARY`) 可进一步提高性能
4. 查看 `OPTIMIZATION_GUIDE.md` 获取详细调优建议
