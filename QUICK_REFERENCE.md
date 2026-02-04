# IMU 200Hz优化 - 快速参考

## 参数对比表

| 参数 | 原值 | 优化值 | 改善 |
|------|------|--------|------|
| `RESPONSE_TIMEOUT_MS` | 30ms | 3ms | **10倍加速** |
| `BUS_SILENCE_US` | 500μs | 100μs | **5倍加速** |
| `TX_ENABLE_DELAY_US` | 30μs | 10μs | **3倍加速** |
| `TX_DISABLE_DELAY_US` | 60μs | 20μs | **3倍加速** |
| `FAIL_COOLDOWN_MS` | 20ms | 5ms | **4倍加速** |
| `MAX_BACKOFF_SHIFT` | 4 | 3 | 降低最大退避 |

## 性能对比

| 指标 | 原版 | 优化版 |
|------|------|--------|
| 理论最大频率 | ~30Hz | **200Hz+** |
| 单IMU通信时间 | ~31ms | **~0.75ms** |
| 双IMU周期 | ~62ms | **~2-3ms** |
| 超时惩罚 | 30ms | **3ms** |

## 文件说明

- `imu_200hz_optimized.ino` - 优化后的Arduino代码
- `OPTIMIZATION_NOTES.md` - 详细优化说明和性能分析
- `QUICK_REFERENCE.md` - 本文件，快速参考

## 使用方法

1. 将`imu_200hz_optimized.ino`上传到ESP32
2. 监控串口输出验证200Hz采样率
3. 如有问题，参考`OPTIMIZATION_NOTES.md`中的调试建议

## 关键优化点

✅ **减少超时等待** - 从30ms降到3ms，失败检测速度提升10倍  
✅ **减少总线延迟** - 总线静默和TX切换延迟大幅降低  
✅ **快速失败恢复** - 失败重试间隔从20ms降到5ms  
✅ **保持代码结构** - 仅修改时序参数，不改变核心逻辑  

## 验证方法

```cpp
// 添加到loop()函数，监控实际采样率
static uint32_t sample_count = 0;
static uint32_t last_print = 0;

sample_count++;
if (millis() - last_print >= 1000) {
  Serial.print("Sampling rate: ");
  Serial.print(sample_count);
  Serial.println(" Hz");
  sample_count = 0;
  last_print = millis();
}
```

预期输出：`Sampling rate: 200-250 Hz`（双IMU总采样率）
