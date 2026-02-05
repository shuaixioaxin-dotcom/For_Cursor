# 快速调优参考表

## 预设配置方案

根据你的应用场景和电磁环境，选择合适的配置：

### 方案1：默认配置（推荐）⭐
**适用**：大多数应用，平衡稳定性与性能

```cpp
static const uint32_t BUS_SILENCE_US = 1500;
static const uint32_t TX_ENABLE_DELAY_US = 100;
static const uint32_t TX_DISABLE_DELAY_US = 150;
static const uint32_t PRE_REQUEST_DELAY_US = 200;
static const uint16_t RESPONSE_TIMEOUT_MS = 150;
static const uint8_t MAX_IMMEDIATE_RETRIES = 2;
static const uint32_t RETRY_DELAY_MS = 5;
```

**性能指标**：
- 轮询频率：~115 Hz
- 预期稳定性：⭐⭐⭐⭐⭐

---

### 方案2：高稳定性配置（恶劣环境）
**适用**：强电磁干扰环境、长线缆、多设备

```cpp
static const uint32_t BUS_SILENCE_US = 2500;          // 2.5ms
static const uint32_t TX_ENABLE_DELAY_US = 150;       // 150us
static const uint32_t TX_DISABLE_DELAY_US = 250;      // 250us
static const uint32_t PRE_REQUEST_DELAY_US = 500;     // 500us
static const uint16_t RESPONSE_TIMEOUT_MS = 200;      // 200ms
static const uint8_t MAX_IMMEDIATE_RETRIES = 3;       // 3次重试
static const uint32_t RETRY_DELAY_MS = 10;            // 10ms间隔
```

**性能指标**：
- 轮询频率：~80 Hz
- 预期稳定性：⭐⭐⭐⭐⭐⭐

**牺牲**：实时性降低约50%

---

### 方案3：平衡性能配置（较好环境）
**适用**：电磁环境良好、短线缆、设备稳定

```cpp
static const uint32_t BUS_SILENCE_US = 1000;          // 1ms
static const uint32_t TX_ENABLE_DELAY_US = 50;        // 50us
static const uint32_t TX_DISABLE_DELAY_US = 100;      // 100us
static const uint32_t PRE_REQUEST_DELAY_US = 100;     // 100us
static const uint16_t RESPONSE_TIMEOUT_MS = 120;      // 120ms
static const uint8_t MAX_IMMEDIATE_RETRIES = 1;       // 1次重试
static const uint32_t RETRY_DELAY_MS = 5;             // 5ms间隔
```

**性能指标**：
- 轮询频率：~140 Hz
- 预期稳定性：⭐⭐⭐⭐

**优势**：实时性较好

---

### 方案4：极限性能（仅测试用）
**适用**：理想环境、短距离、单独测试

```cpp
static const uint32_t BUS_SILENCE_US = 500;           // 0.5ms（Modbus最小值）
static const uint32_t TX_ENABLE_DELAY_US = 30;        // 30us
static const uint32_t TX_DISABLE_DELAY_US = 60;       // 60us
static const uint32_t PRE_REQUEST_DELAY_US = 50;      // 50us
static const uint16_t RESPONSE_TIMEOUT_MS = 100;      // 100ms
static const uint8_t MAX_IMMEDIATE_RETRIES = 1;       // 1次重试
static const uint32_t RETRY_DELAY_MS = 3;             // 3ms间隔
```

**性能指标**：
- 轮询频率：~160 Hz
- 预期稳定性：⭐⭐⭐

**警告**：生产环境不推荐，可能不稳定

---

## 参数含义速查

| 参数 | 含义 | 影响 | 典型范围 |
|------|------|------|----------|
| `BUS_SILENCE_US` | 总线静默时间 | 防止冲突 | 500-3000 us |
| `TX_ENABLE_DELAY_US` | 发送使能延迟 | 驱动器准备 | 30-200 us |
| `TX_DISABLE_DELAY_US` | 发送禁用延迟 | 数据完整性 | 60-300 us |
| `PRE_REQUEST_DELAY_US` | 请求前延迟 | 总线稳定 | 0-500 us |
| `RESPONSE_TIMEOUT_MS` | 响应超时 | 误判率 | 100-300 ms |
| `MAX_IMMEDIATE_RETRIES` | 最大重试次数 | 恢复能力 | 0-5 次 |
| `RETRY_DELAY_MS` | 重试间隔 | 恢复速度 | 3-20 ms |

## 故障排查流程

### 问题：仍然频繁超时

**诊断**：
1. 检查 `RESPONSE_TIMEOUT_MS` 是否太短
2. 检查IMU设备是否正常响应
3. 观察 `Retry Success` 是否>0

**解决**：
- 增加 `RESPONSE_TIMEOUT_MS` 到200ms
- 增加 `MAX_IMMEDIATE_RETRIES` 到3

---

### 问题：CRC错误频繁

**诊断**：
1. 检查 `BUS_SILENCE_US` 是否不足
2. 检查 `TX_DISABLE_DELAY_US` 是否太短
3. 可能是总线冲突或数据截断

**解决**：
- 增加 `BUS_SILENCE_US` 到2500us
- 增加 `TX_DISABLE_DELAY_US` 到250us
- 检查RS485终端电阻

---

### 问题：更新频率太低

**诊断**：
- 参数过于保守
- 重试次数过多

**解决**（仅在数据稳定后）：
- 使用"方案3：平衡性能配置"
- 减少 `MAX_IMMEDIATE_RETRIES` 到1
- 减小 `BUS_SILENCE_US` 到1000us

---

### 问题：偶发丢帧

**诊断**：
- 电磁干扰
- 总线时序不稳定

**解决**：
- 使用"方案2：高稳定性配置"
- 检查硬件（接地、屏蔽、线缆）
- 增加 `PRE_REQUEST_DELAY_US`

---

## 硬件配置建议

### RS485终端电阻
```
[ESP32] ----RS485---- [IMU1] ---- [IMU2]
   |                                  |
  120Ω                              120Ω
```
仅在总线两端安装120Ω电阻

### 接线规范
- 使用双绞线（如超五类网线）
- A/B线分别使用一对绞线
- GND单独走线或使用屏蔽
- 避免与220V强电平行布线

### 电源去耦
在每个IMU和ESP32的5V/3.3V电源引脚附近添加：
- 100nF陶瓷电容（贴片放置）
- 10uF电解电容（可选）

## 监控指标

### 健康状态判断

**优秀**：
```
Success: >99%, Retry Success: 1-5%, Timeout: <0.1%, CRC Error: 0%
Frequency: >100 Hz (2 IMUs)
```

**良好**：
```
Success: >95%, Retry Success: 5-10%, Timeout: <1%, CRC Error: <1%
Frequency: >80 Hz
```

**需改善**：
```
Success: <95%, Retry Success: >10%, Timeout: >1%, CRC Error: >1%
Frequency: <80 Hz
```

**异常**（需检查硬件）：
```
Success: <90%, Timeout: >5%, CRC Error: >3%
```

## 波特率选择

当前：921600 bps

### 降低波特率的优势
- 更长的传输距离
- 更好的抗干扰能力
- 更低的错误率

### 可选波特率（需同步修改IMU设置）

| 波特率 | 适用距离 | 抗干扰 | 速度 |
|--------|----------|--------|------|
| 921600 | <50m | ⭐⭐ | ⭐⭐⭐⭐⭐ |
| 460800 | <100m | ⭐⭐⭐ | ⭐⭐⭐⭐ |
| 230400 | <200m | ⭐⭐⭐⭐ | ⭐⭐⭐ |
| 115200 | <500m | ⭐⭐⭐⭐⭐ | ⭐⭐ |

**修改方法**：
```cpp
// 在代码中修改
static const uint32_t RS485_BAUD = 460800;  // 例如改为460800

// 同时需要配置IMU设备波特率
```

## 快速测试脚本

### 基准测试（10秒）
```cpp
void benchmark() {
  unsigned long start = millis();
  uint32_t success = 0, fail = 0;
  
  while (millis() - start < 10000) {
    loop();
    if (imu_data[0].valid) success++;
    else fail++;
  }
  
  Serial.printf("Benchmark: Success=%lu, Fail=%lu, Rate=%.1f%%\n", 
                success, fail, 100.0 * success / (success + fail));
}
```

在 `setup()` 末尾调用 `benchmark()` 进行快速测试。

## 总结

- **首次使用**：从"方案1：默认配置"开始
- **仍有问题**：升级到"方案2：高稳定性配置"
- **性能不足**：降级到"方案3：平衡性能配置"
- **持续监控**：观察统计输出，根据实际情况微调

记住：**稳定性优先于性能**，数据丢失比延迟更严重。
