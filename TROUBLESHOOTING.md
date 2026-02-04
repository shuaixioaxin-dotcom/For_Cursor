# 故障排查指南

## 使用监控版本进行诊断

首先上传 `imu_200hz_with_monitoring.ino`，观察每秒输出的性能报告。

## 常见问题及解决方案

### 问题1: 采样率低于200Hz

#### 症状
```
Sample Rate: 45.2 Hz  ← 远低于目标
```

#### 可能原因及解决方案

**原因A: 超时次数过多**
```
Timeouts: 156  ← 大量超时
```
✅ **解决方案：** 增加超时时间
```cpp
static const uint16_t RESPONSE_TIMEOUT_MS = 5;  // 从3ms改为5ms
```

**原因B: IMU失败重试**
```
IMU1 - Fail Streak: 3, Valid: NO  ← 连续失败
```
✅ **解决方案：** 检查IMU硬件连接和ID配置
- 确认IMU ID正确（默认1和2）
- 检查RS485接线：A-A，B-B
- 测试单个IMU是否正常

**原因C: 串口输出过慢**
- 如果使用了其他串口输出或日志
- 减少输出频率或使用更高波特率

---

### 问题2: 成功率低（<90%）

#### 症状
```
Success Rate: 67.8 %  ← 成功率低
CRC Errors: 89       ← CRC校验失败
```

#### 解决方案

**步骤1: 检查硬件连接**
- [ ] RS485 A线和B线是否正确连接
- [ ] 线缆是否为屏蔽双绞线
- [ ] 接地是否良好
- [ ] 连接器是否牢固

**步骤2: 检查终端电阻**
- [ ] 总线两端是否有120Ω终端电阻
- [ ] 电阻是否并联在A和B之间
- [ ] 电阻阻值是否准确

**步骤3: 检查电缆长度**
| 波特率 | 最大推荐距离 |
|--------|-------------|
| 921600 | 50m |
| 500000 | 100m |
| 115200 | 1000m |

如果电缆过长，考虑：
- 降低波特率
- 使用中继器
- 缩短距离

**步骤4: 检查RS485收发器**
- 使用高速收发器（如MAX13487E、MAX3485E）
- 避免使用低速型号（如旧版MAX485）
- 检查收发器供电是否稳定（3.3V或5V）

**步骤5: 优化TX延迟**
```cpp
// 如果使用慢速收发器，增加切换延迟
static const uint32_t TX_ENABLE_DELAY_US  = 20;  // 从10us增加到20us
static const uint32_t TX_DISABLE_DELAY_US = 40;  // 从20us增加到40us
```

---

### 问题3: Loop时间过长

#### 症状
```
Loop Time (min/max/last): 2450 / 8900 / 5200 μs  ← max太大
```

#### 原因分析

**正常范围：**
- 最小: 100-500 μs（无通信时的空循环）
- 最大: 1000-2000 μs（通信+处理）
- 平均: 500-1500 μs

**异常情况：**
- 如果max >5000 μs，说明有阻塞

#### 解决方案

**检查点1: Serial.print性能**
```cpp
// 如果输出过于频繁，考虑降频
static uint32_t last_output_ms = 0;
if (millis() - last_output_ms >= 5) {  // 每5ms输出一次，200Hz
  outputImuData();
  last_output_ms = millis();
}
```

**检查点2: 其他阻塞操作**
- 移除所有`delay()`调用
- 移除长时间的计算
- 避免在loop中使用阻塞式I/O

---

### 问题4: 某个IMU持续失败

#### 症状
```
IMU1 - Fail Streak: 0, Valid: YES  ← 正常
IMU2 - Fail Streak: 3, Valid: NO   ← 异常
```

#### 排查步骤

**步骤1: 单独测试IMU2**
```cpp
// 临时修改，只轮询IMU2
#define NUM_IMUS 1
const uint8_t IMU_IDS[NUM_IMUS] = {2};  // 只测试ID=2的IMU
```

**步骤2: 检查IMU配置**
- 确认IMU的Modbus地址是否为2
- 确认IMU波特率设置为921600
- 确认IMU数据格式正确

**步骤3: 检查物理连接**
- 交换IMU1和IMU2的位置测试
- 如果问题随IMU移动 → IMU硬件问题
- 如果问题不随IMU移动 → 总线问题

**步骤4: 使用Modbus调试工具**
- 使用Modbus Master软件测试IMU2
- 确认可以正确读取寄存器0x0034开始的22个寄存器
- 检查响应时间是否正常

---

### 问题5: 随机出现CRC错误

#### 症状
```
CRC Errors: 3-5  ← 偶尔出现
Success Rate: 95-98%
```

#### 原因及解决

**原因: 电磁干扰（EMI）**

✅ **解决方案：**
1. 使用屏蔽电缆并正确接地
2. 远离电机、继电器等干扰源
3. RS485线缆与电源线分开布线
4. 增加滤波电容（在收发器VCC和GND之间）

**原因: 时序问题**

✅ **解决方案：**
```cpp
// 增加总线静默时间
static const uint32_t BUS_SILENCE_US = 200;  // 从100us增加到200us
```

---

### 问题6: 启动时无响应

#### 症状
- 上传代码后无任何输出
- 串口监视器无数据

#### 排查步骤

**步骤1: 检查串口波特率**
```
确保串口监视器设置为: 2000000
```

**步骤2: 检查RS485_DE_RE引脚**
```cpp
// 确认GPIO25没有被其他模块占用
#define RS485_DE_RE_PIN 25
```
如果冲突，修改为其他可用GPIO

**步骤3: 检查Serial2引脚**
```cpp
// ESP32默认Serial2引脚
RX: GPIO32
TX: GPIO33
```
确认这些引脚没有硬件冲突

**步骤4: 添加启动信息**
```cpp
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);  // 等待串口初始化
  Serial.println("System starting...");
  // ... 其他初始化代码
}
```

---

## 性能优化检查清单

### 硬件检查清单
- [ ] RS485收发器支持高速切换（<5μs）
- [ ] 使用CAT5e或更好的屏蔽双绞线
- [ ] 电缆长度 <50m（@921600波特率）
- [ ] 两端正确配置120Ω终端电阻
- [ ] 电源稳定，纹波 <100mV
- [ ] 良好的接地

### 软件检查清单
- [ ] IMU地址配置正确
- [ ] IMU波特率设置为921600
- [ ] Modbus寄存器地址正确（0x0034）
- [ ] 寄存器数量正确（22个，0x0016）
- [ ] ESP32串口引脚配置正确

### 性能指标检查清单
- [ ] Sample Rate >200 Hz
- [ ] Success Rate >98%
- [ ] Timeouts <5 per second
- [ ] CRC Errors <3 per second
- [ ] Loop Time (max) <3000 μs

---

## 调试工具

### 1. 查看原始数据

在 `tryExtractFrame()` 中添加：
```cpp
// 打印接收到的原始数据
Serial.print("RX: ");
for (size_t i = 0; i < rxCount(); i++) {
  size_t idx = (rx_tail + i) % RX_RING_SZ;
  Serial.print(rx_ring[idx], HEX);
  Serial.print(" ");
}
Serial.println();
```

### 2. 监控通信时间

```cpp
uint32_t tx_start = micros();
sendRequestCached(current_imu_index);
uint32_t tx_end = micros();

Serial.print("TX time: ");
Serial.print(tx_end - tx_start);
Serial.println(" us");
```

### 3. 波形分析

使用逻辑分析仪或示波器监测：
- RS485 A/B信号完整性
- DE/RE切换时序
- 信号上升/下降时间

---

## 获取帮助

如果以上方案都无法解决问题，请提供以下信息：

1. **监控版本的完整输出**（至少5秒的数据）
2. **硬件配置**
   - ESP32型号
   - RS485收发器型号
   - IMU型号
   - 电缆类型和长度
3. **修改过的参数**（如果有）
4. **问题描述和出现频率**

---

## 性能基准参考

### 理想条件下的预期性能
```
========== Performance Statistics ==========
Sample Rate: 200-250 Hz
Request Rate: 400-500 Hz
Success Rate: 99.5-100.0 %
Total Requests: 450
Successful: 449
Timeouts: 0-2
CRC Errors: 0-1
Loop Time (min/max/last): 180 / 1850 / 720 μs
IMU1 - Fail Streak: 0, Valid: YES
IMU2 - Fail Streak: 0, Valid: YES
============================================
```

### 可接受的性能（需要优化）
```
Sample Rate: 180-200 Hz
Success Rate: 95-99 %
Timeouts: <10
CRC Errors: <5
Loop Time (max): <3000 μs
```

### 不可接受的性能（需要排查）
```
Sample Rate: <180 Hz
Success Rate: <95 %
Timeouts: >20
CRC Errors: >10
Loop Time (max): >5000 μs
```
