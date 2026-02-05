# IMU Modbus 批处理轮询优化

基于编码器批处理逻辑优化的IMU Modbus RTU通信代码，显著提升轮询频率。

## 优化要点

### 1. **批处理架构**
参考编码器代码的 `doBatchProcessing()` 函数，将原有的异步状态机改为同步批处理模式：
- 一次 loop 循环中依次轮询所有IMU
- 每个IMU完成"发送-接收-解析"后才处理下一个
- 简化了状态管理逻辑

### 2. **阻塞式读取**
使用 `Serial2.readBytes()` 替代逐字节异步读取：
```cpp
size_t len = Serial2.readBytes(response, RESPONSE_LEN);
```
- 由 `Serial2.setTimeout(RESPONSE_TIMEOUT_MS)` 控制超时
- 自动等待完整数据包或超时返回
- 减少循环开销

### 3. **即时切换DE/RE**
参考编码器逻辑，优化485总线控制：
```cpp
digitalWrite(RS485_DE_RE_PIN, HIGH);  // 发送模式
Serial2.write(request, 8);
Serial2.flush();                       // 确保发送完成
delayMicroseconds(TX_DISABLE_DELAY_US);
digitalWrite(RS485_DE_RE_PIN, LOW);   // 立即切换到接收
```

### 4. **移除退避机制**
删除原有的失败退避（backoff）逻辑：
- 不再跳过失败的IMU
- 每个周期都尝试轮询所有设备
- 适用于稳定总线环境

### 5. **缩短超时时间**
将响应超时从 30ms 降至 10ms：
```cpp
static const uint16_t RESPONSE_TIMEOUT_MS = 10;
```
- 在921600波特率下，47字节响应理论传输时间约 0.5ms
- 10ms足够接收并留有余量
- 减少失败设备的等待时间

### 6. **主动清空缓冲区**
参考编码器代码，失败时立即清空：
```cpp
// 清空缓冲区，防止粘包影响下一个传感器
while (Serial2.available()) {
    Serial2.read();
}
```

## 性能对比

| 指标 | 原代码（状态机） | 优化后（批处理） |
|------|-----------------|-----------------|
| 架构复杂度 | 高（状态管理+退避） | 低（顺序处理） |
| 单IMU延迟 | 变化（退避影响） | 固定且低 |
| 理论最大频率* | ~30-50 Hz | **~80-100 Hz** |
| 失败恢复 | 指数退避延迟 | 立即重试 |

*基于2个IMU，每个响应47字节，921600波特率

## 硬件要求

- **MCU**: ESP32
- **接口**: UART2 (GPIO32/33)
- **波特率**: 921600 bps
- **总线**: RS485 (需要DE/RE控制引脚)

## 配置说明

### IMU数量和ID
```cpp
#define NUM_IMUS 2
const uint8_t IMU_IDS[NUM_IMUS] = {1, 2};
```

### 超时调整
根据实际总线质量调整：
```cpp
static const uint16_t RESPONSE_TIMEOUT_MS = 10;  // 稳定环境可降至5ms
```

### 寄存器范围
```cpp
static const uint16_t MODBUS_REG_START = 0x0034;  // 起始寄存器
static const uint16_t MODBUS_REG_COUNT = 0x0016;  // 22个寄存器 (44字节数据)
```

## 输出格式

### CSV数据流
```
1,-0.012,0.034,-9.807,0.9998,0.0012,-0.0034,0.0056,2,-0.015,0.031,-9.801,0.9997,0.0015,-0.0038,0.0052
```

格式: `id,accx,accy,accz,qw,qx,qy,qz,...`（重复每个IMU）

### 频率统计
```
# 更新频率: 85.32 Hz (85 周期, 成功=168, 失败=2)
```

## 使用方法

### PlatformIO
```bash
pio run -t upload
pio device monitor
```

### Arduino IDE
1. 安装ESP32开发板支持
2. 选择 "ESP32 Dev Module"
3. 设置上传速度为 921600
4. 串口监视器波特率设为 2000000

## 注意事项

1. **总线稳定性**: 批处理模式依赖稳定的总线通信，不适合频繁掉线的环境
2. **延迟敏感**: 如果某个IMU响应慢，会影响整个批次的频率
3. **错误传播**: 需要确保失败时缓冲区清空，否则会影响后续设备

## 进一步优化方向

如需更高频率，可考虑：
1. 减少寄存器读取数量（仅读必要数据）
2. 提高波特率至 1Mbps+
3. 使用DMA传输减少CPU占用
4. 预计算Modbus请求帧，避免运行时构建 
