# Modbus编码器超高频批量读取系统

## 项目简介

这是一个基于ESP32的超高频Modbus编码器批量读取系统，通过RS485总线同时读取6个编码器的实时角度数据。

### 核心特性

- ⚡ **超高频率**：通过双核优化和激进超时策略，实现 **1000+ Hz** 的数据采集频率
- 🔄 **批量处理**：一次循环完成所有6个编码器的顺序读取
- 🚀 **双核并行**：Core 0专注数据采集，Core 1负责输出显示
- 💡 **实时反馈**：WS2812 LED显示通讯状态（绿色=正常，红色=异常）
- 📊 **CSV输出**：实时输出逗号分隔的角度数据，便于数据采集和分析

### 主要优化措施

1. **移除Serial2.flush()** - 节省约40-60us/编码器的等待时间
2. **双核分离处理** - 消除LED和串口输出对采集循环的阻塞
3. **更激进的超时参数** - 减少不必要的等待时间
4. **内联关键函数** - 减少函数调用开销
5. **双缓冲无锁设计** - 零锁等待的数据交换
6. **编译器优化** - 使用-O3和循环展开等优化选项

### 性能指标

| 指标 | 数值 |
|------|------|
| 目标频率 | 1000-1200 Hz |
| 波特率 | 2.5 Mbps |
| 编码器数量 | 6 个 |
| 单周期时间 | < 900 us |
| 数据输出频率 | 100 Hz (10ms间隔) |

## 硬件要求

- ESP32开发板（需双核支持）
- RS485收发器模块
- 6个Modbus RTU编码器（地址1-6）
- WS2812 LED（可选，用于状态指示）
- 蜂鸣器（可选，用于启动提示音）

## 接线说明

| ESP32引脚 | 连接 |
|-----------|------|
| GPIO32 | RS485 RX |
| GPIO33 | RS485 TX |
| GPIO25 | RS485 DE/RE |
| GPIO26 | WS2812 Data |
| GPIO2  | 蜂鸣器 |

## 快速开始

### 1. 克隆项目
```bash
git clone <repository-url>
cd modbus-encoder-batch-optimization-c94c
```

### 2. 编译和上传
```bash
# 使用PlatformIO
pio run --target upload

# 或使用PlatformIO IDE
# 点击 "Upload" 按钮
```

### 3. 监控输出
```bash
pio device monitor -b 2000000
```

### 预期输出示例
```
# System Ready - Ultra-Fast Dual-Core Mode (2.5Mbps)
# Optimizations: No flush() + Aggressive timeouts + Dual-core separation
12.45,23.67,89.12,156.78,234.56,312.34
13.50,24.12,90.45,157.23,235.67,313.89
# Update Frequency: 1120.35 Hz
...
```

## 详细文档

- [完整优化指南](OPTIMIZATION_GUIDE.md) - 详细的优化措施和性能分析
- [源代码](src/main.cpp) - 包含详细注释的主程序

## 配置说明

### 修改编码器数量
```cpp
// src/main.cpp
#define NUM_ENCODERS 6  // 修改为实际编码器数量
const uint8_t ENCODER_IDS[NUM_ENCODERS] = {1, 2, 3, 4, 5, 6};  // 修改ID列表
```

### 调整波特率
```cpp
// src/main.cpp - setup()
Serial2.begin(2500000, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
// 根据实际情况可降低到 2000000 或 1000000
```

### 修改输出频率
```cpp
// src/main.cpp - taskOutputDisplay()
const uint32_t OUTPUT_INTERVAL = 10;  // 输出间隔(ms)，可调整为5/20/50等
```

## 故障排除

### 问题：频率不稳定或错误率高

**可能原因和解决方案：**

1. **线缆过长或质量差**
   - 使用屏蔽双绞线
   - 减少线缆长度到 < 2米
   - 添加终端电阻（120Ω）

2. **波特率过高**
   ```cpp
   Serial2.begin(2000000, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
   // 降低到 2Mbps 或 1Mbps
   ```

3. **超时参数过激进**
   ```cpp
   int len = readBytesFast(response, 7, 400, 100);  // 放宽到400/100
   ```

4. **编码器响应慢**
   - 检查编码器型号和固件版本
   - 某些编码器可能不支持高频轮询

### 问题：LED不亮或不更新

- 检查WS2812接线
- 确认FastLED库版本 >= 3.6.0
- 检查Core 1任务是否正常运行

### 问题：串口输出乱码

- 确认监控波特率为 2000000
- 检查USB线缆质量
- 尝试降低串口输出频率

## 性能测试

运行24小时稳定性测试：
```bash
# 记录输出到文件
pio device monitor -b 2000000 > test_log.txt

# 分析错误率
grep "Red" test_log.txt | wc -l
```

## 贡献

欢迎提交Issue和Pull Request！

## 许可证

MIT License

## 作者

shuaixioaxin-dotcom

## 更新日志

### 2026-02-03
- 实现双核并行处理
- 移除Serial2.flush()优化
- 优化超时参数
- 添加详细文档
