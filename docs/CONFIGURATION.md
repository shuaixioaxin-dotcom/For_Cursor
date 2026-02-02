# 配置指南

## 配置文件说明

所有配置集中在 `include/config.h` 文件中。

## 必须配置的参数

### 1. GPIO引脚分配

根据实际接线修改：

```cpp
// UART0 - 编码器 0,1
#define UART0_TX_PIN    1      // 修改为实际使用的TX引脚
#define UART0_RX_PIN    3      // 修改为实际使用的RX引脚
#define UART0_DE_PIN    4      // 修改为实际使用的DE/RE引脚

// UART1 - 编码器 2,3
#define UART1_TX_PIN    17
#define UART1_RX_PIN    16
#define UART1_DE_PIN    5

// UART2 - 编码器 4,5
#define UART2_TX_PIN    25
#define UART2_RX_PIN    26
#define UART2_DE_PIN    27
```

### 2. 接收端MAC地址

**步骤：**

1. 先上传接收端程序（receiver_example.cpp）
2. 打开串口监视器，记录显示的MAC地址
3. 在发送端的config.h中填入：

```cpp
// 示例：假设接收端MAC是 AA:BB:CC:DD:EE:FF
#define RECEIVER_MAC {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}
```

## 可选配置参数

### 1. 波特率调整

```cpp
// 默认2.5Mbps
#define UART_BAUD_RATE  2500000

// 如果通信不稳定，可以降低：
// #define UART_BAUD_RATE  1000000   // 1Mbps
// #define UART_BAUD_RATE  500000    // 500Kbps
```

### 2. 采样频率

```cpp
// 默认200Hz
#define TARGET_FREQ_HZ 200

// 如果需要更高频率：
// #define TARGET_FREQ_HZ 400   // 400Hz (需要测试验证)
// #define TARGET_FREQ_HZ 500   // 500Hz (可能不稳定)
```

**频率限制计算：**
```
单次采集时间 = 6个编码器 × 92μs ≈ 550μs
理论最高频率 = 1000000μs / 550μs ≈ 1818Hz
实际建议最高 = 500Hz（考虑处理时间）
```

### 3. 超时时间

```cpp
// 默认1ms超时
#define RESPONSE_TIMEOUT_US 1000

// 如果线路较长或波特率较低，可以增加：
// #define RESPONSE_TIMEOUT_US 2000   // 2ms
```

### 4. 编码器命令

```cpp
// 默认从0x50开始
#define ENCODER_CMD_BASE 0x50

// 如果您的编码器使用不同的命令：
// #define ENCODER_CMD_BASE 0x30  // 根据实际协议修改
```

### 5. 响应帧长度

```cpp
// 默认22字节
#define RESPONSE_FRAME_SIZE 22

// 如果您的编码器响应不同长度：
// #define RESPONSE_FRAME_SIZE 16  // 根据实际协议修改
```

### 6. 编码器数量

```cpp
// 默认6个编码器
#define ENCODER_COUNT 6

// 如果需要更多或更少：
// #define ENCODER_COUNT 9   // 3路×3个
// #define ENCODER_COUNT 3   // 3路×1个
```

**注意：** 修改编码器数量后，需要同步修改`main.cpp`中的读取逻辑！

### 7. 调试输出

```cpp
// 启用调试输出（会降低性能）
#define ENABLE_DEBUG_OUTPUT 1

// 生产环境建议关闭：
// #define ENABLE_DEBUG_OUTPUT 0
```

### 8. 性能统计

```cpp
// 启用性能统计
#define ENABLE_PERFORMANCE_STATS 1

// 如果不需要统计，可以关闭节省CPU：
// #define ENABLE_PERFORMANCE_STATS 0
```

## 高级配置

### 1. UART缓冲区大小

```cpp
// 默认配置
#define UART_RX_BUF_SIZE 256
#define UART_TX_BUF_SIZE 64

// 如果出现数据丢失，可以增加：
// #define UART_RX_BUF_SIZE 512
// #define UART_TX_BUF_SIZE 128
```

### 2. ESP-NOW信道

在`main.cpp`的`initESPNow()`函数中：

```cpp
// 设置固定信道，避免干扰
WiFi.channel(6);  // 可选1-13，建议6或11
```

### 3. FreeRTOS任务优先级（DMA版本）

在`main_with_dma.cpp`的`setup()`中：

```cpp
// 数据采集任务（最高优先级）
xTaskCreatePinnedToCore(
    dataAcquisitionTask,
    "DataAcq",
    4096,      // 堆栈大小
    NULL,
    3,         // 优先级：3=高，2=中，1=低
    NULL,
    1          // CPU核心：0或1
);
```

## 针对不同场景的配置建议

### 场景1：开发调试

```cpp
#define UART_BAUD_RATE  1000000      // 降低波特率提高稳定性
#define TARGET_FREQ_HZ 50            // 降低频率便于观察
#define ENABLE_DEBUG_OUTPUT 1        // 启用详细日志
#define ENABLE_PERFORMANCE_STATS 1   // 启用性能监控
#define RESPONSE_TIMEOUT_US 2000     // 增加超时容错
```

### 场景2：生产环境

```cpp
#define UART_BAUD_RATE  2500000      // 最高速度
#define TARGET_FREQ_HZ 200           // 目标频率
#define ENABLE_DEBUG_OUTPUT 0        // 关闭调试降低开销
#define ENABLE_PERFORMANCE_STATS 0   // 关闭统计
#define RESPONSE_TIMEOUT_US 1000     // 标准超时
```

### 场景3：长距离传输（>5米）

```cpp
#define UART_BAUD_RATE  1000000      // 降低波特率
#define RESPONSE_TIMEOUT_US 3000     // 增加超时时间
// 建议使用更好的线缆和终端电阻
```

### 场景4：极限性能测试

```cpp
#define UART_BAUD_RATE  2500000
#define TARGET_FREQ_HZ 500           // 尝试更高频率
#define ENABLE_DEBUG_OUTPUT 0        // 必须关闭
#define ENABLE_PERFORMANCE_STATS 0   // 必须关闭
// 使用 main_with_dma.cpp 版本
```

## 配置验证

### 编译时检查

代码会在编译时进行一些基本检查，如果配置不合理会报错：

```cpp
// 在 config.h 中添加：
#if (1000000 / TARGET_FREQ_HZ) < 500
#error "采样频率过高！单次采集至少需要500us"
#endif

#if ENCODER_COUNT > 10
#error "编码器数量过多！ESP32只有3个UART"
#endif
```

### 运行时验证

在接收端串口监视器中观察：

```
✓ 数据包序列号连续递增
✓ 采样率接近目标值（如200Hz）
✓ 所有编码器的valid_flags都为1（0x3F = 0b00111111）
✓ 无超时或校验错误
```

## 故障诊断配置

### 诊断模式配置

```cpp
// 单独测试每个UART
#define TEST_MODE 1  // 0=正常，1=只测试UART0，2=只测试UART1，3=只测试UART2

// 在main.cpp中添加条件编译：
#if TEST_MODE == 1
    // 只初始化和读取UART0
#endif
```

### 详细日志配置

```cpp
// 输出详细时序信息
#define DEBUG_TIMING 1

// 在readEncoder函数中添加：
#if DEBUG_TIMING
Serial.printf("Encoder %d: Send=%luus, Wait=%luus, Total=%luus\n",
              encoderId, sendTime, waitTime, totalTime);
#endif
```

## 配置文件模板

在`include/`目录下可以创建多个配置模板：

```
include/
├── config.h                 # 当前使用的配置
├── config_debug.h.template  # 调试模板
├── config_production.h.template  # 生产模板
└── config_test.h.template   # 测试模板
```

切换配置：
```bash
cp include/config_debug.h.template include/config.h
```

## 常见配置错误

### ❌ 错误1：MAC地址全为FF

```cpp
// 错误：使用广播地址
#define RECEIVER_MAC {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

// 正确：使用接收端的实际MAC
#define RECEIVER_MAC {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF}
```

### ❌ 错误2：引脚冲突

```cpp
// 错误：多个功能使用同一引脚
#define UART0_TX_PIN 1
#define UART1_TX_PIN 1  // 冲突！

// 正确：每个引脚唯一
#define UART0_TX_PIN 1
#define UART1_TX_PIN 17
```

### ❌ 错误3：缓冲区过小

```cpp
// 错误：缓冲区小于响应帧
#define UART_RX_BUF_SIZE 16
#define RESPONSE_FRAME_SIZE 22  // 会溢出！

// 正确：缓冲区至少是响应帧的2倍
#define UART_RX_BUF_SIZE 256
#define RESPONSE_FRAME_SIZE 22
```

## 性能调优建议

1. **优先级顺序：** 稳定性 > 速度 > CPU占用
2. **逐步优化：** 先降低配置确保工作，再逐步提升
3. **实测为准：** 不同硬件环境性能差异大
4. **保留余量：** 不要将参数配置到极限值

配置完成后，建议先用基础版本（main.cpp）验证，稳定后再切换到DMA优化版本。