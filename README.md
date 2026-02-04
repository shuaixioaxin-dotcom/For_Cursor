# ESP32 IMU Reader - 200Hz 优化版

## 概述

这是一个用于ESP32的Modbus RTU IMU数据读取程序，优化目标是达到200Hz的数据输出频率。

## 优化内容

### 1. 时序参数优化

| 参数 | 原值 | 优化值 | 说明 |
|------|------|--------|------|
| RESPONSE_TIMEOUT_MS | 30ms | 2ms | 减少超时等待时间 |
| BUS_SILENCE_US | 500us | 50us | 减少总线静默时间 |
| TX_ENABLE_DELAY_US | 30us | 10us | 减少方向切换延迟 |
| TX_DISABLE_DELAY_US | 60us | 20us | 减少方向切换延迟 |
| FAIL_COOLDOWN_MS | 20ms | 5ms | 减少失败冷却时间 |
| MAX_BACKOFF_SHIFT | 4 | 2 | 减少最大退避 |

### 2. 算法优化

- **CRC16查表法**：使用256项查表替代位运算，提升约4倍速度
- **快速浮点转字符串**：自定义`ftoa_fast()`函数替代`sprintf()`
- **批量输出**：使用缓冲区一次性写入串口，减少函数调用开销

### 3. 定时优化

- **微秒级定时**：使用`micros()`替代`millis()`，精度从±1ms提升到±1us
- **固定间隔输出**：使用累加方式(`last_output_us += OUTPUT_INTERVAL_US`)避免定时漂移
- **200Hz固定输出**：每5000us输出一次数据，与通信解耦

### 4. 通信优化

- **增大RX缓冲区**：`Serial2.setRxBufferSize(256)`
- **预构建请求帧**：启动时构建所有请求帧缓存
- **提前CRC验证**：在帧提取时验证CRC，避免无效解析

## 通信时序分析

```
波特率: 921600 bps
每字节: 10位 (8数据 + 1起始 + 1停止)

请求帧: 8字节 × 10位 ÷ 921600 = 0.087ms
响应帧: 49字节 × 10位 ÷ 921600 = 0.532ms
单IMU通信: ~0.62ms

2个IMU理论最小周期: ~1.24ms
加上处理开销: ~1.5-2ms

200Hz (5ms周期) 有足够余量
```

## 性能统计

程序每10秒输出一次性能统计：
```
# STATS: freq=200.0Hz, success_rate=100.0%, success=4000, fail=0
```

## 硬件连接

```
ESP32 Pin 32 -> RS485模块 RO (接收)
ESP32 Pin 33 -> RS485模块 DI (发送)
ESP32 Pin 25 -> RS485模块 DE/RE (方向控制)
```

## 编译配置

使用PlatformIO：
```bash
pio run
pio run --target upload
pio device monitor
```

## 输出格式

每行输出所有IMU数据，逗号分隔：
```
acc_x1,acc_y1,acc_z1,qw1,qx1,qy1,qz1,acc_x2,acc_y2,acc_z2,qw2,qx2,qy2,qz2
```

- 加速度：3位小数
- 四元数：4位小数
- 无效数据显示为0

## 注意事项

1. 确保IMU设备波特率设置为921600
2. 确保IMU设备ID分别为1和2
3. 2M串口波特率需要USB串口芯片支持
4. 若通信不稳定，可适当增加RESPONSE_TIMEOUT_MS
