# Quad IMU Data Handling - STM32F10x

本项目实现了使用STM32F10x微控制器同时接收4路IMU（惯性测量单元）数据，并将整合后的数据通过单一串口发送出去的功能。

## 功能特性

- 支持同时接收4路IMU数据
- 支持中断接收和DMA接收两种模式
- 数据整合后通过USART1统一输出
- 支持文本格式和二进制格式输出
- 完整的数据解析和错误处理

## 硬件连接

| 串口 | 引脚 (TX/RX) | 功能 |
|------|--------------|------|
| USART1 | PA9/PA10 | 数据输出（连接PC或其他设备） |
| USART2 | PA2/PA3 | IMU1 数据接收 |
| USART3 | PB10/PB11 | IMU2 数据接收 |
| UART4 | PC10/PC11 | IMU3 数据接收 |
| UART5 | PC12/PD2 | IMU4 数据接收 |

## 配置选项

在 `quad_imu_config.h` 或 `main.c` 中可以配置以下选项：

```c
/* 启用/禁用固定数据数组解码示例 */
#define ENABLE_FIXED_DATA_EXAMPLE     0

/* 启用/禁用DMA接收模式（0=中断模式，1=DMA模式）*/
#define ENABLE_USART_DMA              0

/* IMU设备数量 */
#define NUM_IMU                       4

/* 各串口波特率 */
#define USART1_BAUD 115200
#define USART2_BAUD 115200
#define USART3_BAUD 115200
#define UART4_BAUD  115200
#define UART5_BAUD  115200
```

## 数据输出格式

### 文本格式输出

整合后的IMU数据以文本格式输出，包含：
- 加速度计数据 (ACC)
- 陀螺仪数据 (GYR)
- 磁力计数据 (MAG)
- 欧拉角 (EULER)
- 四元数 (QUAT)
- 气压数据 (PRES)
- 时间戳 (TS)

示例输出：
```
====== Combined IMU Data ======
--- IMU1 (Valid, Frame Len:XX) ---
  ACC: 0.012, -0.034, 9.812 (m/s^2)
  GYR: 0.123, -0.456, 0.789 (deg/s)
  MAG: 12.34, -56.78, 90.12 (uT)
  EULER: Roll=1.23, Pitch=-4.56, Yaw=78.90 (deg)
  QUAT: 0.9876, 0.0123, -0.0456, 0.0789
  PRES: 101325.00 Pa, TS: 12345 ms
--- IMU2 (Valid, Frame Len:XX) ---
  ...
==============================
Valid IMUs: 4/4
==============================
```

### 二进制格式输出（可选）

使用 `quad_imu_binary_output.c` 中的功能可以输出紧凑的二进制数据包：

```
+--------+--------+--------+--------+------+------+------+------+--------+
| Header | Length | SeqNum | Valid  | IMU1 | IMU2 | IMU3 | IMU4 | CRC16  |
| 0x5A   | 0xA5   | 2bytes | Mask   | Data | Data | Data | Data | 2bytes |
+--------+--------+--------+--------+------+------+------+------+--------+
```

每个IMU数据块包含72字节数据。

## 文件结构

```
├── README.md                    # 项目说明文档
├── src/
│   ├── main.c                   # 主程序（4路IMU数据接收与整合）
│   └── quad_imu_binary_output.c # 二进制输出格式支持
└── inc/
    └── quad_imu_config.h        # 配置头文件
```

## 依赖项

- STM32F10x标准外设库
- HiPNUC IMU解码库 (`hipnuc_dec.h`, `hipnuc_dec.c`)
- delay库 (`delay.h`, `delay.c`)

## DMA通道映射

| DMA通道 | 外设 | 说明 |
|---------|------|------|
| DMA1 Channel6 | USART2 RX | IMU1 |
| DMA1 Channel3 | USART3 RX | IMU2 |
| DMA2 Channel3 | UART4 RX | IMU3 |
| - | UART5 RX | IMU4（不支持DMA，始终使用中断模式）|

## 注意事项

1. **UART5 DMA限制**：STM32F10x系列的UART5不支持DMA功能，即使启用DMA模式，UART5也会使用中断接收。

2. **缓冲区大小**：每个IMU通道有512字节的接收缓冲区，如果IMU发送的数据帧较大，可能需要调整 `UART_RX_BUF_SIZE`。

3. **波特率**：所有IMU必须以115200波特率发送数据。如需修改，请在配置中更改相应的波特率定义。

4. **MCU型号**：本代码适用于STM32F10x大容量或互联型设备（需要UART4和UART5）。

## 使用方法

1. 将代码添加到您的STM32工程中
2. 确保已包含必要的依赖库
3. 根据实际硬件连接修改引脚配置（如需要）
4. 编译并下载到目标板
5. 连接4个IMU设备到相应的串口
6. 通过USART1查看整合后的数据输出

## 许可证

本项目仅供学习和演示目的，请参考原始代码的许可证要求。

## 参考

- [HiPNUC官网](http://www.hipnuc.com)
- STM32F10x参考手册
