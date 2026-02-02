#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==================== 硬件配置 ====================

// RS485 UART 配置 - 3路并行
// 第1路：UART0 - 编码器 0,1
#define UART0_TX_PIN    1
#define UART0_RX_PIN    3
#define UART0_DE_PIN    4   // RS485 DE/RE 控制引脚

// 第2路：UART1 - 编码器 2,3
#define UART1_TX_PIN    17
#define UART1_RX_PIN    16
#define UART1_DE_PIN    5   // RS485 DE/RE 控制引脚

// 第3路：UART2 - 编码器 4,5
#define UART2_TX_PIN    25
#define UART2_RX_PIN    26
#define UART2_DE_PIN    27  // RS485 DE/RE 控制引脚

// ==================== 通信参数 ====================

// 波特率：2.5Mbps
#define UART_BAUD_RATE  2500000

// UART 缓冲区大小
#define UART_RX_BUF_SIZE 256
#define UART_TX_BUF_SIZE 64

// ==================== 协议配置 ====================

// 编码器数量
#define ENCODER_COUNT 6

// 多摩川协议命令
#define ENCODER_CMD_BASE 0x50  // 编码器0命令为0x50(80)，编码器1为0x51(81)...

// 响应帧长度
#define RESPONSE_FRAME_SIZE 22

// 超时时间（微秒）
#define RESPONSE_TIMEOUT_US 1000  // 1ms超时

// ==================== 采样配置 ====================

// 目标采样频率：200Hz
#define TARGET_FREQ_HZ 200
#define SAMPLING_INTERVAL_US (1000000 / TARGET_FREQ_HZ)  // 5000us = 5ms

// ==================== ESP-NOW 配置 ====================

// 接收端MAC地址（需要根据实际接收设备修改）
// 示例MAC地址，请替换为实际地址
#define RECEIVER_MAC {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}

// ESP-NOW 数据包结构
struct EncoderDataPacket {
    uint32_t timestamp;           // 时间戳（微秒）
    uint16_t sequence;            // 序列号
    uint8_t encoder_data[ENCODER_COUNT][RESPONSE_FRAME_SIZE];  // 6个编码器的数据
    uint8_t valid_flags;          // 数据有效性标志位（bit0-5对应编码器0-5）
    uint16_t sample_rate;         // 实际采样率（Hz）
    uint8_t checksum;             // 校验和
} __attribute__((packed));

// ==================== 调试配置 ====================

// 启用串口调试输出
#define ENABLE_DEBUG_OUTPUT 1

// 性能统计
#define ENABLE_PERFORMANCE_STATS 1

#endif // CONFIG_H
