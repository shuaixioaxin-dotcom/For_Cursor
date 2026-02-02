/**
 * ESP32 编码器数据采集系统 - DMA优化版本
 * 
 * 此版本使用ESP32 UART驱动的DMA功能，通过中断方式处理数据
 * 性能更高，CPU占用更低
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include "config.h"

// ==================== UART DMA 配置 ====================

#define UART_NUM_0 UART_NUM_0
#define UART_NUM_1 UART_NUM_1
#define UART_NUM_2 UART_NUM_2

// UART 配置结构
typedef struct {
    uart_port_t uart_num;
    int tx_pin;
    int rx_pin;
    int de_pin;
    uint8_t encoder_id[2];  // 该UART对应的两个编码器ID
} UartChannel;

UartChannel uartChannels[3] = {
    {UART_NUM_0, UART0_TX_PIN, UART0_RX_PIN, UART0_DE_PIN, {0, 1}},
    {UART_NUM_1, UART1_TX_PIN, UART1_RX_PIN, UART1_DE_PIN, {2, 3}},
    {UART_NUM_2, UART2_TX_PIN, UART2_RX_PIN, UART2_DE_PIN, {4, 5}}
};

// ==================== 全局变量 ====================

// 编码器数据缓冲区（双缓冲机制）
uint8_t encoderDataBuffer[2][ENCODER_COUNT][RESPONSE_FRAME_SIZE];
volatile uint8_t currentBuffer = 0;  // 当前写入缓冲区
volatile uint8_t dataValidFlags = 0;

// ESP-NOW
EncoderDataPacket espnowPacket;
uint16_t packetSequence = 0;
uint8_t receiverMacAddress[] = RECEIVER_MAC;

// 同步信号量
SemaphoreHandle_t dataReadySemaphore;
QueueHandle_t uart_queue[3];

// 性能统计
volatile uint32_t successCount = 0;
volatile uint32_t errorCount = 0;
uint32_t lastStatsTime = 0;

// ==================== UART DMA 初始化 ====================

void initUartWithDMA(UartChannel &channel) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };
    
    // 配置UART参数
    uart_param_config(channel.uart_num, &uart_config);
    
    // 设置引脚
    uart_set_pin(channel.uart_num, channel.tx_pin, channel.rx_pin, 
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    // 安装UART驱动，启用DMA
    uart_driver_install(channel.uart_num, UART_RX_BUF_SIZE * 2, 
                       UART_TX_BUF_SIZE * 2, 10, &uart_queue[channel.uart_num], 0);
    
    // 配置RS485控制引脚
    gpio_set_direction((gpio_num_t)channel.de_pin, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)channel.de_pin, 0);  // 默认接收模式
}

// ==================== RS485 控制（优化版） ====================

inline void rs485SetTransmit(int dePin) {
    gpio_set_level((gpio_num_t)dePin, 1);
    ets_delay_us(5);  // 使用更精确的延迟
}

inline void rs485SetReceive(int dePin) {
    gpio_set_level((gpio_num_t)dePin, 0);
    ets_delay_us(5);
}

// ==================== 编码器读取（DMA版本） ====================

bool readEncoderDMA(UartChannel &channel, uint8_t encoderCmd, uint8_t *dataBuffer) {
    // 清空UART缓冲区
    uart_flush_input(channel.uart_num);
    
    // 切换到发送模式
    rs485SetTransmit(channel.de_pin);
    
    // 发送命令
    uart_write_bytes(channel.uart_num, (const char*)&encoderCmd, 1);
    uart_wait_tx_done(channel.uart_num, pdMS_TO_TICKS(10));
    
    // 切换到接收模式
    rs485SetReceive(channel.de_pin);
    
    // 使用DMA接收数据
    int len = uart_read_bytes(channel.uart_num, dataBuffer, 
                             RESPONSE_FRAME_SIZE, pdMS_TO_TICKS(1));
    
    return (len == RESPONSE_FRAME_SIZE);
}

// ==================== 数据采集任务 ====================

void dataAcquisitionTask(void *parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(1000 / TARGET_FREQ_HZ);  // 5ms for 200Hz
    
    while (true) {
        uint32_t startTime = micros();
        uint8_t writeBuffer = currentBuffer;
        bool allSuccess = true;
        uint8_t validFlags = 0;
        
        // 顺序读取3路UART的6个编码器
        for (int ch = 0; ch < 3; ch++) {
            UartChannel &channel = uartChannels[ch];
            
            // 读取该通道的两个编码器
            for (int i = 0; i < 2; i++) {
                uint8_t encoderId = channel.encoder_id[i];
                uint8_t cmd = ENCODER_CMD_BASE + encoderId;
                
                bool success = readEncoderDMA(channel, cmd, 
                                             encoderDataBuffer[writeBuffer][encoderId]);
                
                if (success) {
                    validFlags |= (1 << encoderId);
                } else {
                    allSuccess = false;
                    errorCount++;
                }
            }
        }
        
        // 更新有效性标志
        dataValidFlags = validFlags;
        
        if (allSuccess) {
            successCount++;
        }
        
        // 通知数据准备就绪
        xSemaphoreGive(dataReadySemaphore);
        
        uint32_t elapsedTime = micros() - startTime;
        
        // 周期性任务延迟
        vTaskDelayUntil(&lastWakeTime, frequency);
    }
}

// ==================== ESP-NOW 发送任务 ====================

void espnowSendTask(void *parameter) {
    while (true) {
        // 等待数据准备就绪
        if (xSemaphoreTake(dataReadySemaphore, portMAX_DELAY) == pdTRUE) {
            // 准备数据包
            espnowPacket.timestamp = micros();
            espnowPacket.sequence = packetSequence++;
            
            // 复制编码器数据（从当前缓冲区）
            uint8_t readBuffer = currentBuffer;
            memcpy(espnowPacket.encoder_data, encoderDataBuffer[readBuffer], 
                   sizeof(encoderDataBuffer[0]));
            
            espnowPacket.valid_flags = dataValidFlags;
            espnowPacket.sample_rate = TARGET_FREQ_HZ;
            
            // 计算校验和
            uint8_t checksum = 0;
            uint8_t *data_ptr = (uint8_t*)&espnowPacket;
            for (size_t i = 0; i < sizeof(EncoderDataPacket) - 1; i++) {
                checksum ^= data_ptr[i];
            }
            espnowPacket.checksum = checksum;
            
            // 发送数据
            esp_now_send(receiverMacAddress, (uint8_t*)&espnowPacket, 
                        sizeof(EncoderDataPacket));
        }
    }
}

// ==================== ESP-NOW 回调 ====================

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // 可以在这里添加发送状态处理
}

// ==================== 初始化 ESP-NOW ====================

void initESPNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW 初始化失败");
        return;
    }
    
    esp_now_register_send_cb(onDataSent);
    
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("添加对等设备失败");
        return;
    }
    
    Serial.println("ESP-NOW 初始化成功");
    Serial.printf("本机MAC: %s\n", WiFi.macAddress().c_str());
}

// ==================== 统计任务 ====================

void statsTask(void *parameter) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));  // 每秒更新一次
        
        Serial.printf("采样统计 - 成功: %lu, 失败: %lu, 成功率: %.2f%%\n",
                     successCount, errorCount,
                     (successCount * 100.0) / (successCount + errorCount + 0.001));
        
        // 重置计数器
        successCount = 0;
        errorCount = 0;
    }
}

// ==================== Setup ====================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n========================================");
    Serial.println("ESP32 编码器数据采集系统 (DMA优化版)");
    Serial.println("========================================");
    
    // 创建信号量
    dataReadySemaphore = xSemaphoreCreateBinary();
    
    // 初始化3路UART
    for (int i = 0; i < 3; i++) {
        initUartWithDMA(uartChannels[i]);
        Serial.printf("UART%d 初始化完成 (DMA模式)\n", i);
    }
    
    // 初始化ESP-NOW
    initESPNow();
    
    Serial.printf("目标频率: %d Hz\n", TARGET_FREQ_HZ);
    Serial.println("系统初始化完成，启动任务...");
    
    // 创建数据采集任务（高优先级）
    xTaskCreatePinnedToCore(
        dataAcquisitionTask,
        "DataAcq",
        4096,
        NULL,
        3,  // 高优先级
        NULL,
        1   // 核心1
    );
    
    // 创建ESP-NOW发送任务（中优先级）
    xTaskCreatePinnedToCore(
        espnowSendTask,
        "ESPNowSend",
        4096,
        NULL,
        2,  // 中优先级
        NULL,
        0   // 核心0
    );
    
    // 创建统计任务（低优先级）
    xTaskCreatePinnedToCore(
        statsTask,
        "Stats",
        2048,
        NULL,
        1,  // 低优先级
        NULL,
        0   // 核心0
    );
    
    Serial.println("所有任务已启动");
}

// ==================== Loop ====================

void loop() {
    // 主循环空闲，所有工作由任务完成
    vTaskDelay(pdMS_TO_TICKS(1000));
}
