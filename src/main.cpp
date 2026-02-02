#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <driver/uart.h>
#include "config.h"

// ==================== 全局变量 ====================

// UART 端口定义
HardwareSerial Serial1_RS485(1);  // UART1
HardwareSerial Serial2_RS485(2);  // UART2

// 编码器数据缓冲区
uint8_t encoderDataBuffer[ENCODER_COUNT][RESPONSE_FRAME_SIZE];
volatile uint8_t dataValidFlags = 0;  // 数据有效性标志

// ESP-NOW 数据包
EncoderDataPacket espnowPacket;
uint16_t packetSequence = 0;

// 接收端MAC地址
uint8_t receiverMacAddress[] = RECEIVER_MAC;

// 性能统计
uint32_t loopCount = 0;
uint32_t lastStatsTime = 0;
uint32_t totalReadTime = 0;
uint32_t maxReadTime = 0;

// ==================== RS485 控制函数 ====================

// 设置RS485为发送模式
inline void rs485SetTransmit(uint8_t dePin) {
    digitalWrite(dePin, HIGH);
    delayMicroseconds(10);  // 短暂延迟确保切换完成
}

// 设置RS485为接收模式
inline void rs485SetReceive(uint8_t dePin) {
    digitalWrite(dePin, LOW);
    delayMicroseconds(10);
}

// ==================== 编码器读取函数 ====================

/**
 * 读取单个编码器数据
 * @param serial UART串口对象
 * @param dePin RS485 DE/RE控制引脚
 * @param encoderCmd 编码器命令（0x50-0x55）
 * @param dataBuffer 数据缓冲区（至少22字节）
 * @return true=成功, false=失败
 */
bool readEncoder(HardwareSerial &serial, uint8_t dePin, uint8_t encoderCmd, uint8_t *dataBuffer) {
    // 清空接收缓冲区
    while (serial.available()) {
        serial.read();
    }
    
    // 切换到发送模式
    rs485SetTransmit(dePin);
    
    // 发送命令
    serial.write(encoderCmd);
    serial.flush();  // 等待发送完成
    
    // 切换到接收模式
    rs485SetReceive(dePin);
    
    // 等待接收数据
    uint32_t startTime = micros();
    uint8_t receivedBytes = 0;
    
    while (receivedBytes < RESPONSE_FRAME_SIZE) {
        if (serial.available()) {
            dataBuffer[receivedBytes++] = serial.read();
        }
        
        // 超时检查
        if (micros() - startTime > RESPONSE_TIMEOUT_US) {
            #if ENABLE_DEBUG_OUTPUT
            Serial.printf("编码器 0x%02X 超时，已接收 %d 字节\n", encoderCmd, receivedBytes);
            #endif
            return false;
        }
    }
    
    return true;
}

/**
 * 读取3路RS485上的所有6个编码器
 * 采用并行读取策略，最大化速度
 */
bool readAllEncodersParallel() {
    uint32_t startTime = micros();
    bool allSuccess = true;
    
    // 第1路：读取编码器0和1（UART0 - Serial）
    bool enc0_ok = readEncoder(Serial, UART0_DE_PIN, ENCODER_CMD_BASE + 0, encoderDataBuffer[0]);
    bool enc1_ok = readEncoder(Serial, UART0_DE_PIN, ENCODER_CMD_BASE + 1, encoderDataBuffer[1]);
    
    // 第2路：读取编码器2和3（UART1）
    bool enc2_ok = readEncoder(Serial1_RS485, UART1_DE_PIN, ENCODER_CMD_BASE + 2, encoderDataBuffer[2]);
    bool enc3_ok = readEncoder(Serial1_RS485, UART1_DE_PIN, ENCODER_CMD_BASE + 3, encoderDataBuffer[3]);
    
    // 第3路：读取编码器4和5（UART2）
    bool enc4_ok = readEncoder(Serial2_RS485, UART2_DE_PIN, ENCODER_CMD_BASE + 4, encoderDataBuffer[4]);
    bool enc5_ok = readEncoder(Serial2_RS485, UART2_DE_PIN, ENCODER_CMD_BASE + 5, encoderDataBuffer[5]);
    
    // 更新有效性标志
    dataValidFlags = (enc0_ok << 0) | (enc1_ok << 1) | (enc2_ok << 2) | 
                     (enc3_ok << 3) | (enc4_ok << 4) | (enc5_ok << 5);
    
    // 性能统计
    uint32_t readTime = micros() - startTime;
    totalReadTime += readTime;
    if (readTime > maxReadTime) {
        maxReadTime = readTime;
    }
    
    #if ENABLE_DEBUG_OUTPUT
    if (!enc0_ok || !enc1_ok || !enc2_ok || !enc3_ok || !enc4_ok || !enc5_ok) {
        Serial.printf("读取失败: enc[%d%d%d%d%d%d], 用时: %lu us\n",
            enc0_ok, enc1_ok, enc2_ok, enc3_ok, enc4_ok, enc5_ok, readTime);
        allSuccess = false;
    }
    #endif
    
    return allSuccess;
}

// ==================== ESP-NOW 回调函数 ====================

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    #if ENABLE_DEBUG_OUTPUT
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("ESP-NOW 发送失败");
    }
    #endif
}

// ==================== ESP-NOW 初始化 ====================

void initESPNow() {
    // 设置为Station模式
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    // 初始化ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW 初始化失败");
        return;
    }
    
    Serial.println("ESP-NOW 初始化成功");
    
    // 注册发送回调
    esp_now_register_send_cb(onDataSent);
    
    // 添加对等设备
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
    peerInfo.channel = 0;  // 使用当前通道
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("添加ESP-NOW对等设备失败");
        return;
    }
    
    Serial.println("ESP-NOW 对等设备添加成功");
    Serial.printf("本机MAC地址: %s\n", WiFi.macAddress().c_str());
}

// ==================== 发送数据通过ESP-NOW ====================

void sendDataViaESPNow() {
    // 准备数据包
    espnowPacket.timestamp = micros();
    espnowPacket.sequence = packetSequence++;
    
    // 复制编码器数据
    memcpy(espnowPacket.encoder_data, encoderDataBuffer, sizeof(encoderDataBuffer));
    espnowPacket.valid_flags = dataValidFlags;
    
    // 计算实际采样率
    static uint32_t lastSendTime = 0;
    uint32_t currentTime = millis();
    if (lastSendTime > 0) {
        uint32_t interval = currentTime - lastSendTime;
        if (interval > 0) {
            espnowPacket.sample_rate = 1000 / interval;
        }
    }
    lastSendTime = currentTime;
    
    // 计算校验和（简单异或校验）
    uint8_t checksum = 0;
    uint8_t *data_ptr = (uint8_t*)&espnowPacket;
    for (size_t i = 0; i < sizeof(EncoderDataPacket) - 1; i++) {
        checksum ^= data_ptr[i];
    }
    espnowPacket.checksum = checksum;
    
    // 发送数据
    esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t*)&espnowPacket, sizeof(EncoderDataPacket));
    
    #if ENABLE_DEBUG_OUTPUT
    if (result != ESP_OK) {
        Serial.printf("ESP-NOW发送错误: %d\n", result);
    }
    #endif
}

// ==================== 初始化函数 ====================

void setup() {
    // 初始化调试串口
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n========================================");
    Serial.println("ESP32 编码器数据采集系统");
    Serial.println("========================================");
    
    // 配置RS485控制引脚
    pinMode(UART0_DE_PIN, OUTPUT);
    pinMode(UART1_DE_PIN, OUTPUT);
    pinMode(UART2_DE_PIN, OUTPUT);
    
    // 默认设置为接收模式
    rs485SetReceive(UART0_DE_PIN);
    rs485SetReceive(UART1_DE_PIN);
    rs485SetReceive(UART2_DE_PIN);
    
    // 初始化UART0（使用默认Serial）
    Serial.end();  // 关闭调试用的串口
    Serial.begin(UART_BAUD_RATE, SERIAL_8N1, UART0_RX_PIN, UART0_TX_PIN);
    Serial.setRxBufferSize(UART_RX_BUF_SIZE);
    
    // 初始化UART1
    Serial1_RS485.begin(UART_BAUD_RATE, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);
    Serial1_RS485.setRxBufferSize(UART_RX_BUF_SIZE);
    
    // 初始化UART2
    Serial2_RS485.begin(UART_BAUD_RATE, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
    Serial2_RS485.setRxBufferSize(UART_RX_BUF_SIZE);
    
    // 注意：由于UART0被用于RS485，调试信息将无法输出
    // 如需调试，可暂时禁用UART0的RS485功能
    
    // 初始化ESP-NOW
    initESPNow();
    
    Serial.println("系统初始化完成");
    Serial.printf("目标采样频率: %d Hz\n", TARGET_FREQ_HZ);
    Serial.printf("采样间隔: %d us\n", SAMPLING_INTERVAL_US);
    
    delay(1000);
}

// ==================== 主循环 ====================

void loop() {
    static uint32_t lastSampleTime = 0;
    uint32_t currentTime = micros();
    
    // 定时采样控制（200Hz = 5000us间隔）
    if (currentTime - lastSampleTime >= SAMPLING_INTERVAL_US) {
        lastSampleTime = currentTime;
        
        // 读取所有编码器数据
        readAllEncodersParallel();
        
        // 通过ESP-NOW发送数据
        sendDataViaESPNow();
        
        loopCount++;
        
        #if ENABLE_PERFORMANCE_STATS
        // 每秒输出一次性能统计
        if (millis() - lastStatsTime >= 1000) {
            uint32_t avgReadTime = (loopCount > 0) ? (totalReadTime / loopCount) : 0;
            
            // 由于UART0被占用，这里的输出可能看不到
            // 可以通过ESP-NOW将统计信息发送出去，或使用其他调试方式
            
            // 重置统计
            totalReadTime = 0;
            maxReadTime = 0;
            loopCount = 0;
            lastStatsTime = millis();
        }
        #endif
    }
    
    // 短暂延迟，避免CPU占用过高
    delayMicroseconds(100);
}
