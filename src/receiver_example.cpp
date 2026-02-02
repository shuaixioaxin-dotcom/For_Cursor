/**
 * ESP-NOW 接收端示例代码
 * 
 * 用于接收编码器数据并通过串口输出
 * 可以连接到PC进行数据分析
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "config.h"

// ==================== 全局变量 ====================

EncoderDataPacket receivedPacket;
uint32_t packetCount = 0;
uint32_t errorCount = 0;
uint32_t lastPacketTime = 0;

// ==================== ESP-NOW 接收回调 ====================

void onDataReceive(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    if (data_len != sizeof(EncoderDataPacket)) {
        Serial.printf("错误：数据长度不匹配 (%d vs %d)\n", data_len, sizeof(EncoderDataPacket));
        errorCount++;
        return;
    }
    
    // 复制数据
    memcpy(&receivedPacket, data, sizeof(EncoderDataPacket));
    
    // 校验和验证
    uint8_t checksum = 0;
    uint8_t *data_ptr = (uint8_t*)&receivedPacket;
    for (size_t i = 0; i < sizeof(EncoderDataPacket) - 1; i++) {
        checksum ^= data_ptr[i];
    }
    
    if (checksum != receivedPacket.checksum) {
        Serial.println("错误：校验和不匹配");
        errorCount++;
        return;
    }
    
    packetCount++;
    
    // 计算实际接收频率
    uint32_t currentTime = millis();
    uint32_t interval = currentTime - lastPacketTime;
    lastPacketTime = currentTime;
    
    // 输出数据（每10个包输出一次详细信息）
    if (packetCount % 10 == 0) {
        Serial.println("\n========================================");
        Serial.printf("数据包 #%u (序列号: %u)\n", packetCount, receivedPacket.sequence);
        Serial.printf("时间戳: %lu us\n", receivedPacket.timestamp);
        Serial.printf("采样率: %u Hz\n", receivedPacket.sample_rate);
        Serial.printf("接收间隔: %lu ms (%.1f Hz)\n", interval, 1000.0 / interval);
        Serial.printf("有效标志: 0x%02X\n", receivedPacket.valid_flags);
        
        // 输出每个编码器的前4个字节（示例）
        for (int i = 0; i < ENCODER_COUNT; i++) {
            if (receivedPacket.valid_flags & (1 << i)) {
                Serial.printf("编码器 %d: ", i);
                for (int j = 0; j < 4; j++) {  // 只显示前4字节
                    Serial.printf("%02X ", receivedPacket.encoder_data[i][j]);
                }
                Serial.println("...");
            } else {
                Serial.printf("编码器 %d: 无效数据\n", i);
            }
        }
        
        Serial.printf("错误计数: %lu\n", errorCount);
        Serial.println("========================================\n");
    } else {
        // 简化输出
        Serial.printf(".");
        if (packetCount % 50 == 0) {
            Serial.println();
        }
    }
}

// ==================== Setup ====================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n========================================");
    Serial.println("ESP-NOW 接收端");
    Serial.println("========================================");
    
    // 设置为Station模式
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    // 显示本机MAC地址（需要配置到发送端）
    Serial.printf("本机MAC地址: %s\n", WiFi.macAddress().c_str());
    Serial.println("请将此MAC地址配置到发送端的 config.h 中");
    Serial.println("========================================\n");
    
    // 初始化ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW 初始化失败");
        return;
    }
    
    // 注册接收回调
    esp_now_register_recv_cb(onDataReceive);
    
    Serial.println("ESP-NOW 初始化成功，等待数据...\n");
}

// ==================== Loop ====================

void loop() {
    // 检测超时（3秒无数据）
    static uint32_t lastCheckTime = 0;
    if (millis() - lastCheckTime > 3000) {
        lastCheckTime = millis();
        
        if (millis() - lastPacketTime > 3000 && packetCount > 0) {
            Serial.println("\n警告：超过3秒未接收到数据");
        }
    }
    
    delay(100);
}
