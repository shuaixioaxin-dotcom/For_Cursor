/**
 * @file main.cpp
 * @brief 14路编码器数据采集与无线传输系统
 * @platform ESP32-S3
 * @framework Arduino
 * 
 * 功能说明：
 * 1. 通过SPI总线读取14个编码器数据
 * 2. 数据简单处理后暂存至Flash/RAM
 * 3. 按键触发，通过WiFi发送至Linux设备
 */

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <Preferences.h>  // ESP32 NVS Flash存储
#include <ArduinoJson.h>

// ============== 配置参数 ==============
#define ENCODER_COUNT       14      // 编码器数量
#define SAMPLE_BUFFER_SIZE  100     // 缓存采样次数
#define SPI_CLOCK_SPEED     1000000 // SPI时钟 1MHz

// WiFi配置
const char* WIFI_SSID     = "YourWiFiSSID";
const char* WIFI_PASSWORD = "YourWiFiPassword";

// Linux服务器配置 (TCP Socket方式)
const char* SERVER_IP     = "192.168.1.100";
const uint16_t SERVER_PORT = 8888;

// ============== GPIO定义 ==============
// SPI引脚
#define SPI_MOSI    11
#define SPI_MISO    13
#define SPI_SCLK    12

// 14个编码器CS片选引脚
const uint8_t CS_PINS[ENCODER_COUNT] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,  // CS0-CS9
    14, 15, 16, 17                   // CS10-CS13
};

// 功能引脚
#define BUTTON_PIN  0   // 发送按键 (ESP32-S3 BOOT键)
#define LED_PIN     48  // 状态LED

// ============== 数据结构 ==============
typedef struct {
    uint32_t timestamp;                     // 时间戳(ms)
    int32_t  encoder_values[ENCODER_COUNT]; // 编码器原始值
    float    processed_values[ENCODER_COUNT]; // 处理后的值(角度/位置)
} EncoderData_t;

// ============== 全局变量 ==============
SPIClass* encoderSPI = nullptr;
Preferences preferences;  // NVS Flash存储
WiFiClient client;

// 数据缓存 (存储在RAM中，可改用PSRAM)
EncoderData_t dataBuffer[SAMPLE_BUFFER_SIZE];
volatile uint16_t bufferIndex = 0;
volatile bool sendRequested = false;

// ============== 函数声明 ==============
void initGPIO();
void initSPI();
void initWiFi();
void initFlashStorage();
int32_t readEncoder(uint8_t encoderIndex);
void readAllEncoders(EncoderData_t* data);
void processEncoderData(EncoderData_t* data);
void saveToFlash(EncoderData_t* data);
void sendDataToServer();
void buttonISR();
void blinkLED(int times, int delayMs);

// ============== 初始化函数 ==============

void initGPIO() {
    // 初始化CS引脚
    for (int i = 0; i < ENCODER_COUNT; i++) {
        pinMode(CS_PINS[i], OUTPUT);
        digitalWrite(CS_PINS[i], HIGH);  // CS默认高电平(未选中)
    }
    
    // 初始化按键 (带中断)
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
    
    // 初始化LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    Serial.println("[GPIO] 初始化完成");
}

void initSPI() {
    // 使用SPI2 (HSPI)
    encoderSPI = new SPIClass(HSPI);
    encoderSPI->begin(SPI_SCLK, SPI_MISO, SPI_MOSI, -1);
    
    Serial.println("[SPI] 初始化完成");
}

void initWiFi() {
    Serial.print("[WiFi] 连接中: ");
    Serial.println(WIFI_SSID);
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 30) {
        delay(500);
        Serial.print(".");
        retries++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.print("[WiFi] 已连接, IP: ");
        Serial.println(WiFi.localIP());
        blinkLED(3, 100);
    } else {
        Serial.println();
        Serial.println("[WiFi] 连接失败!");
        blinkLED(10, 50);
    }
}

void initFlashStorage() {
    // 使用NVS存储配置和少量数据
    preferences.begin("encoder_data", false);
    
    // 读取上次保存的缓冲区索引
    bufferIndex = preferences.getUShort("buf_idx", 0);
    
    Serial.printf("[Flash] 初始化完成, 缓冲区索引: %d\n", bufferIndex);
}

// ============== 编码器读取函数 ==============

/**
 * @brief 读取单个编码器数据
 * @param encoderIndex 编码器索引 (0-13)
 * @return 编码器原始值
 * 
 * 注意: 此函数需要根据实际编码器型号修改SPI时序
 * 示例为 AS5048A 14-bit磁编码器
 */
int32_t readEncoder(uint8_t encoderIndex) {
    if (encoderIndex >= ENCODER_COUNT) return -1;
    
    uint8_t cs = CS_PINS[encoderIndex];
    
    // SPI设置 (根据编码器数据手册调整)
    encoderSPI->beginTransaction(SPISettings(SPI_CLOCK_SPEED, MSBFIRST, SPI_MODE1));
    
    digitalWrite(cs, LOW);  // 选中编码器
    delayMicroseconds(1);
    
    // 读取数据 (AS5048A示例: 发送读取命令，接收16-bit数据)
    uint16_t command = 0xFFFF;  // 读取角度命令
    uint16_t response = encoderSPI->transfer16(command);
    
    digitalWrite(cs, HIGH);  // 取消选中
    
    encoderSPI->endTransaction();
    
    // 解析数据 (AS5048A: 14-bit角度值)
    int32_t value = response & 0x3FFF;  // 提取低14位
    
    return value;
}

/**
 * @brief 读取所有编码器数据
 */
void readAllEncoders(EncoderData_t* data) {
    data->timestamp = millis();
    
    for (int i = 0; i < ENCODER_COUNT; i++) {
        data->encoder_values[i] = readEncoder(i);
    }
}

/**
 * @brief 数据处理 (滤波、单位转换等)
 */
void processEncoderData(EncoderData_t* data) {
    for (int i = 0; i < ENCODER_COUNT; i++) {
        // 示例: 将14-bit值转换为角度 (0-360度)
        // AS5048A: 16384 counts = 360度
        float angle = (float)data->encoder_values[i] * 360.0f / 16384.0f;
        
        // 简单滤波 (可以添加更复杂的滤波算法)
        static float lastValues[ENCODER_COUNT] = {0};
        float alpha = 0.8f;  // 滤波系数
        angle = alpha * angle + (1 - alpha) * lastValues[i];
        lastValues[i] = angle;
        
        data->processed_values[i] = angle;
    }
}

/**
 * @brief 保存数据到Flash (NVS)
 * 注意: NVS适合少量数据，大量数据建议使用SPIFFS/LittleFS
 */
void saveToFlash(EncoderData_t* data) {
    // 将数据存入RAM缓冲区
    if (bufferIndex < SAMPLE_BUFFER_SIZE) {
        memcpy(&dataBuffer[bufferIndex], data, sizeof(EncoderData_t));
        bufferIndex++;
        
        // 保存索引到NVS
        preferences.putUShort("buf_idx", bufferIndex);
    } else {
        Serial.println("[存储] 缓冲区已满!");
    }
}

// ============== 数据发送函数 ==============

/**
 * @brief 通过WiFi发送数据到Linux服务器
 */
void sendDataToServer() {
    if (bufferIndex == 0) {
        Serial.println("[发送] 缓冲区为空，无数据发送");
        return;
    }
    
    Serial.printf("[发送] 准备发送 %d 条数据...\n", bufferIndex);
    
    // 连接服务器
    if (!client.connect(SERVER_IP, SERVER_PORT)) {
        Serial.println("[发送] 服务器连接失败!");
        blinkLED(5, 100);
        return;
    }
    
    digitalWrite(LED_PIN, HIGH);
    
    // 构建JSON数据
    StaticJsonDocument<8192> doc;
    doc["device_id"] = WiFi.macAddress();
    doc["sample_count"] = bufferIndex;
    
    JsonArray samples = doc.createNestedArray("samples");
    
    for (int i = 0; i < bufferIndex; i++) {
        JsonObject sample = samples.createNestedObject();
        sample["ts"] = dataBuffer[i].timestamp;
        
        JsonArray values = sample.createNestedArray("values");
        for (int j = 0; j < ENCODER_COUNT; j++) {
            values.add(dataBuffer[i].processed_values[j]);
        }
    }
    
    // 发送数据
    String jsonStr;
    serializeJson(doc, jsonStr);
    
    client.println(jsonStr);
    client.flush();
    
    // 等待服务器响应
    unsigned long timeout = millis() + 5000;
    while (client.connected() && millis() < timeout) {
        if (client.available()) {
            String response = client.readStringUntil('\n');
            Serial.printf("[发送] 服务器响应: %s\n", response.c_str());
            break;
        }
        delay(10);
    }
    
    client.stop();
    
    // 清空缓冲区
    bufferIndex = 0;
    preferences.putUShort("buf_idx", 0);
    
    digitalWrite(LED_PIN, LOW);
    blinkLED(2, 200);
    
    Serial.println("[发送] 完成!");
}

// ============== 中断服务函数 ==============

void IRAM_ATTR buttonISR() {
    static unsigned long lastPress = 0;
    unsigned long now = millis();
    
    // 消抖
    if (now - lastPress > 200) {
        sendRequested = true;
        lastPress = now;
    }
}

// ============== 辅助函数 ==============

void blinkLED(int times, int delayMs) {
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(delayMs);
        digitalWrite(LED_PIN, LOW);
        delay(delayMs);
    }
}

// ============== 主程序 ==============

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n========================================");
    Serial.println("  14路编码器数据采集与传输系统");
    Serial.println("  Platform: ESP32-S3");
    Serial.println("========================================\n");
    
    initGPIO();
    initSPI();
    initFlashStorage();
    initWiFi();
    
    Serial.println("\n[系统] 初始化完成，开始数据采集...");
    Serial.println("[系统] 按下BOOT键发送数据到服务器\n");
}

void loop() {
    static unsigned long lastSampleTime = 0;
    const unsigned long sampleInterval = 100;  // 采样间隔100ms (10Hz)
    
    // 定时采集数据
    if (millis() - lastSampleTime >= sampleInterval) {
        lastSampleTime = millis();
        
        EncoderData_t newData;
        readAllEncoders(&newData);
        processEncoderData(&newData);
        saveToFlash(&newData);
        
        // 打印调试信息 (可选)
        if (bufferIndex % 10 == 0) {
            Serial.printf("[采集] #%d, 编码器0: %.2f°, 编码器13: %.2f°\n",
                          bufferIndex,
                          newData.processed_values[0],
                          newData.processed_values[13]);
        }
    }
    
    // 检查发送请求
    if (sendRequested) {
        sendRequested = false;
        sendDataToServer();
    }
    
    // 检查WiFi连接状态
    static unsigned long lastWiFiCheck = 0;
    if (millis() - lastWiFiCheck > 30000) {
        lastWiFiCheck = millis();
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("[WiFi] 连接断开，重新连接...");
            WiFi.reconnect();
        }
    }
}
