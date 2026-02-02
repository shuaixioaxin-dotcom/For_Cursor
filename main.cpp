/**
 * @file main.cpp
 * @brief Multi-encoder communication - Fast Batch Processing Mode
 * @note Optimized for high frequency by bypassing library overhead and
 * implementing a tight send-receive loop for all encoders.
 */

#include <Arduino.h>
#include <ModbusRTU.h>
#include <FastLED.h>
#include <soc/gpio_struct.h> // 用于GPIO寄存器直接操作

// ================= 引脚配置 =================
#define RS485_RX_PIN 32
#define RS485_TX_PIN 33
#define RS485_DE_RE_PIN 25

#define WS2812_PIN 26
#define NUM_LEDS 1
#define BUZZER_PIN 2

// ================= 编码器配置 =================
#define NUM_ENCODERS 6
const uint8_t ENCODER_IDS[NUM_ENCODERS] = {1, 2, 3, 4, 5, 6};

// ================= 批量通讯参数 =================
// 每个读 1 个寄存器的请求帧长度为 8 字节，响应帧长度为 7 字节
uint8_t request_frames[NUM_ENCODERS][8];

// ================= 数据监测变量 =================
unsigned long cycle_count = 0;
unsigned long freq_calc_start = 0;
const uint32_t FREQ_REPORT_INTERVAL = 1000;
float actual_hz = 0;

// ================= 对象实例化 =================
ModbusRTU mb;
CRGB leds[NUM_LEDS];

// 使用 uint16_t 存储原始值以提高效率
uint16_t encoder_raw_values[NUM_ENCODERS];
bool encoder_status[NUM_ENCODERS];
// 全局状态标志
bool all_encoders_ok = false;

unsigned long last_request_time = 0;
unsigned long last_output_time = 0;
unsigned long last_freq_report_time = 0;
unsigned long last_led_update_time = 0;
const uint32_t LED_UPDATE_INTERVAL = 50; // LED每50ms更新一次

// ================= CRC16 计算辅助函数 =================
uint16_t calculateCRC(uint8_t *buf, int len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else crc >>= 1;
        }
    }
    return crc;
}

// ================= 蜂鸣器功能 =================
void playStartupSound() {
    int melody[] = {1000, 1500, 2000};
    for (int i = 0; i < 3; i++) {
        tone(BUZZER_PIN, melody[i]);
        delay(100);
        noTone(BUZZER_PIN);
        delay(50);
    }
}

// ================= 核心：批量处理函数 =================

// 优化：微秒级超时读取函数
// response_timeout_us: 等待第一个字节的超时时间
// inter_byte_timeout_us: 字节间超时时间
int readBytesFast(uint8_t *buffer, int length, uint32_t response_timeout_us, uint32_t inter_byte_timeout_us) {
    int count = 0;
    unsigned long start = micros();
    
    // 1. 等待第一个字节
    while (!Serial2.available()) {
        if (micros() - start > response_timeout_us) return 0;
    }
    
    // 2. 读取后续字节
    while (count < length) {
        if (Serial2.available()) {
            buffer[count++] = Serial2.read();
            start = micros(); // 重置字节间超时计时
        } else {
            if (micros() - start > inter_byte_timeout_us) break;
        }
    }
    return count;
}

/**
 * 按照顺序快速完成一轮所有编码器的读取
 * 去掉了异步等待，通过阻塞式读取确保最高频率且不碰撞
 */
void doBatchProcessing() {
    bool current_batch_ok = true;

    for (int i = 0; i < NUM_ENCODERS; i++) {
        // 1. 发送请求 - 使用GPIO寄存器直接操作
        GPIO.out_w1ts = (1UL << RS485_DE_RE_PIN); // HIGH
        Serial2.write(request_frames[i], 8);
        Serial2.flush(); // 确保数据完全发出
        GPIO.out_w1tc = (1UL << RS485_DE_RE_PIN); // LOW

        // 2. 等待并读取响应 (Modbus RTU 1寄存器响应为 7 字节)
        uint8_t response[7];
        // 优化：使用自定义微秒级读取
        // 波特率 2.5Mbps -> 1字节约 4us
        // 响应超时设为 800us，字节间超时设为 100us
        int len = readBytesFast(response, 7, 800, 100);

        if (len == 7 && response[0] == ENCODER_IDS[i] && response[1] == 0x03) {
            // 优化：仅存储原始值，计算推迟到输出阶段
            encoder_raw_values[i] = (response[3] << 8) | response[4];
            encoder_status[i] = true;
        } else {
            encoder_status[i] = false;
            current_batch_ok = false;
            // 优化：快速清空缓冲区
            uint8_t clear_count = 0;
            while(Serial2.available() && clear_count++ < 16) Serial2.read();
        }
    }

    all_encoders_ok = current_batch_ok;
    cycle_count++;
    if (freq_calc_start == 0) freq_calc_start = millis();
}

// 辅助函数：快速将数值转为 "XXX.YY" 格式并追加到 buffer
// val 应该是 (angle * 100) 的整数形式
char* fast_angle_to_str(char* buf, uint32_t val) {
    uint32_t int_part = val / 100;
    uint32_t dec_part = val % 100;
    
    // 整数部分
    if (int_part == 0) {
        *buf++ = '0';
    } else {
        char tmp[10];
        int i = 0;
        while (int_part > 0) {
            tmp[i++] = (int_part % 10) + '0';
            int_part /= 10;
        }
        while (i > 0) {
            *buf++ = tmp[--i];
        }
    }
    
    *buf++ = '.';
    
    // 小数部分 (固定2位)
    *buf++ = (dec_part / 10) + '0';
    *buf++ = (dec_part % 10) + '0';
    
    return buf;
}

// 输出 CSV 格式数据
void outputSimpleCSV() {
    char buffer[128]; // 6个编码器数据足够小
    char* ptr = buffer;
    
    for (int i = 0; i < NUM_ENCODERS; i++) {
        uint16_t raw = encoder_raw_values[i];
        
        // 使用整数运算计算角度，避免浮点
        // Angle = raw * 360.0 / 65536.0
        // Target = Angle * 100 = raw * 36000 / 65536
        // 36000 / 65536 = 1125 / 2048 (约分)
        // 2048 是 2^11，可以用位移代替除法
        uint32_t angle_x100 = ((uint32_t)raw * 1125) >> 11;
        
        ptr = fast_angle_to_str(ptr, angle_x100);
        
        if (i < NUM_ENCODERS - 1) {
            *ptr++ = ',';
        }
    }
    *ptr++ = '\r';
    *ptr++ = '\n';
    
    Serial.write((const uint8_t*)buffer, ptr - buffer);
}

// 报告当前频率
void reportFrequency() {
    if (freq_calc_start > 0 && cycle_count > 0) {
        unsigned long elapsed_time = millis() - freq_calc_start;
        actual_hz = (float)cycle_count * 1000.0 / elapsed_time;
        Serial.printf("# Update Frequency: %.2f Hz\n", actual_hz);
        cycle_count = 0;
        freq_calc_start = millis();
    }
}

void setup() {
    delay(500);
    // 提升调试串口波特率到 2000000
    Serial.begin(2000000); 

    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(RS485_DE_RE_PIN, OUTPUT);
    digitalWrite(RS485_DE_RE_PIN, LOW);

    FastLED.addLeds<WS2812B, WS2812_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(50);
    leds[0] = CRGB::Orange;
    FastLED.show();

    playStartupSound();

    // 预生成每个编码器的 Modbus 请求帧
    for (int i = 0; i < NUM_ENCODERS; i++) {
        request_frames[i][0] = ENCODER_IDS[i];
        request_frames[i][1] = 0x03;
        request_frames[i][2] = 0x00;
        request_frames[i][3] = 0x01; // 地址 1
        request_frames[i][4] = 0x00;
        request_frames[i][5] = 0x01; // 读取 1 个寄存器
        uint16_t crc = calculateCRC(request_frames[i], 6);
        request_frames[i][6] = crc & 0xFF;
        request_frames[i][7] = (crc >> 8) & 0xFF;
    }

    // 初始化 RS485 串口，波特率提升至 2500000
    Serial2.begin(2500000, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    Serial2.setRxBufferSize(256); // 稍微增加缓冲区，虽然主要靠实时读取

    leds[0] = CRGB::Blue;
    FastLED.show();
    Serial.println("# System Ready - High Freq Batch Mode (2.5Mbps)");
    last_freq_report_time = millis();
}

void loop() {
    // 1. 执行一轮批量采集
    doBatchProcessing();

    // 2. 定时更新LED状态（使用全局标志位）
    unsigned long current_time = millis();
    if (current_time - last_led_update_time >= LED_UPDATE_INTERVAL) {
        last_led_update_time = current_time;
        leds[0] = all_encoders_ok ? CRGB::Green : CRGB::Red;
        FastLED.show();
    }

    // 3. 定时输出数据 (10ms 输出一次，避免串口阻塞影响采集)
    if (current_time - last_output_time >= 10) {
        last_output_time = current_time;
        outputSimpleCSV();
    }

    // 4. 定时报告实际刷新频率
    if (current_time - last_freq_report_time >= FREQ_REPORT_INTERVAL) {
        last_freq_report_time = current_time;
        // reportFrequency(); // 用户要求注释掉
    }
}
