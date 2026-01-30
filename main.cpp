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
#define NUM_ENCODERS 16
const uint8_t ENCODER_IDS[NUM_ENCODERS] = {1, 2, 3,4, 5,6,7 ,8 ,9 ,10 ,11 ,12, 13, 14 ,15 ,16};


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

float encoder_angles[NUM_ENCODERS];
bool encoder_status[NUM_ENCODERS];

unsigned long last_request_time = 0;
unsigned long last_output_time = 0;
unsigned long last_freq_report_time = 0;
unsigned long last_led_update_time = 0;
const uint32_t LED_UPDATE_INTERVAL = 50; // LED每50ms更新一次，减少开销

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
// response_timeout_us: 等待第一个字节的超时时间 (建议 800us - 1500us)
// inter_byte_timeout_us: 字节间超时时间 (建议 100us - 200us)
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
    for (int i = 0; i < NUM_ENCODERS; i++) {
        // 1. 发送请求 - 使用GPIO寄存器直接操作，比digitalWrite快约2-3倍
        // RS485_DE_RE_PIN=25 < 32，可以直接使用GPIO.out_w1ts/out_w1tc
        GPIO.out_w1ts = (1UL << RS485_DE_RE_PIN); // 设置为HIGH
        Serial2.write(request_frames[i], 8);
        Serial2.flush(); // 确保数据完全发出
        GPIO.out_w1tc = (1UL << RS485_DE_RE_PIN); // 设置为LOW，立即切换到接收模式

        // 2. 等待并读取响应 (Modbus RTU 1寄存器响应为 7 字节)
        uint8_t response[7];
        // 优化：使用自定义微秒级读取
        // 响应超时设为 1200us (1.2ms)，字节间超时设为 150us
        // 2Mbps波特率下，1字节传输约5us。从机响应时间通常在100us-1ms之间。
        int len = readBytesFast(response, 7, 1200, 150);

        if (len == 7 && response[0] == ENCODER_IDS[i] && response[1] == 0x03) {
            // 优化：减少浮点运算，使用整数运算
            uint16_t raw = (response[3] << 8) | response[4];
            // 360.0/65536.0 ≈ 0.0054931640625，预先计算
            encoder_angles[i] = raw * 0.0054931640625f;
            encoder_status[i] = true;
        } else {
            encoder_status[i] = false;
            // 优化：限制清空次数，避免长时间阻塞
            uint8_t clear_count = 0;
            while(Serial2.available() && clear_count++ < 16) Serial2.read();
        }
    }

    cycle_count++;
    if (freq_calc_start == 0) freq_calc_start = millis();
}

// 输出 CSV 格式数据
// 优化：使用缓冲区一次性输出，减少 Serial.print 调用开销
void outputSimpleCSV() {
    char buffer[256]; // 足够容纳 16 * 10 字符
    int pos = 0;
    
    for (int i = 0; i < NUM_ENCODERS; i++) {
        // 手动格式化比 sprintf 快，但为了代码简洁这里先用 sprintf
        pos += sprintf(buffer + pos, "%.2f", encoder_angles[i]);
        if (i < NUM_ENCODERS - 1) {
            buffer[pos++] = ',';
        }
    }
    buffer[pos++] = '\r';
    buffer[pos++] = '\n';
    
    Serial.write((const uint8_t*)buffer, pos);
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
    // 提升调试串口波特率到 2000000，减少打印阻塞时间
    Serial.begin(2000000); 

    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(RS485_DE_RE_PIN, OUTPUT);
    digitalWrite(RS485_DE_RE_PIN, LOW);

    FastLED.addLeds<WS2812B, WS2812_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(50);
    leds[0] = CRGB::Orange;
    FastLED.show();

    playStartupSound();

    // 预生成每个编码器的 Modbus 请求帧 [ID] 03 00 01 00 01 [CRC_L] [CRC_H]
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

    // 初始化串口 2 (2000000)
    Serial2.begin(2000000, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    // 优化：减少超时时间
    Serial2.setTimeout(2);

    leds[0] = CRGB::Blue;
    FastLED.show();
    Serial.println("# System Ready - Fast Batch Mode Enabled (Optimized)");
    last_freq_report_time = millis();
}

void loop() {
    // 1. 执行一轮批量采集
    doBatchProcessing();

    // 2. 定时更新LED状态（降低更新频率以减少开销）
    unsigned long current_time = millis();
    if (current_time - last_led_update_time >= LED_UPDATE_INTERVAL) {
        last_led_update_time = current_time;
        bool all_ok = true;
        for (int i = 0; i < NUM_ENCODERS; i++) {
            if (!encoder_status[i]) {
                all_ok = false;
                break; // 找到失败就退出
            }
        }
        leds[0] = all_ok ? CRGB::Green : CRGB::Red;
        FastLED.show();
    }

    // 3. 定时输出数据 (为了不影响频率，仅在 10ms 间隔后输出一次)
    if (current_time - last_output_time >= 10) {
        last_output_time = current_time;
        outputSimpleCSV();
    }

    // 4. 定时报告实际刷新频率
    if (current_time - last_freq_report_time >= FREQ_REPORT_INTERVAL) {
        last_freq_report_time = current_time;
        reportFrequency();
    }
}
