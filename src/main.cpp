/**
 * @file main.cpp
 * @brief Multi-encoder communication - Ultra-Fast Dual-Core Batch Processing Mode
 * @note Optimized for maximum frequency using FreeRTOS dual-core processing,
 * minimized overhead, and aggressive timing optimization.
 * 
 * 优化策略：
 * 1. 双核分离：Core 0 专注数据采集，Core 1 负责输出显示
 * 2. 移除flush()：直接切换到接收模式，节省约40-60us
 * 3. 更激进的超时参数：减少等待时间
 * 4. 内联关键函数：减少函数调用开销
 * 5. 预计算缓冲区：避免动态分配
 * 6. 减少跨核同步开销：使用无锁环形缓冲区
 */

#include <Arduino.h>
#include <ModbusRTU.h>
#include <FastLED.h>
#include <soc/gpio_struct.h>

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
uint8_t request_frames[NUM_ENCODERS][8];

// ================= 双核共享数据（使用原子操作避免互斥锁开销）=================
struct EncoderData {
    uint16_t values[NUM_ENCODERS];
    bool status[NUM_ENCODERS];
    bool all_ok;
    volatile uint32_t seq_number; // 序列号，用于检测新数据
};

// 双缓冲区：避免数据撕裂，无需互斥锁
EncoderData encoder_data_buffer[2];
volatile uint8_t write_buffer_idx = 0;
volatile uint8_t read_buffer_idx = 1;

// ================= 性能监测变量 =================
volatile unsigned long cycle_count = 0;
volatile unsigned long freq_calc_start = 0;
const uint32_t FREQ_REPORT_INTERVAL = 1000;

// ================= 对象实例化 =================
CRGB leds[NUM_LEDS];

// ================= 内联优化：CRC16 计算 =================
inline uint16_t calculateCRC(uint8_t *buf, int len) {
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

// ================= 核心优化：超快速读取函数 =================
// 进一步优化：减少超时检查频率，使用更激进的参数
inline __attribute__((always_inline)) int readBytesFast(uint8_t *buffer, int length, uint32_t response_timeout_us, uint32_t inter_byte_timeout_us) {
    int count = 0;
    unsigned long start = micros();
    
    // 等待第一个字节 - 使用更短的超时
    while (!Serial2.available()) {
        if (micros() - start > response_timeout_us) return 0;
    }
    
    // 读取后续字节 - 批量读取优化
    while (count < length) {
        int available = Serial2.available();
        if (available > 0) {
            int to_read = min(available, length - count);
            for (int i = 0; i < to_read; i++) {
                buffer[count++] = Serial2.read();
            }
            start = micros();
        } else {
            if (micros() - start > inter_byte_timeout_us) break;
        }
    }
    return count;
}

// ================= 核心：批量处理函数（运行在Core 0）=================
void doBatchProcessing() {
    static uint32_t seq = 0;
    bool current_batch_ok = true;
    
    // 获取写缓冲区
    EncoderData* data = &encoder_data_buffer[write_buffer_idx];

    for (int i = 0; i < NUM_ENCODERS; i++) {
        // 优化1: 移除Serial2.flush()，直接切换到接收模式
        // flush()会等待FIFO清空，耗时约40-60us，对高频通讯是巨大开销
        GPIO.out_w1ts = (1UL << RS485_DE_RE_PIN); // 发送模式
        Serial2.write(request_frames[i], 8);
        // 不等待flush，直接切换 - 数据会在后台DMA发送
        delayMicroseconds(32); // 波特率2.5Mbps，8字节约32us（8*8/2.5=25.6us，留余量）
        GPIO.out_w1tc = (1UL << RS485_DE_RE_PIN); // 接收模式

        // 优化2: 更激进的超时参数
        // 7字节 @ 2.5Mbps ≈ 22.4us，响应延迟通常<100us
        uint8_t response[7];
        int len = readBytesFast(response, 7, 300, 80); // 降低超时时间

        if (len == 7 && response[0] == ENCODER_IDS[i] && response[1] == 0x03) {
            data->values[i] = (response[3] << 8) | response[4];
            data->status[i] = true;
        } else {
            data->status[i] = false;
            current_batch_ok = false;
            // 快速清空缓冲区
            for(uint8_t clear_count = 0; Serial2.available() && clear_count < 16; clear_count++) {
                Serial2.read();
            }
        }
    }

    data->all_ok = current_batch_ok;
    data->seq_number = ++seq;
    
    // 原子切换缓冲区
    uint8_t old_write = write_buffer_idx;
    write_buffer_idx = read_buffer_idx;
    read_buffer_idx = old_write;
    
    cycle_count++;
}

// ================= Core 0 任务：专注于数据采集 =================
void taskDataAcquisition(void *parameter) {
    freq_calc_start = millis();
    
    // 最高优先级
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
    
    while (true) {
        doBatchProcessing();
        // 不使用任何delay，全速运行
        // 如果需要控制频率，可以使用 delayMicroseconds(50); 等微秒级延迟
    }
}

// ================= Core 1 任务：负责输出显示 =================
void taskOutputDisplay(void *parameter) {
    unsigned long last_output_time = 0;
    unsigned long last_freq_report_time = 0;
    unsigned long last_led_update_time = 0;
    uint32_t last_seq = 0;
    
    const uint32_t OUTPUT_INTERVAL = 10; // 每10ms输出一次
    const uint32_t LED_UPDATE_INTERVAL = 100; // LED每100ms更新一次（降低频率）
    
    // 预分配输出缓冲区
    char output_buffer[192];
    
    while (true) {
        unsigned long current_time = millis();
        
        // 获取读缓冲区
        EncoderData* data = &encoder_data_buffer[read_buffer_idx];
        uint32_t current_seq = data->seq_number;
        
        // 只在有新数据时才输出
        if (current_seq != last_seq) {
            last_seq = current_seq;
            
            // 输出数据
            if (current_time - last_output_time >= OUTPUT_INTERVAL) {
                last_output_time = current_time;
                
                char* ptr = output_buffer;
                for (int i = 0; i < NUM_ENCODERS; i++) {
                    uint16_t raw = data->values[i];
                    uint32_t angle_x100 = ((uint32_t)raw * 1125) >> 11;
                    
                    // 快速整数转字符串
                    uint32_t int_part = angle_x100 / 100;
                    uint32_t dec_part = angle_x100 % 100;
                    
                    if (int_part == 0) {
                        *ptr++ = '0';
                    } else {
                        char tmp[10];
                        int j = 0;
                        uint32_t n = int_part;
                        while (n > 0) {
                            tmp[j++] = (n % 10) + '0';
                            n /= 10;
                        }
                        while (j > 0) {
                            *ptr++ = tmp[--j];
                        }
                    }
                    
                    *ptr++ = '.';
                    *ptr++ = (dec_part / 10) + '0';
                    *ptr++ = (dec_part % 10) + '0';
                    
                    if (i < NUM_ENCODERS - 1) {
                        *ptr++ = ',';
                    }
                }
                *ptr++ = '\r';
                *ptr++ = '\n';
                
                Serial.write((const uint8_t*)output_buffer, ptr - output_buffer);
            }
            
            // 更新LED（降低频率以减少对Core 1的占用）
            if (current_time - last_led_update_time >= LED_UPDATE_INTERVAL) {
                last_led_update_time = current_time;
                leds[0] = data->all_ok ? CRGB::Green : CRGB::Red;
                FastLED.show();
            }
        }
        
        // 报告频率
        if (current_time - last_freq_report_time >= FREQ_REPORT_INTERVAL) {
            last_freq_report_time = current_time;
            unsigned long elapsed_time = current_time - freq_calc_start;
            if (elapsed_time > 0 && cycle_count > 0) {
                float actual_hz = (float)cycle_count * 1000.0 / elapsed_time;
                Serial.printf("# Update Frequency: %.2f Hz\n", actual_hz);
                cycle_count = 0;
                freq_calc_start = current_time;
            }
        }
        
        // 给其他任务让出一点时间
        vTaskDelay(1); // 1ms延迟
    }
}

// ================= Setup =================
void setup() {
    delay(500);
    Serial.begin(2000000);

    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(RS485_DE_RE_PIN, OUTPUT);
    digitalWrite(RS485_DE_RE_PIN, LOW);

    FastLED.addLeds<WS2812B, WS2812_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(50);
    leds[0] = CRGB::Orange;
    FastLED.show();

    playStartupSound();

    // 预生成所有Modbus请求帧
    for (int i = 0; i < NUM_ENCODERS; i++) {
        request_frames[i][0] = ENCODER_IDS[i];
        request_frames[i][1] = 0x03;
        request_frames[i][2] = 0x00;
        request_frames[i][3] = 0x01;
        request_frames[i][4] = 0x00;
        request_frames[i][5] = 0x01;
        uint16_t crc = calculateCRC(request_frames[i], 6);
        request_frames[i][6] = crc & 0xFF;
        request_frames[i][7] = (crc >> 8) & 0xFF;
    }

    // 初始化RS485串口 - 保持2.5Mbps
    Serial2.begin(2500000, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    Serial2.setRxBufferSize(256); // 适当增加缓冲区
    Serial2.setTxBufferSize(256);

    // 初始化双缓冲区
    for (int i = 0; i < 2; i++) {
        encoder_data_buffer[i].all_ok = false;
        encoder_data_buffer[i].seq_number = 0;
        for (int j = 0; j < NUM_ENCODERS; j++) {
            encoder_data_buffer[i].values[j] = 0;
            encoder_data_buffer[i].status[j] = false;
        }
    }

    leds[0] = CRGB::Blue;
    FastLED.show();
    Serial.println("# System Ready - Ultra-Fast Dual-Core Mode (2.5Mbps)");
    Serial.println("# Optimizations: No flush() + Aggressive timeouts + Dual-core separation");

    // 创建双核任务
    // Core 0: 数据采集（最高优先级）
    xTaskCreatePinnedToCore(
        taskDataAcquisition,
        "DataAcq",
        4096,
        NULL,
        configMAX_PRIORITIES - 1, // 最高优先级
        NULL,
        0 // Core 0
    );

    // Core 1: 输出显示（较低优先级）
    xTaskCreatePinnedToCore(
        taskOutputDisplay,
        "Output",
        4096,
        NULL,
        1, // 较低优先级
        NULL,
        1 // Core 1
    );
}

void loop() {
    // 主循环空置，所有工作由FreeRTOS任务完成
    vTaskDelay(portMAX_DELAY);
}
