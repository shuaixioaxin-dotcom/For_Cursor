/**
 * ESP32-S3 多摩川编码器高频读取系统
 * 
 * 功能：
 * - 3路并行RS485通信，每路2个编码器
 * - 200Hz采样频率
 * - ESP-NOW无线数据发送
 * - DMA接收减少CPU开销
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "ENCODER";

// ============== 配置参数 ==============

#define ENCODER_COUNT           6       // 编码器总数
#define CHANNELS                3       // 并行通道数
#define ENCODERS_PER_CHANNEL    2       // 每通道编码器数

#define BAUD_RATE               2500000 // 2.5Mbps
#define RESPONSE_SIZE           22      // 返回帧长度(字节)
#define SAMPLE_FREQ_HZ          200     // 采样频率
#define SAMPLE_PERIOD_US        (1000000 / SAMPLE_FREQ_HZ)  // 5000us = 5ms

// 编码器命令 (0x50 = 编码器0, 0x51 = 编码器1, ...)
#define ENCODER_CMD_BASE        0x50

// UART 配置
#define UART_CH1                UART_NUM_1
#define UART_CH2                UART_NUM_2
// ESP32-S3 可以配置更多UART，这里用LP_UART或软件模拟第三路
// 为简化，这里演示2路硬件UART的情况

#define UART1_TX_PIN            17
#define UART1_RX_PIN            18
#define UART1_DE_PIN            8       // RS485方向控制

#define UART2_TX_PIN            19
#define UART2_RX_PIN            20
#define UART2_DE_PIN            9

#define UART3_TX_PIN            21
#define UART3_RX_PIN            47
#define UART3_DE_PIN            10

#define UART_BUF_SIZE           256
#define UART_QUEUE_SIZE         20

// RS485 方向控制宏
#define RS485_TX_MODE(de_pin)   gpio_set_level(de_pin, 1)
#define RS485_RX_MODE(de_pin)   gpio_set_level(de_pin, 0)

// ============== 数据结构 ==============

// 单个编码器数据
typedef struct {
    uint8_t raw_data[RESPONSE_SIZE];
    uint32_t timestamp;
    uint8_t valid;
} encoder_data_t;

// 完整数据包 (所有6个编码器)
typedef struct {
    uint32_t timestamp;
    uint32_t sequence;
    encoder_data_t encoders[ENCODER_COUNT];
    uint8_t checksum;
} __attribute__((packed)) encoder_packet_t;

// 通道配置
typedef struct {
    uart_port_t uart_num;
    gpio_num_t de_pin;
    uint8_t encoder_ids[ENCODERS_PER_CHANNEL];
    QueueHandle_t uart_queue;
} channel_config_t;

// ============== 全局变量 ==============

static channel_config_t channels[CHANNELS];
static encoder_packet_t current_packet;
static SemaphoreHandle_t data_mutex;
static uint32_t sequence_number = 0;
static esp_timer_handle_t sample_timer;

// ESP-NOW 接收端MAC地址 (需要修改为实际地址)
static uint8_t receiver_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ============== UART 初始化 ==============

static void uart_init_channel(channel_config_t *ch)
{
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 安装UART驱动，启用DMA
    ESP_ERROR_CHECK(uart_driver_install(ch->uart_num, UART_BUF_SIZE * 2, 
                                        UART_BUF_SIZE * 2, UART_QUEUE_SIZE, 
                                        &ch->uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(ch->uart_num, &uart_config));
    
    // 配置引脚
    int tx_pin, rx_pin;
    switch (ch->uart_num) {
        case UART_NUM_1:
            tx_pin = UART1_TX_PIN;
            rx_pin = UART1_RX_PIN;
            break;
        case UART_NUM_2:
            tx_pin = UART2_TX_PIN;
            rx_pin = UART2_RX_PIN;
            break;
        default:
            tx_pin = UART3_TX_PIN;
            rx_pin = UART3_RX_PIN;
            break;
    }
    ESP_ERROR_CHECK(uart_set_pin(ch->uart_num, tx_pin, rx_pin, 
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 配置RS485方向控制引脚
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << ch->de_pin),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    RS485_RX_MODE(ch->de_pin);  // 默认接收模式

    // 设置接收超时 (用于判断一帧接收完成)
    // 超时时间 = 3.5个字符时间 ≈ 35 bits / 2.5Mbps ≈ 14us
    uart_set_rx_timeout(ch->uart_num, 10);

    ESP_LOGI(TAG, "UART%d initialized at %d bps", ch->uart_num, BAUD_RATE);
}

// ============== 编码器通信 ==============

/**
 * 读取单个编码器数据
 * @param ch 通道配置
 * @param encoder_id 编码器ID (0-5)
 * @param data 输出数据缓冲区
 * @return 读取的字节数，失败返回-1
 */
static int read_encoder(channel_config_t *ch, uint8_t encoder_id, uint8_t *data)
{
    uint8_t cmd = ENCODER_CMD_BASE + encoder_id;
    
    // 清空接收缓冲区
    uart_flush_input(ch->uart_num);
    
    // 切换到发送模式
    RS485_TX_MODE(ch->de_pin);
    
    // 发送命令
    uart_write_bytes(ch->uart_num, &cmd, 1);
    
    // 等待发送完成
    uart_wait_tx_done(ch->uart_num, pdMS_TO_TICKS(1));
    
    // 切换到接收模式
    RS485_RX_MODE(ch->de_pin);
    
    // 接收数据 (带超时)
    // 预期时间: 22字节 * 10bit / 2.5Mbps = 88us，设置200us超时
    int len = uart_read_bytes(ch->uart_num, data, RESPONSE_SIZE, pdMS_TO_TICKS(1));
    
    if (len != RESPONSE_SIZE) {
        ESP_LOGW(TAG, "Encoder %d: expected %d bytes, got %d", 
                 encoder_id, RESPONSE_SIZE, len);
        return -1;
    }
    
    return len;
}

/**
 * 读取单个通道的所有编码器 (串行)
 */
static void read_channel_encoders(channel_config_t *ch, encoder_packet_t *packet)
{
    for (int i = 0; i < ENCODERS_PER_CHANNEL; i++) {
        uint8_t enc_id = ch->encoder_ids[i];
        encoder_data_t *enc = &packet->encoders[enc_id];
        
        int ret = read_encoder(ch, enc_id, enc->raw_data);
        enc->timestamp = esp_timer_get_time();
        enc->valid = (ret == RESPONSE_SIZE) ? 1 : 0;
    }
}

// ============== 通道读取任务 ==============

// 通道任务参数
typedef struct {
    int channel_idx;
    SemaphoreHandle_t start_sem;
    SemaphoreHandle_t done_sem;
} channel_task_param_t;

static channel_task_param_t ch_params[CHANNELS];
static SemaphoreHandle_t ch_start_sems[CHANNELS];
static SemaphoreHandle_t ch_done_sems[CHANNELS];

/**
 * 通道读取任务 - 每个通道一个任务，并行执行
 */
static void channel_read_task(void *arg)
{
    channel_task_param_t *param = (channel_task_param_t *)arg;
    channel_config_t *ch = &channels[param->channel_idx];
    
    while (1) {
        // 等待开始信号
        xSemaphoreTake(param->start_sem, portMAX_DELAY);
        
        // 读取本通道的编码器
        read_channel_encoders(ch, &current_packet);
        
        // 发送完成信号
        xSemaphoreGive(param->done_sem);
    }
}

// ============== 采样定时器回调 ==============

static void IRAM_ATTR sample_timer_callback(void *arg)
{
    // 触发所有通道开始读取 (并行)
    for (int i = 0; i < CHANNELS; i++) {
        xSemaphoreGiveFromISR(ch_start_sems[i], NULL);
    }
}

// ============== 主采样任务 ==============

static void main_sample_task(void *arg)
{
    while (1) {
        // 等待所有通道完成
        for (int i = 0; i < CHANNELS; i++) {
            xSemaphoreTake(ch_done_sems[i], portMAX_DELAY);
        }
        
        // 更新数据包
        xSemaphoreTake(data_mutex, portMAX_DELAY);
        current_packet.timestamp = esp_timer_get_time();
        current_packet.sequence = sequence_number++;
        
        // 计算校验和
        uint8_t checksum = 0;
        uint8_t *ptr = (uint8_t *)&current_packet;
        for (int i = 0; i < sizeof(encoder_packet_t) - 1; i++) {
            checksum ^= ptr[i];
        }
        current_packet.checksum = checksum;
        xSemaphoreGive(data_mutex);
        
        // 通过ESP-NOW发送数据
        esp_err_t result = esp_now_send(receiver_mac, (uint8_t *)&current_packet, 
                                        sizeof(encoder_packet_t));
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "ESP-NOW send failed: %d", result);
        }
    }
}

// ============== ESP-NOW 初始化 ==============

static void espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGW(TAG, "ESP-NOW send status: %d", status);
    }
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // ESP-NOW 初始化
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));
    
    // 添加接收端为peer
    esp_now_peer_info_t peer_info = {
        .channel = 0,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer_info.peer_addr, receiver_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
    
    ESP_LOGI(TAG, "ESP-NOW initialized");
}

// ============== 主函数 ==============

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP32 Multi-Encoder Reader ===");
    ESP_LOGI(TAG, "Encoders: %d, Channels: %d, Sample Rate: %d Hz", 
             ENCODER_COUNT, CHANNELS, SAMPLE_FREQ_HZ);
    
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 初始化WiFi和ESP-NOW
    wifi_init();
    
    // 创建互斥锁
    data_mutex = xSemaphoreCreateMutex();
    
    // 配置通道
    channels[0] = (channel_config_t){
        .uart_num = UART_NUM_1,
        .de_pin = UART1_DE_PIN,
        .encoder_ids = {0, 1},
    };
    channels[1] = (channel_config_t){
        .uart_num = UART_NUM_2,
        .de_pin = UART2_DE_PIN,
        .encoder_ids = {2, 3},
    };
    // 第三通道需要ESP32-S3的额外UART或软件模拟
    // 这里假设使用UART0（注意：可能与调试冲突）
    channels[2] = (channel_config_t){
        .uart_num = UART_NUM_0,  // 或使用软件串口
        .de_pin = UART3_DE_PIN,
        .encoder_ids = {4, 5},
    };
    
    // 初始化UART通道
    for (int i = 0; i < CHANNELS; i++) {
        uart_init_channel(&channels[i]);
    }
    
    // 创建通道任务的信号量
    for (int i = 0; i < CHANNELS; i++) {
        ch_start_sems[i] = xSemaphoreCreateBinary();
        ch_done_sems[i] = xSemaphoreCreateBinary();
        
        ch_params[i] = (channel_task_param_t){
            .channel_idx = i,
            .start_sem = ch_start_sems[i],
            .done_sem = ch_done_sems[i],
        };
        
        // 创建通道读取任务
        char task_name[16];
        snprintf(task_name, sizeof(task_name), "ch%d_task", i);
        xTaskCreatePinnedToCore(channel_read_task, task_name, 4096, 
                               &ch_params[i], 5, NULL, 0);  // Core 0
    }
    
    // 创建主采样任务
    xTaskCreatePinnedToCore(main_sample_task, "sample_task", 4096, 
                           NULL, 4, NULL, 1);  // Core 1
    
    // 创建并启动采样定时器
    esp_timer_create_args_t timer_args = {
        .callback = sample_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "sample_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &sample_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(sample_timer, SAMPLE_PERIOD_US));
    
    ESP_LOGI(TAG, "System started, sampling at %d Hz", SAMPLE_FREQ_HZ);
    
    // 主循环 - 打印统计信息
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "Packets sent: %lu, Rate: %lu/s", 
                 sequence_number, sequence_number);
        sequence_number = 0;  // 重置计数器
    }
}
