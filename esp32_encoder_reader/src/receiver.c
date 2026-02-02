/**
 * ESP-NOW 数据接收端
 * 
 * 功能：
 * - 接收编码器数据包
 * - 解析并打印数据
 * - 可通过串口转发给上位机
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "RECEIVER";

#define ENCODER_COUNT   6
#define RESPONSE_SIZE   22

// 数据结构 (与发送端一致)
typedef struct {
    uint8_t raw_data[RESPONSE_SIZE];
    uint32_t timestamp;
    uint8_t valid;
} encoder_data_t;

typedef struct {
    uint32_t timestamp;
    uint32_t sequence;
    encoder_data_t encoders[ENCODER_COUNT];
    uint8_t checksum;
} __attribute__((packed)) encoder_packet_t;

// 接收队列
static QueueHandle_t recv_queue;

// 统计
static uint32_t packet_count = 0;
static uint32_t error_count = 0;
static uint32_t last_sequence = 0;

/**
 * 校验数据包
 */
static bool verify_checksum(encoder_packet_t *packet)
{
    uint8_t checksum = 0;
    uint8_t *ptr = (uint8_t *)packet;
    for (int i = 0; i < sizeof(encoder_packet_t) - 1; i++) {
        checksum ^= ptr[i];
    }
    return checksum == packet->checksum;
}

/**
 * 解析编码器原始数据
 * 根据多摩川协议解析位置/速度等信息
 * 这里是示例，需要根据实际协议修改
 */
static void parse_encoder_data(uint8_t encoder_id, uint8_t *raw_data)
{
    // 示例：假设前4字节是位置数据 (需要根据实际协议修改)
    uint32_t position = (raw_data[0] << 24) | (raw_data[1] << 16) | 
                        (raw_data[2] << 8) | raw_data[3];
    
    // 示例：假设第5-8字节是速度数据
    int32_t velocity = (raw_data[4] << 24) | (raw_data[5] << 16) | 
                       (raw_data[6] << 8) | raw_data[7];
    
    ESP_LOGI(TAG, "Encoder %d: pos=%lu, vel=%ld", encoder_id, position, velocity);
}

/**
 * ESP-NOW 接收回调
 */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, 
                           const uint8_t *data, int len)
{
    if (len != sizeof(encoder_packet_t)) {
        ESP_LOGW(TAG, "Invalid packet size: %d", len);
        error_count++;
        return;
    }
    
    encoder_packet_t *packet = (encoder_packet_t *)data;
    
    // 放入队列异步处理
    if (xQueueSend(recv_queue, packet, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Queue full, packet dropped");
    }
}

/**
 * 数据处理任务
 */
static void data_process_task(void *arg)
{
    encoder_packet_t packet;
    
    while (1) {
        if (xQueueReceive(recv_queue, &packet, portMAX_DELAY) == pdTRUE) {
            packet_count++;
            
            // 校验
            if (!verify_checksum(&packet)) {
                ESP_LOGW(TAG, "Checksum error!");
                error_count++;
                continue;
            }
            
            // 检测丢包
            if (packet.sequence != last_sequence + 1 && last_sequence != 0) {
                uint32_t lost = packet.sequence - last_sequence - 1;
                ESP_LOGW(TAG, "Lost %lu packets", lost);
            }
            last_sequence = packet.sequence;
            
            // 解析数据
            ESP_LOGI(TAG, "--- Packet #%lu ---", packet.sequence);
            for (int i = 0; i < ENCODER_COUNT; i++) {
                if (packet.encoders[i].valid) {
                    parse_encoder_data(i, packet.encoders[i].raw_data);
                } else {
                    ESP_LOGW(TAG, "Encoder %d: INVALID", i);
                }
            }
        }
    }
}

/**
 * 统计任务
 */
static void stats_task(void *arg)
{
    uint32_t last_count = 0;
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        uint32_t rate = packet_count - last_count;
        last_count = packet_count;
        
        ESP_LOGI(TAG, "=== Stats: %lu pkt/s, errors: %lu ===", rate, error_count);
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
    
    // 打印本机MAC地址 (发送端需要配置此地址)
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    // ESP-NOW 初始化
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    
    ESP_LOGI(TAG, "ESP-NOW receiver initialized");
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== ESP-NOW Encoder Data Receiver ===");
    
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 创建接收队列
    recv_queue = xQueueCreate(50, sizeof(encoder_packet_t));
    
    // 初始化WiFi和ESP-NOW
    wifi_init();
    
    // 创建处理任务
    xTaskCreate(data_process_task, "process", 4096, NULL, 5, NULL);
    xTaskCreate(stats_task, "stats", 2048, NULL, 3, NULL);
    
    ESP_LOGI(TAG, "Receiver started, waiting for data...");
}
