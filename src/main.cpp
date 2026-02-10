/**
 * ESP-NOW Encoder Data Transceiver — Broadcast Mode
 *
 * FLAGE==0: Receiver (master) — 接收所有发送端的广播数据，串口输出角度值
 * FLAGE==1: Sender  (server) — RS485 读编码器，广播发送 ESP-NOW 数据包
 *
 * 广播模式优势：
 *   - 发送端无需配置接收端 MAC 地址
 *   - 多个发送端可同时向一个接收端发数据
 *   - 接收端自动接收所有 ESP-NOW 广播包
 */

#define FLAGE 0

#if FLAGE == 0  // ==================== RECEIVER ====================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <cstring>

namespace {

constexpr uint32_t kUartBaudrate   = 2000000;
constexpr uint8_t  kEspNowChannel  = 1;
constexpr uint8_t  kMaxEncoders    = 32;

struct EncoderPacket {
    uint32_t seq;
    uint8_t  count;
    uint8_t  all_ok;
    uint16_t values[kMaxEncoders];
    uint8_t  status[kMaxEncoders];
} __attribute__((packed));

// ---------- 共享状态（回调 ↔ loop）----------
EncoderPacket gPacket;
volatile bool     gPacketReady = false;
volatile uint32_t gRxCount     = 0;
volatile uint32_t gRxValid     = 0;
volatile uint32_t gRxBadLen    = 0;
volatile uint32_t gLastRxMs    = 0;
volatile int      gLastLen     = 0;
uint8_t           gLastMac[6]  = {};
portMUX_TYPE      gMux = portMUX_INITIALIZER_UNLOCKED;

// ---------- ESP-NOW 接收回调 ----------
// 注意：回调运行在 WiFi 任务上下文，禁止做 Serial 输出等耗时操作！
void onReceive(const uint8_t* mac, const uint8_t* data, int len) {
    portENTER_CRITICAL(&gMux);
    gRxCount++;
    gLastRxMs = millis();
    gLastLen  = len;
    memcpy(gLastMac, mac, 6);

    if (len == static_cast<int>(sizeof(EncoderPacket))) {
        memcpy(&gPacket, data, sizeof(EncoderPacket));
        gPacketReady = true;
        gRxValid++;
    } else {
        gRxBadLen++;
    }
    portEXIT_CRITICAL(&gMux);
}

// ---------- WiFi + ESP-NOW 初始化 ----------
bool initEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.disconnect(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    delay(100);

    // 设置信道（带重试）
    esp_err_t err;
    for (int retry = 0; retry < 3; ++retry) {
        err = esp_wifi_set_channel(kEspNowChannel, WIFI_SECOND_CHAN_NONE);
        if (err == ESP_OK) break;
        delay(50);
    }
    if (err != ESP_OK) {
        Serial.printf("# ERROR: set_channel failed: %d\n", static_cast<int>(err));
        return false;
    }

    // 验证信道
    uint8_t ch = 0;
    wifi_second_chan_t sec;
    esp_wifi_get_channel(&ch, &sec);
    if (ch != kEspNowChannel) {
        Serial.printf("# ERROR: channel mismatch, want %u got %u\n", kEspNowChannel, ch);
        return false;
    }

    err = esp_now_init();
    if (err != ESP_OK) {
        Serial.printf("# ERROR: esp_now_init failed: %d\n", static_cast<int>(err));
        return false;
    }

    // 广播接收无需注册 peer，只需注册回调
    esp_now_register_recv_cb(onReceive);
    return true;
}

// ---------- 角度输出 ----------
void printAngles(const EncoderPacket& pkt) {
    for (uint8_t i = 0; i < pkt.count; ++i) {
        uint32_t a = (static_cast<uint32_t>(pkt.values[i]) * 1125) >> 11;
        Serial.printf("%3lu.%02lu",
                      static_cast<unsigned long>(a / 100),
                      static_cast<unsigned long>(a % 100));
        if (i < pkt.count - 1) Serial.print(',');
    }
    Serial.println();
}

}  // namespace

void setup() {
    Serial.begin(kUartBaudrate);
    delay(200);

    bool ok = initEspNow();

    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("# Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X  ch=%u  init=%s\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                  kEspNowChannel, ok ? "OK" : "FAIL");
    Serial.printf("# EncoderPacket size: %u bytes\n",
                  static_cast<unsigned>(sizeof(EncoderPacket)));
    Serial.println("# Waiting for broadcast data from senders...");
}

void loop() {
    EncoderPacket local;
    bool hasPacket = false;

    portENTER_CRITICAL(&gMux);
    if (gPacketReady) {
        local = gPacket;
        gPacketReady = false;
        hasPacket = true;
    }
    portEXIT_CRITICAL(&gMux);

    if (hasPacket) {
        printAngles(local);
    }

    // 每 2 秒打印状态
    static uint32_t lastReport = 0;
    uint32_t now = millis();
    if (now - lastReport >= 2000) {
        lastReport = now;
        uint32_t rxc, rxv, rxb, lms;
        int llen;
        uint8_t lmac[6];
        portENTER_CRITICAL(&gMux);
        rxc = gRxCount; rxv = gRxValid; rxb = gRxBadLen;
        lms = gLastRxMs; llen = gLastLen;
        memcpy(lmac, gLastMac, 6);
        portEXIT_CRITICAL(&gMux);

        Serial.printf("-- STAT rx=%lu ok=%lu bad=%lu len=%d from=%02X:%02X:%02X:%02X:%02X:%02X ago=%lums\n",
                      static_cast<unsigned long>(rxc),
                      static_cast<unsigned long>(rxv),
                      static_cast<unsigned long>(rxb),
                      llen,
                      lmac[0], lmac[1], lmac[2], lmac[3], lmac[4], lmac[5],
                      static_cast<unsigned long>(now - lms));
    }

    vTaskDelay(1);
}

#else  // ==================== SENDER ====================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
#include <cstring>

#include "MultiEncoder.h"

namespace {

// ---------- 编码器配置 ----------
constexpr uint8_t  kEncoderIds[]    = {1, 2, 3, 4, 5, 6};
constexpr uint8_t  kEncoderCount    = sizeof(kEncoderIds) / sizeof(kEncoderIds[0]);
constexpr int      kRs485RxPin      = 32;
constexpr int      kRs485TxPin      = 33;
constexpr int      kRs485DeRePin    = 25;
constexpr uint32_t kRs485Baudrate   = 2500000;

// ---------- ESP-NOW 配置 ----------
constexpr uint8_t  kBroadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
constexpr uint8_t  kEspNowChannel   = 1;
constexpr uint32_t kSendIntervalMs  = 5;

// ---------- 双核任务配置 ----------
constexpr uint8_t     kEncoderCore  = 1;  // Core 1: 编码器采集（独占，无 WiFi 干扰）
constexpr UBaseType_t kEncoderPrio  = configMAX_PRIORITIES - 1;
constexpr uint8_t     kSendCore     = 0;  // Core 0: ESP-NOW（与 WiFi 栈同核）

// ---------- 编码器实例 ----------
MultiEncoder encoder({
    .numEncoders       = kEncoderCount,
    .encoderIds        = kEncoderIds,
    .rxPin             = kRs485RxPin,
    .txPin             = kRs485TxPin,
    .deRePin           = kRs485DeRePin,
    .baudrate          = kRs485Baudrate,
    .responseTimeoutUs = 300,
    .interByteTimeoutUs = 80,
    .txTurnaroundUs    = 32,
    .serialRxBufferSize = 256,
    .serialTxBufferSize = 256,
    .idleDelayUs       = 0,
    .yieldEveryBatches = 16,
    .yieldDelayTicks   = 1,
});

// ---------- 数据包定义（必须与接收端 sizeof 一致）----------
struct EncoderPacket {
    uint32_t seq;
    uint8_t  count;
    uint8_t  all_ok;
    uint16_t values[32];  // 与接收端 kMaxEncoders=32 一致
    uint8_t  status[32];
} __attribute__((packed));

EncoderPacket gPacket;
uint16_t gValues[32];
bool     gStatus[32];

volatile uint32_t gTxOk   = 0;
volatile uint32_t gTxFail = 0;
bool gEspNowReady = false;

// ---------- 回调 ----------
void onSent(const uint8_t* mac, esp_now_send_status_t st) {
    if (st == ESP_NOW_SEND_SUCCESS) gTxOk++; else gTxFail++;
}

// ---------- WiFi + ESP-NOW 初始化 ----------
bool initEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.disconnect(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    delay(100);

    // 设置信道（带重试）
    esp_err_t err;
    for (int retry = 0; retry < 3; ++retry) {
        err = esp_wifi_set_channel(kEspNowChannel, WIFI_SECOND_CHAN_NONE);
        if (err == ESP_OK) break;
        delay(50);
    }
    if (err != ESP_OK) {
        Serial.printf("# ERROR: set_channel failed: %d\n", static_cast<int>(err));
        return false;
    }

    // 验证信道
    uint8_t ch = 0;
    wifi_second_chan_t sec;
    esp_wifi_get_channel(&ch, &sec);
    if (ch != kEspNowChannel) {
        Serial.printf("# ERROR: channel mismatch, want %u got %u\n", kEspNowChannel, ch);
        return false;
    }

    err = esp_now_init();
    if (err != ESP_OK) {
        Serial.printf("# ERROR: esp_now_init failed: %d\n", static_cast<int>(err));
        return false;
    }
    esp_now_register_send_cb(onSent);

    // 添加广播 peer — channel=0 表示使用当前接口信道
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, kBroadcastMac, 6);
    peer.channel = 0;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        Serial.printf("# ERROR: add_peer failed: %d\n", static_cast<int>(err));
        return false;
    }

    return true;
}

// ---------- ESP-NOW 发送任务 (Core 0) ----------
void taskSender(void*) {
    uint32_t lastSeq    = 0;
    uint32_t lastSendMs = 0;

    while (true) {
        if (!gEspNowReady) { vTaskDelay(10); continue; }

        bool allOk = false;
        if (encoder.copyIfNew(&lastSeq, gValues, gStatus, &allOk)) {
            gPacket.seq    = lastSeq;
            gPacket.count  = encoder.count();
            gPacket.all_ok = allOk ? 1 : 0;
            for (uint8_t i = 0; i < gPacket.count; ++i) {
                gPacket.values[i] = gValues[i];
                gPacket.status[i] = gStatus[i] ? 1 : 0;
            }
            for (uint8_t i = gPacket.count; i < 32; ++i) {
                gPacket.values[i] = 0;
                gPacket.status[i] = 0;
            }
        }

        uint32_t now = millis();
        if (lastSeq > 0 && (now - lastSendMs) >= kSendIntervalMs) {
            lastSendMs = now;
            esp_now_send(kBroadcastMac,
                         reinterpret_cast<const uint8_t*>(&gPacket),
                         sizeof(gPacket));
        }

        vTaskDelay(1);
    }
}

}  // namespace

void setup() {
    Serial.begin(2000000);
    delay(200);

    // 1. 编码器初始化
    bool encOk = encoder.begin(Serial2);
    if (!encOk) {
        Serial.println("# Encoder init failed!");
    }

    // 2. 启动编码器采集任务（Core 1, 最高优先级）
    if (encOk) {
        esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(kEncoderCore));
        encoder.start(kEncoderCore, kEncoderPrio, 4096);
    }

    // 3. ESP-NOW 初始化
    gEspNowReady = initEspNow();
    if (!gEspNowReady) {
        Serial.println("# ESP-NOW init failed!");
    }

    memset(&gPacket, 0, sizeof(gPacket));
    gPacket.count = encoder.count();

    // 4. 启动发送任务（Core 0）
    xTaskCreatePinnedToCore(taskSender, "Sender", 4096, nullptr,
                            tskIDLE_PRIORITY + 1, nullptr, kSendCore);

    // 5. 打印配置信息
    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("# Sender MAC: %02X:%02X:%02X:%02X:%02X:%02X  ch=%u\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], kEspNowChannel);
    Serial.printf("# Broadcast to: FF:FF:FF:FF:FF:FF\n");
    Serial.printf("# EncoderPacket size: %u bytes\n",
                  static_cast<unsigned>(sizeof(EncoderPacket)));
    Serial.printf("# enc=%s espnow=%s dual_core=yes\n",
                  encOk ? "OK" : "FAIL", gEspNowReady ? "OK" : "FAIL");
}

void loop() {
    static uint32_t lastReport = 0;
    uint32_t now = millis();
    if (now - lastReport >= 2000) {
        lastReport = now;
        Serial.printf("-- TX ok=%lu fail=%lu seq=%lu all_ok=%u\n",
                      static_cast<unsigned long>(gTxOk),
                      static_cast<unsigned long>(gTxFail),
                      static_cast<unsigned long>(gPacket.seq),
                      gPacket.all_ok);
    }
    vTaskDelay(100);
}

#endif
