/**
 * ESP-NOW Encoder Data Transceiver — Broadcast Mode
 *
 * FLAGE==1: Receiver (master) — receives encoder data via ESP-NOW,
 *           transparently forwards to UART.
 * FLAGE==0: Sender (server)  — reads encoders via RS485, sends via ESP-NOW
 *           broadcast.
 *
 * CHANGES (v2):
 *   1. Sender uses BROADCAST (FF:FF:FF:FF:FF:FF) — no MAC config needed.
 *   2. Receiver callback compatible with both Arduino ESP32 v2.x and v3.x.
 *   3. Receiver logs ALL incoming ESP-NOW packets (not just matching size).
 *   4. Robust channel setting with retry and verification.
 *   5. Sender channel fix retained (set_channel AFTER wifi_start).
 */

#define FLAGE 0

#if FLAGE == 1  // ==================== RECEIVER (master) ====================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <cstring>

namespace {
constexpr uint32_t kUartBaudrate = 2000000;
constexpr uint8_t kEspNowChannel = 1;
constexpr bool kDebugSerial = true;

constexpr uint8_t kMaxEncoders = 32;
constexpr uint16_t kUartMagic = 0xA55A;

struct EncoderPacket {
    uint32_t seq;
    uint8_t count;
    uint8_t all_ok;
    uint16_t values[kMaxEncoders];
    uint8_t status[kMaxEncoders];
} __attribute__((packed));

struct UartFrameHeader {
    uint16_t magic;
    uint16_t length;
} __attribute__((packed));

EncoderPacket gPacket;
volatile bool gPacketReady = false;
volatile uint32_t gRxCount = 0;
volatile uint32_t gRxBadLen = 0;
volatile uint32_t gLastRxMs = 0;
volatile int gLastLen = 0;
uint8_t gLastMac[6] = {};
uint8_t gLocalMac[6] = {};
portMUX_TYPE gPacketMux = portMUX_INITIALIZER_UNLOCKED;

// ---- ESP-NOW receive callback (compatible with v2.x and v3.x) ----
//
// Arduino ESP32 v2.x (ESP-IDF 4.x):
//   typedef void (*esp_now_recv_cb_t)(const uint8_t *mac_addr,
//                                     const uint8_t *data, int data_len);
//
// Arduino ESP32 v3.x (ESP-IDF 5.x):
//   typedef void (*esp_now_recv_cb_t)(const esp_now_recv_info_t *esp_now_info,
//                                     const uint8_t *data, int data_len);
//
// We detect the version at compile time.

#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
// ---- v3.x / ESP-IDF 5.x callback ----
void onEspNowReceive(const esp_now_recv_info_t* info, const uint8_t* data,
                     int len) {
    const uint8_t* mac = info->src_addr;
#else
// ---- v2.x / ESP-IDF 4.x callback ----
void onEspNowReceive(const uint8_t* mac, const uint8_t* data, int len) {
#endif
    portENTER_CRITICAL(&gPacketMux);
    gRxCount++;
    gLastRxMs = millis();
    gLastLen = len;
    if (mac) {
        memcpy(gLastMac, mac, 6);
    }

    if (len == static_cast<int>(sizeof(EncoderPacket))) {
        memcpy(&gPacket, data, sizeof(EncoderPacket));
        gPacketReady = true;
    } else {
        gRxBadLen++;
    }
    portEXIT_CRITICAL(&gPacketMux);
}

/**
 * Set WiFi channel with retry and verification.
 */
bool setChannelRobust(uint8_t channel, int maxRetries = 5) {
    for (int attempt = 0; attempt < maxRetries; ++attempt) {
        esp_err_t err =
            esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
        if (err != ESP_OK) {
            Serial.printf("# set_channel attempt %d failed: 0x%X\n",
                          attempt + 1, err);
            delay(100);
            continue;
        }

        // Verify
        uint8_t actualCh = 0;
        wifi_second_chan_t secondCh;
        esp_wifi_get_channel(&actualCh, &secondCh);
        if (actualCh == channel) {
            Serial.printf("# Channel set to %u (verified, attempt %d)\n",
                          actualCh, attempt + 1);
            return true;
        }

        Serial.printf("# Channel mismatch: wanted %u got %u, retrying...\n",
                      channel, actualCh);
        delay(100);
    }
    return false;
}

bool initEspNow() {
    // Step 1: Initialise WiFi in STA mode
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.disconnect(true, true);
    delay(100);

    // Step 2: Set channel (WiFi must be started — WiFi.mode(WIFI_STA) does that)
    if (!setChannelRobust(kEspNowChannel)) {
        Serial.println("# ERROR: Failed to set WiFi channel!");
        return false;
    }

    // Step 3: Init ESP-NOW
    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        Serial.printf("# ERROR: esp_now_init failed: 0x%X\n", err);
        return false;
    }

    // Step 4: Register receive callback
    esp_now_register_recv_cb(onEspNowReceive);

    Serial.println("# ESP-NOW receiver init OK (broadcast mode).");
    return true;
}

void sendPacketUart(const EncoderPacket& pkt) {
    UartFrameHeader header = {kUartMagic,
                              static_cast<uint16_t>(sizeof(EncoderPacket))};
    Serial.write(reinterpret_cast<const uint8_t*>(&header), sizeof(header));
    Serial.write(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}
}  // namespace

void setup() {
    Serial.begin(kUartBaudrate);
    Serial.setRxBufferSize(512);
    Serial.setTxBufferSize(512);
    delay(300);

    Serial.println("# ===== ESP-NOW Receiver (Broadcast) =====");
    Serial.printf("# EncoderPacket size = %u bytes\n",
                  (unsigned)sizeof(EncoderPacket));

    if (!initEspNow()) {
        Serial.println("# ESP-NOW init FAILED!");
    }

    WiFi.macAddress(gLocalMac);
    Serial.printf("# Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  gLocalMac[0], gLocalMac[1], gLocalMac[2],
                  gLocalMac[3], gLocalMac[4], gLocalMac[5]);

    uint8_t ch = 0;
    wifi_second_chan_t sch;
    esp_wifi_get_channel(&ch, &sch);
    Serial.printf("# Current WiFi channel: %u\n", ch);
}

void loop() {
    EncoderPacket local;
    bool hasPacket = false;
    uint32_t rxCount = 0;
    uint32_t rxBadLen = 0;
    uint32_t lastRxMs = 0;
    int lastLen = 0;
    uint8_t lastMac[6] = {};
    static uint32_t lastReportMs = 0;

    portENTER_CRITICAL(&gPacketMux);
    if (gPacketReady) {
        local = gPacket;
        gPacketReady = false;
        hasPacket = true;
    }
    rxCount = gRxCount;
    rxBadLen = gRxBadLen;
    lastRxMs = gLastRxMs;
    lastLen = gLastLen;
    memcpy(lastMac, gLastMac, sizeof(lastMac));
    portEXIT_CRITICAL(&gPacketMux);

    // Transparent pass-through: forward to UART immediately
    if (hasPacket) {
        sendPacketUart(local);
    }

    if (kDebugSerial) {
        uint32_t now = millis();
        if (now - lastReportMs >= 1000) {
            lastReportMs = now;

            uint8_t ch = 0;
            wifi_second_chan_t sch;
            esp_wifi_get_channel(&ch, &sch);

            Serial.printf(
                "# RX: cnt=%lu badlen=%lu last_ms=%lu len=%d ch=%u "
                "mac=%02X:%02X:%02X:%02X:%02X:%02X\n",
                static_cast<unsigned long>(rxCount),
                static_cast<unsigned long>(rxBadLen),
                static_cast<unsigned long>(lastRxMs), lastLen, ch,
                lastMac[0], lastMac[1], lastMac[2],
                lastMac[3], lastMac[4], lastMac[5]);
        }
    }

    vTaskDelay(1);
}

#else  // ======================= SENDER (server) ===========================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <cstring>

#include "MultiEncoder.h"

namespace {
constexpr uint8_t kEncoderIds[] = {1, 2, 3, 4, 5, 6};
constexpr uint8_t kEncoderCount = sizeof(kEncoderIds) / sizeof(kEncoderIds[0]);

constexpr int kRs485RxPin = 32;
constexpr int kRs485TxPin = 33;
constexpr int kRs485DeRePin = 25;
constexpr uint32_t kRs485Baudrate = 2500000;

// BROADCAST — no need to know receiver MAC
constexpr uint8_t kBroadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

constexpr uint8_t kEspNowChannel = 1;
constexpr uint32_t kEspNowSendIntervalMs = 5;
constexpr uint8_t kEncoderTaskCore = 1;
constexpr UBaseType_t kEncoderTaskPriority = tskIDLE_PRIORITY + 2;
constexpr uint8_t kSendTaskCore = 0;

MultiEncoder encoder({
    .numEncoders = kEncoderCount,
    .encoderIds = kEncoderIds,
    .rxPin = kRs485RxPin,
    .txPin = kRs485TxPin,
    .deRePin = kRs485DeRePin,
    .baudrate = kRs485Baudrate,
    .responseTimeoutUs = 300,
    .interByteTimeoutUs = 80,
    .txTurnaroundUs = 32,
    .serialRxBufferSize = 256,
    .serialTxBufferSize = 256,
    .idleDelayUs = 0,
    .yieldEveryBatches = 16,
    .yieldDelayTicks = 1,
});

struct EncoderPacket {
    uint32_t seq;
    uint8_t count;
    uint8_t all_ok;
    uint16_t values[MultiEncoder::kMaxEncoders];
    uint8_t status[MultiEncoder::kMaxEncoders];
} __attribute__((packed));

EncoderPacket packet;
uint16_t values[MultiEncoder::kMaxEncoders];
bool status[MultiEncoder::kMaxEncoders];

volatile uint32_t gSendOkCount = 0;
volatile uint32_t gSendFailCount = 0;

#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 5
void onEspNowSent(const uint8_t* mac, esp_now_send_status_t sendStatus) {
#else
void onEspNowSent(const uint8_t* mac, esp_now_send_status_t sendStatus) {
#endif
    if (sendStatus == ESP_NOW_SEND_SUCCESS) {
        gSendOkCount++;
    } else {
        gSendFailCount++;
    }
}

/**
 * Set WiFi channel with retry and verification.
 */
bool setChannelRobust(uint8_t channel, int maxRetries = 5) {
    for (int attempt = 0; attempt < maxRetries; ++attempt) {
        esp_err_t err =
            esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
        if (err != ESP_OK) {
            Serial.printf("# set_channel attempt %d failed: 0x%X\n",
                          attempt + 1, err);
            delay(100);
            continue;
        }

        uint8_t actualCh = 0;
        wifi_second_chan_t secondCh;
        esp_wifi_get_channel(&actualCh, &secondCh);
        if (actualCh == channel) {
            Serial.printf("# Channel set to %u (verified, attempt %d)\n",
                          actualCh, attempt + 1);
            return true;
        }

        Serial.printf("# Channel mismatch: wanted %u got %u, retrying...\n",
                      channel, actualCh);
        delay(100);
    }
    return false;
}

/**
 * Initialise WiFi + ESP-NOW for the sender.
 *
 * KEY CHANGES:
 *   1. Removed esp_wifi_stop()/esp_wifi_start() — set_channel called
 *      when WiFi is already running after WiFi.mode(WIFI_STA).
 *   2. Uses BROADCAST peer (FF:FF:FF:FF:FF:FF) — receiver does not
 *      need to be pre-configured.
 *   3. Channel setting with retry and verification.
 */
bool initEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.disconnect(true, true);
    delay(100);

    // Set channel AFTER WiFi is started
    if (!setChannelRobust(kEspNowChannel)) {
        Serial.println("# ERROR: Failed to set WiFi channel!");
        return false;
    }

    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        Serial.printf("# ERROR: esp_now_init failed: 0x%X\n", err);
        return false;
    }

    esp_now_register_send_cb(onEspNowSent);

    // Add BROADCAST peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, kBroadcastMac, 6);
    peerInfo.channel = 0;  // 0 = use current channel for broadcast
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;

    err = esp_now_add_peer(&peerInfo);
    if (err != ESP_OK) {
        Serial.printf("# ERROR: esp_now_add_peer(broadcast) failed: 0x%X\n",
                      err);
        return false;
    }

    Serial.println("# ESP-NOW sender init OK (broadcast mode).");
    return true;
}

void taskEspNowSender(void* parameter) {
    uint32_t lastSeq = 0;
    uint32_t lastSendMs = 0;
    bool pending = false;
    uint32_t localSendCount = 0;
    uint32_t localFailCount = 0;

    while (true) {
        bool allOk = false;
        if (encoder.copyIfNew(&lastSeq, values, status, &allOk)) {
            packet.seq = lastSeq;
            packet.count = encoder.count();
            packet.all_ok = allOk ? 1 : 0;
            for (uint8_t i = 0; i < packet.count; ++i) {
                packet.values[i] = values[i];
                packet.status[i] = status[i] ? 1 : 0;
            }
            for (uint8_t i = packet.count; i < MultiEncoder::kMaxEncoders;
                 ++i) {
                packet.values[i] = 0;
                packet.status[i] = 0;
            }
            pending = true;
        }

        uint32_t now = millis();
        if (pending &&
            (kEspNowSendIntervalMs == 0 ||
             (now - lastSendMs) >= kEspNowSendIntervalMs)) {
            lastSendMs = now;
            pending = false;

            esp_err_t result = esp_now_send(
                kBroadcastMac,
                reinterpret_cast<const uint8_t*>(&packet),
                sizeof(packet));
            localSendCount++;

            if (result != ESP_OK) {
                localFailCount++;
            }

            // Periodic diagnostics
            static uint32_t lastDiagMs = 0;
            if (now - lastDiagMs >= 2000) {
                lastDiagMs = now;

                uint8_t ch = 0;
                wifi_second_chan_t sch;
                esp_wifi_get_channel(&ch, &sch);

                Serial.printf(
                    "# TX: seq=%lu sent=%lu api_fail=%lu "
                    "cb_ok=%lu cb_fail=%lu ch=%u pkt_size=%u\n",
                    static_cast<unsigned long>(packet.seq),
                    static_cast<unsigned long>(localSendCount),
                    static_cast<unsigned long>(localFailCount),
                    static_cast<unsigned long>(gSendOkCount),
                    static_cast<unsigned long>(gSendFailCount),
                    ch,
                    (unsigned)sizeof(packet));
            }
        }

        vTaskDelay(1);
    }
}
}  // namespace

void setup() {
    Serial.begin(2000000);
    delay(300);

    Serial.println("# ===== ESP-NOW Encoder Sender (Broadcast) =====");
    Serial.printf("# EncoderPacket size = %u bytes\n",
                  (unsigned)sizeof(EncoderPacket));

    if (!encoder.begin(Serial2)) {
        Serial.println("# Encoder init failed.");
        return;
    }

    encoder.start(kEncoderTaskCore, kEncoderTaskPriority, 4096);

    if (!initEspNow()) {
        Serial.println("# ESP-NOW init FAILED!");
        return;
    }

    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("# Sender MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    uint8_t ch = 0;
    wifi_second_chan_t sch;
    esp_wifi_get_channel(&ch, &sch);
    Serial.printf("# Current WiFi channel: %u\n", ch);
    Serial.printf("# Sending to: BROADCAST (FF:FF:FF:FF:FF:FF)\n");

    xTaskCreatePinnedToCore(
        taskEspNowSender,
        "EspNowSend",
        4096,
        nullptr,
        tskIDLE_PRIORITY + 1,
        nullptr,
        kSendTaskCore);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}

#endif
