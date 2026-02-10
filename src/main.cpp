/**
 * ESP-NOW Encoder Data Transceiver
 *
 * FLAGE==1: Receiver (master) — receives encoder data via ESP-NOW,
 *           transparently forwards to UART.
 * FLAGE==0: Sender (server)  — reads encoders via RS485, sends via ESP-NOW.
 *
 * KEY FIX: The sender's initEspNow() previously called esp_wifi_set_channel()
 *          between esp_wifi_stop() and esp_wifi_start(). This silently fails
 *          because the WiFi driver must be started before setting the channel.
 *          The sender ended up on the wrong channel, so the receiver on ch1
 *          never received any packets.
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
volatile uint32_t gLastRxMs = 0;
volatile int gLastLen = 0;
uint8_t gLastMac[6] = {};
uint8_t gLocalMac[6] = {};
portMUX_TYPE gPacketMux = portMUX_INITIALIZER_UNLOCKED;

/**
 * ESP-NOW receive callback.
 * Copies the incoming packet into the global buffer for the main loop
 * to forward via UART (transparent pass-through).
 */
void onEspNowReceive(const uint8_t* mac, const uint8_t* data, int len) {
    if (len != static_cast<int>(sizeof(EncoderPacket))) {
        return;
    }

    portENTER_CRITICAL(&gPacketMux);
    memcpy(&gPacket, data, sizeof(EncoderPacket));
    gPacketReady = true;
    gRxCount++;
    gLastRxMs = millis();
    gLastLen = len;
    memcpy(gLastMac, mac, sizeof(gLastMac));
    portEXIT_CRITICAL(&gPacketMux);
}

/**
 * Initialise WiFi + ESP-NOW for the receiver.
 *
 * FIX: Ensure esp_wifi_set_channel() is called AFTER WiFi is fully started.
 *      Previously in sender code, it was called between stop/start, causing
 *      silent failure. Here we also add error checking and a small delay
 *      for robustness.
 */
bool initEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.disconnect(true, true);

    // Ensure WiFi is started before setting channel
    delay(100);

    esp_err_t err = esp_wifi_set_channel(kEspNowChannel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        Serial.printf("# ERROR: esp_wifi_set_channel failed: 0x%X\n", err);
        return false;
    }

    err = esp_now_init();
    if (err != ESP_OK) {
        Serial.printf("# ERROR: esp_now_init failed: 0x%X\n", err);
        return false;
    }

    esp_now_register_recv_cb(onEspNowReceive);

    // Verify actual channel
    uint8_t primaryCh = 0;
    wifi_second_chan_t secondCh;
    esp_wifi_get_channel(&primaryCh, &secondCh);
    Serial.printf("# WiFi channel verified: primary=%u\n", primaryCh);

    return true;
}

/**
 * Send an EncoderPacket over UART with a magic header (transparent pass-through).
 */
void sendPacketUart(const EncoderPacket& packet) {
    UartFrameHeader header = {kUartMagic,
                              static_cast<uint16_t>(sizeof(EncoderPacket))};
    Serial.write(reinterpret_cast<const uint8_t*>(&header), sizeof(header));
    Serial.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
}
}  // namespace

void setup() {
    Serial.begin(kUartBaudrate);
    Serial.setRxBufferSize(512);
    Serial.setTxBufferSize(512);

    delay(200);  // Allow UART to settle

    if (!initEspNow()) {
        Serial.println("# ESP-NOW init failed!");
    } else {
        Serial.println("# ESP-NOW receiver initialised OK.");
    }

    WiFi.macAddress(gLocalMac);
    if (kDebugSerial) {
        Serial.printf("# Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X ch=%u\n",
                      gLocalMac[0], gLocalMac[1], gLocalMac[2],
                      gLocalMac[3], gLocalMac[4], gLocalMac[5],
                      kEspNowChannel);
    }
}

void loop() {
    EncoderPacket local;
    bool hasPacket = false;
    uint32_t rxCount = 0;
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
    lastRxMs = gLastRxMs;
    lastLen = gLastLen;
    memcpy(lastMac, gLastMac, sizeof(lastMac));
    portEXIT_CRITICAL(&gPacketMux);

    // Transparent pass-through: immediately forward received packet to UART
    if (hasPacket) {
        sendPacketUart(local);
    }

    if (kDebugSerial) {
        uint32_t now = millis();
        if (now - lastReportMs >= 1000) {
            lastReportMs = now;
            Serial.printf(
                "# ESP-NOW rx=%lu last_ms=%lu len=%d "
                "mac=%02X:%02X:%02X:%02X:%02X:%02X\n",
                static_cast<unsigned long>(rxCount),
                static_cast<unsigned long>(lastRxMs), lastLen,
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

// Update with receiver MAC address.
constexpr uint8_t kPeerMac[6] = {0xA4, 0xF0, 0x0F, 0x1F, 0x04, 0xA0};
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

volatile bool gSendOk = false;

/**
 * ESP-NOW send callback — used for diagnostics.
 */
void onEspNowSent(const uint8_t* mac, esp_now_send_status_t sendStatus) {
    gSendOk = (sendStatus == ESP_NOW_SEND_SUCCESS);
}

/**
 * Initialise WiFi + ESP-NOW for the sender.
 *
 * CRITICAL FIX: The original code called esp_wifi_set_channel() BETWEEN
 * esp_wifi_stop() and esp_wifi_start(). The ESP-IDF WiFi driver requires
 * WiFi to be started before setting the channel. The call silently failed,
 * leaving the sender on the default channel (not channel 1), so the receiver
 * on channel 1 never received any data.
 *
 * Fixed sequence:
 *   1. WiFi.mode(WIFI_STA)       — initialises and starts WiFi
 *   2. WiFi.disconnect()          — disconnect from any AP
 *   3. esp_wifi_set_channel()     — set channel AFTER WiFi is started
 */
bool initEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.disconnect(true, true);

    // FIX: Set channel AFTER WiFi is started (WiFi.mode already starts it).
    //      Do NOT call esp_wifi_stop() before esp_wifi_set_channel().
    delay(100);

    esp_err_t err = esp_wifi_set_channel(kEspNowChannel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        Serial.printf("# ERROR: esp_wifi_set_channel failed: 0x%X\n", err);
        return false;
    }

    err = esp_now_init();
    if (err != ESP_OK) {
        Serial.printf("# ERROR: esp_now_init failed: 0x%X\n", err);
        return false;
    }

    esp_now_register_send_cb(onEspNowSent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, kPeerMac, sizeof(kPeerMac));
    peerInfo.channel = kEspNowChannel;
    peerInfo.encrypt = false;

    err = esp_now_add_peer(&peerInfo);
    if (err != ESP_OK) {
        Serial.printf("# ERROR: esp_now_add_peer failed: 0x%X\n", err);
        return false;
    }

    // Verify actual channel
    uint8_t primaryCh = 0;
    wifi_second_chan_t secondCh;
    esp_wifi_get_channel(&primaryCh, &secondCh);
    Serial.printf("# WiFi channel verified: primary=%u\n", primaryCh);

    return true;
}

void taskEspNowSender(void* parameter) {
    uint32_t lastSeq = 0;
    uint32_t lastSendMs = 0;
    bool pending = false;
    uint32_t sendCount = 0;
    uint32_t failCount = 0;

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
            for (uint8_t i = packet.count; i < MultiEncoder::kMaxEncoders; ++i) {
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
                kPeerMac,
                reinterpret_cast<const uint8_t*>(&packet),
                sizeof(packet));
            sendCount++;

            if (result != ESP_OK) {
                failCount++;
            }

            // Periodic sender diagnostics
            static uint32_t lastDiagMs = 0;
            if (now - lastDiagMs >= 2000) {
                lastDiagMs = now;
                Serial.printf(
                    "# SENDER: seq=%lu sent=%lu fail=%lu lastOk=%d\n",
                    static_cast<unsigned long>(packet.seq),
                    static_cast<unsigned long>(sendCount),
                    static_cast<unsigned long>(failCount),
                    gSendOk ? 1 : 0);
            }
        }

        vTaskDelay(1);
    }
}
}  // namespace

void setup() {
    Serial.begin(2000000);
    delay(200);

    Serial.println("# --- ESP-NOW Encoder Sender ---");

    if (!encoder.begin(Serial2)) {
        Serial.println("# Encoder init failed.");
        return;
    }

    encoder.start(kEncoderTaskCore, kEncoderTaskPriority, 4096);

    if (!initEspNow()) {
        Serial.println("# ESP-NOW init failed!");
        return;
    }

    // Print sender MAC so the receiver can be configured
    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("# Sender MAC: %02X:%02X:%02X:%02X:%02X:%02X ch=%u\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                  kEspNowChannel);
    Serial.printf("# Target MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  kPeerMac[0], kPeerMac[1], kPeerMac[2],
                  kPeerMac[3], kPeerMac[4], kPeerMac[5]);

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
