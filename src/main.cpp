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
constexpr uint8_t kPeerMac[6] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};
constexpr uint8_t kEspNowChannel = 1;
constexpr uint32_t kEspNowSendIntervalMs = 5;
constexpr bool kDebugSerial = true;
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
    .yieldEveryBatches = 64,
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

volatile uint32_t gTxOkCount = 0;
volatile uint32_t gTxFailCount = 0;
volatile esp_err_t gLastSendErr = ESP_OK;
uint8_t gLocalMac[6] = {};

void onEspNowSend(const uint8_t* mac, esp_now_send_status_t status) {
    (void)mac;
    if (status == ESP_NOW_SEND_SUCCESS) {
        gTxOkCount++;
    } else {
        gTxFailCount++;
    }
}

bool initEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.disconnect(true, true);
    esp_wifi_set_channel(kEspNowChannel, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        return false;
    }

    esp_now_register_send_cb(onEspNowSend);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, kPeerMac, sizeof(kPeerMac));
    peerInfo.channel = kEspNowChannel;
    peerInfo.encrypt = false;

    esp_err_t addResult = esp_now_add_peer(&peerInfo);
    if (addResult != ESP_OK && addResult != ESP_ERR_ESPNOW_EXIST) {
        gLastSendErr = addResult;
        return false;
    }

    return true;
}

void taskEspNowSender(void* parameter) {
    uint32_t lastSeq = 0;
    uint32_t lastSendMs = 0;
    bool pending = false;

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
            esp_now_send(kPeerMac, reinterpret_cast<const uint8_t*>(&packet),
                         sizeof(packet));
        }

        vTaskDelay(1);
    }
}
}  // namespace

void setup() {
    Serial.begin(2000000);

    if (!encoder.begin(Serial2)) {
        Serial.println("Encoder init failed.");
        return;
    }

    encoder.start(kEncoderTaskCore, kEncoderTaskPriority, 4096);

    if (!initEspNow()) {
        Serial.println("ESP-NOW init failed.");
        return;
    }

    WiFi.macAddress(gLocalMac);
    if (kDebugSerial) {
        Serial.printf("# Sender MAC: %02X:%02X:%02X:%02X:%02X:%02X ch=%u\n",
                      gLocalMac[0],
                      gLocalMac[1],
                      gLocalMac[2],
                      gLocalMac[3],
                      gLocalMac[4],
                      gLocalMac[5],
                      kEspNowChannel);
        Serial.printf("# Peer MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      kPeerMac[0],
                      kPeerMac[1],
                      kPeerMac[2],
                      kPeerMac[3],
                      kPeerMac[4],
                      kPeerMac[5]);
    }

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
    if (kDebugSerial) {
        static uint32_t lastReportMs = 0;
        uint32_t now = millis();
        if (now - lastReportMs >= 1000) {
            lastReportMs = now;
            Serial.printf("# ESP-NOW tx_ok=%lu tx_fail=%lu last_err=%d\n",
                          static_cast<unsigned long>(gTxOkCount),
                          static_cast<unsigned long>(gTxFailCount),
                          static_cast<int>(gLastSendErr));
        }
    }
    vTaskDelay(portMAX_DELAY);
}
