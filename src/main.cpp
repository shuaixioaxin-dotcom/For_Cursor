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

bool initEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.disconnect(true, true);
    esp_wifi_set_channel(kEspNowChannel, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        return false;
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, kPeerMac, sizeof(kPeerMac));
    peerInfo.channel = kEspNowChannel;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
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

    encoder.start(0, configMAX_PRIORITIES - 1, 4096);

    if (!initEspNow()) {
        Serial.println("ESP-NOW init failed.");
        return;
    }

    xTaskCreatePinnedToCore(
        taskEspNowSender,
        "EspNowSend",
        4096,
        nullptr,
        1,
        nullptr,
        1);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
