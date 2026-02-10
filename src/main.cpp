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

// Update with receiver MAC address (when not using broadcast).
constexpr uint8_t kPeerMac[6] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};
constexpr bool kUseBroadcastPeer = true;
constexpr uint8_t kBroadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
constexpr uint8_t kEspNowChannel = 1;
constexpr uint32_t kEspNowSendIntervalMs = 5;
constexpr uint32_t kEspNowHeartbeatMs = 1000;
constexpr bool kDebugSerial = true;
constexpr uint32_t kDebugPrintIntervalMs = 1000;
constexpr bool kUseRtosTasks = false;
constexpr uint32_t kSerialReadyDelayMs = 200;
constexpr uint8_t kEncoderTaskCore = 1;
constexpr UBaseType_t kEncoderTaskPriority = tskIDLE_PRIORITY + 2;
constexpr uint8_t kSendTaskCore = 0;

const uint8_t* getPeerMac() {
    return kUseBroadcastPeer ? kBroadcastMac : kPeerMac;
}

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
volatile uint32_t gTxQueueFailCount = 0;
volatile esp_err_t gLastChannelErr = ESP_OK;
volatile esp_err_t gLastInitErr = ESP_OK;
volatile esp_err_t gLastPeerErr = ESP_OK;
volatile esp_err_t gLastSendErr = ESP_OK;
volatile esp_now_send_status_t gLastSendStatus = ESP_NOW_SEND_FAIL;
uint8_t gLocalMac[6] = {};
uint8_t gCurrentChannel = 0;
wifi_second_chan_t gCurrentSecond = WIFI_SECOND_CHAN_NONE;
bool gEspNowReady = false;
bool gEncoderReady = false;

void onEspNowSend(const uint8_t* mac, esp_now_send_status_t status) {
    (void)mac;
    gLastSendStatus = status;
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
    gLastChannelErr = esp_wifi_set_channel(kEspNowChannel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_get_channel(&gCurrentChannel, &gCurrentSecond);
    if (gLastChannelErr != ESP_OK) {
        return false;
    }

    gLastInitErr = esp_now_init();
    if (gLastInitErr != ESP_OK) {
        return false;
    }

    esp_now_register_send_cb(onEspNowSend);

    esp_now_peer_info_t peerInfo = {};
    const uint8_t* peerMac = getPeerMac();
    memcpy(peerInfo.peer_addr, peerMac, 6);
    peerInfo.channel = kUseBroadcastPeer ? 0 : kEspNowChannel;
    peerInfo.ifidx = ESP_IF_WIFI_STA;
    peerInfo.encrypt = false;

    gLastPeerErr = esp_now_add_peer(&peerInfo);
    if (gLastPeerErr != ESP_OK && gLastPeerErr != ESP_ERR_ESPNOW_EXIST) {
        return false;
    }

    return true;
}

void updatePacketFromEncoder(uint32_t* lastSeq, bool* pending) {
    bool allOk = false;
    if (!encoder.copyIfNew(lastSeq, values, status, &allOk)) {
        return;
    }

    packet.seq = *lastSeq;
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
    *pending = true;
}

void trySendPacket(bool* pending, uint32_t* lastSendMs) {
    if (!gEspNowReady) {
        return;
    }

    uint32_t now = millis();
    bool dueSend =
        *pending &&
        (kEspNowSendIntervalMs == 0 || (now - *lastSendMs) >= kEspNowSendIntervalMs);
    bool dueHeartbeat =
        !*pending && kEspNowHeartbeatMs > 0 && (now - *lastSendMs) >= kEspNowHeartbeatMs;

    if (!dueSend && !dueHeartbeat) {
        return;
    }

    *lastSendMs = now;
    *pending = false;

    gLastSendErr = esp_now_send(getPeerMac(),
                                reinterpret_cast<const uint8_t*>(&packet),
                                sizeof(packet));
    if (gLastSendErr != ESP_OK) {
        gTxQueueFailCount++;
        gTxFailCount++;
        gLastSendStatus = ESP_NOW_SEND_FAIL;
    }
}

void taskEspNowSender(void* parameter) {
    uint32_t lastSeq = 0;
    uint32_t lastSendMs = 0;
    bool pending = false;

    while (true) {
        updatePacketFromEncoder(&lastSeq, &pending);
        trySendPacket(&pending, &lastSendMs);

        vTaskDelay(1);
    }
}
}  // namespace

void setup() {
    Serial.begin(2000000);
    delay(kSerialReadyDelayMs);

    gEncoderReady = encoder.begin(Serial2);
    if (!gEncoderReady) {
        Serial.println("Encoder init failed.");
    }

    if (gEncoderReady && kUseRtosTasks) {
        encoder.start(kEncoderTaskCore, kEncoderTaskPriority, 4096);
    }

    gEspNowReady = initEspNow();
    if (!gEspNowReady) {
        Serial.println("ESP-NOW init failed.");
    }

    memset(&packet, 0, sizeof(packet));
    packet.count = encoder.count();

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
        const uint8_t* peerMac = getPeerMac();
        Serial.printf("# Peer MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                      peerMac[0],
                      peerMac[1],
                      peerMac[2],
                      peerMac[3],
                      peerMac[4],
                      peerMac[5]);
        Serial.printf("# ESPNOW init=%d channel_err=%d peer_err=%d\n",
                      static_cast<int>(gLastInitErr),
                      static_cast<int>(gLastChannelErr),
                      static_cast<int>(gLastPeerErr));
        Serial.printf("# ESPNOW current_ch=%u broadcast=%u\n",
                      gCurrentChannel,
                      kUseBroadcastPeer ? 1 : 0);
    }

    if (gEspNowReady && kUseRtosTasks) {
        xTaskCreatePinnedToCore(
            taskEspNowSender,
            "EspNowSend",
            4096,
            nullptr,
            tskIDLE_PRIORITY + 1,
            nullptr,
            kSendTaskCore);
    }
}

void loop() {
    static uint32_t lastSeq = 0;
    static uint32_t lastSendMs = 0;
    static bool pending = false;

    if (!kUseRtosTasks) {
        if (gEncoderReady) {
            encoder.processOnce();
            updatePacketFromEncoder(&lastSeq, &pending);
        }
        trySendPacket(&pending, &lastSendMs);
    }

    if (kDebugSerial) {
        static uint32_t lastReportMs = 0;
        uint32_t now = millis();
        if (now - lastReportMs >= kDebugPrintIntervalMs) {
            lastReportMs = now;
            Serial.printf(
                "# ESP-NOW tx_ok=%lu tx_fail=%lu queue_fail=%lu last_err=%d "
                "last_status=%d\n",
                          static_cast<unsigned long>(gTxOkCount),
                          static_cast<unsigned long>(gTxFailCount),
                          static_cast<unsigned long>(gTxQueueFailCount),
                          static_cast<int>(gLastSendErr),
                          static_cast<int>(gLastSendStatus));
        }
    }
    vTaskDelay(1);
}
