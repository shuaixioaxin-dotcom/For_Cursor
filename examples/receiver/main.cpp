#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <cstring>
#include <esp_err.h>

namespace {
constexpr uint32_t kUartBaudrate = 2000000;
constexpr uint8_t kEspNowChannel = 1;
constexpr bool kDebugSerial = true;
constexpr uint32_t kSerialReadyDelayMs = 200;
constexpr size_t kDebugPayloadBytes = 8;
constexpr bool kSendAck = true;

constexpr uint8_t kMaxEncoders = 32;
constexpr uint16_t kUartMagic = 0xA55A;

struct EncoderPacket {
    uint32_t seq;
    uint8_t count;
    uint8_t all_ok;
    uint16_t values[kMaxEncoders];
    uint8_t status[kMaxEncoders];
} __attribute__((packed));

struct TestPacket {
    uint32_t magic;
    uint32_t seq;
    uint32_t ms;
} __attribute__((packed));

struct AckPacket {
    uint32_t magic;
    uint32_t seq;
} __attribute__((packed));

constexpr uint32_t kTestMagic = 0x454E4F57;  // "ENOW"
constexpr uint32_t kAckMagic = 0x41434B30;   // "ACK0"

struct UartFrameHeader {
    uint16_t magic;
    uint16_t length;
} __attribute__((packed));

EncoderPacket gPacket;
volatile bool gPacketReady = false;
volatile uint32_t gRxCount = 0;
volatile uint32_t gRxValid = 0;
volatile uint32_t gRxBadLen = 0;
volatile uint32_t gTestRxCount = 0;
volatile uint32_t gLastTestSeq = 0;
volatile uint32_t gAckSendCount = 0;
volatile uint32_t gAckSendFail = 0;
volatile uint32_t gLastRxMs = 0;
volatile int gLastLen = 0;
uint8_t gLastMac[6] = {};
uint8_t gLocalMac[6] = {};
uint8_t gLastPayload[kDebugPayloadBytes] = {};
volatile esp_err_t gLastChannelErr = ESP_OK;
volatile esp_err_t gLastInitErr = ESP_OK;
uint8_t gCurrentChannel = 0;
wifi_second_chan_t gCurrentSecond = WIFI_SECOND_CHAN_NONE;
volatile esp_err_t gLastAckErr = ESP_OK;
volatile esp_err_t gLastWifiStopErr = ESP_OK;
volatile esp_err_t gLastWifiStartErr = ESP_OK;
volatile esp_err_t gLastDeinitErr = ESP_OK;
portMUX_TYPE gPacketMux = portMUX_INITIALIZER_UNLOCKED;

bool ensurePeer(const uint8_t* mac) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = false;
    if (kEspNowChannel > 0) {
        peerInfo.channel = kEspNowChannel;
    } else {
        peerInfo.channel = 0;
    }
    esp_err_t err = esp_now_add_peer(&peerInfo);
    return (err == ESP_OK || err == ESP_ERR_ESPNOW_EXIST);
}

void onEspNowReceive(const uint8_t* mac, const uint8_t* data, int len) {
    portENTER_CRITICAL(&gPacketMux);
    gRxCount++;
    gLastRxMs = millis();
    gLastLen = len;
    memcpy(gLastMac, mac, sizeof(gLastMac));
    size_t copyLen = len > static_cast<int>(kDebugPayloadBytes) ? kDebugPayloadBytes
                                                                 : static_cast<size_t>(len);
    if (copyLen > 0) {
        memcpy(gLastPayload, data, copyLen);
    }
    if (len == static_cast<int>(sizeof(TestPacket))) {
        TestPacket test;
        memcpy(&test, data, sizeof(test));
        if (test.magic == kTestMagic) {
            gTestRxCount++;
            gLastTestSeq = test.seq;
            if (kSendAck) {
                ensurePeer(mac);
                AckPacket ack = {kAckMagic, test.seq};
                gLastAckErr = esp_now_send(mac,
                                           reinterpret_cast<const uint8_t*>(&ack),
                                           sizeof(ack));
                if (gLastAckErr == ESP_OK) {
                    gAckSendCount++;
                } else {
                    gAckSendFail++;
                }
            }
        }
    }

    if (len == static_cast<int>(sizeof(EncoderPacket))) {
        memcpy(&gPacket, data, sizeof(EncoderPacket));
        gPacketReady = true;
        gRxValid++;
    } else {
        gRxBadLen++;
    }
    portEXIT_CRITICAL(&gPacketMux);
}

bool initEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.disconnect(true, true);
    esp_wifi_set_ps(WIFI_PS_NONE);
    gLastWifiStopErr = esp_wifi_stop();
    gLastWifiStartErr = esp_wifi_start();
    gLastChannelErr = esp_wifi_set_channel(kEspNowChannel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_get_channel(&gCurrentChannel, &gCurrentSecond);

    gLastDeinitErr = esp_now_deinit();
    gLastInitErr = esp_now_init();
    if (gLastInitErr != ESP_OK) {
        return false;
    }

    esp_now_register_recv_cb(onEspNowReceive);
    return true;
}

void sendPacketUart(const EncoderPacket& packet) {
    UartFrameHeader header = {kUartMagic, static_cast<uint16_t>(sizeof(EncoderPacket))};
    Serial.write(reinterpret_cast<const uint8_t*>(&header), sizeof(header));
    Serial.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
}
}  // namespace

void setup() {
    Serial.begin(kUartBaudrate);
    Serial.setRxBufferSize(512);
    Serial.setTxBufferSize(512);
    delay(kSerialReadyDelayMs);

    if (!initEspNow()) {
        Serial.println("ESP-NOW init failed.");
    }

    WiFi.macAddress(gLocalMac);
    if (kDebugSerial) {
        Serial.printf("# Receiver MAC: %02X:%02X:%02X:%02X:%02X:%02X ch=%u\n",
                      gLocalMac[0],
                      gLocalMac[1],
                      gLocalMac[2],
                      gLocalMac[3],
                      gLocalMac[4],
                      gLocalMac[5],
                      kEspNowChannel);
        Serial.printf("# ESPNOW init=%d channel_err=%d current_ch=%u\n",
                      static_cast<int>(gLastInitErr),
                      static_cast<int>(gLastChannelErr),
                      gCurrentChannel);
        Serial.printf("# WIFI stop=%d start=%d deinit=%d\n",
                      static_cast<int>(gLastWifiStopErr),
                      static_cast<int>(gLastWifiStartErr),
                      static_cast<int>(gLastDeinitErr));
    }
}

void loop() {
    EncoderPacket local;
    bool hasPacket = false;
    uint32_t rxCount = 0;
    uint32_t rxValid = 0;
    uint32_t rxBadLen = 0;
    uint32_t lastRxMs = 0;
    int lastLen = 0;
    uint8_t lastMac[6] = {};
    uint8_t lastPayload[kDebugPayloadBytes] = {};
    static uint32_t lastReportMs = 0;

    portENTER_CRITICAL(&gPacketMux);
    if (gPacketReady) {
        local = gPacket;
        gPacketReady = false;
        hasPacket = true;
    }
    rxCount = gRxCount;
    rxValid = gRxValid;
    rxBadLen = gRxBadLen;
    lastRxMs = gLastRxMs;
    lastLen = gLastLen;
    memcpy(lastMac, gLastMac, sizeof(lastMac));
    memcpy(lastPayload, gLastPayload, sizeof(lastPayload));
    portEXIT_CRITICAL(&gPacketMux);

    if (hasPacket) {
        sendPacketUart(local);
    }

    if (kDebugSerial) {
        uint32_t now = millis();
        if (now - lastReportMs >= 1000) {
            lastReportMs = now;
            Serial.printf(
                "# ESP-NOW rx=%lu ok=%lu bad=%lu test=%lu ack_ok=%lu ack_fail=%lu last_ms=%lu len=%d mac=%02X:%02X:%02X:%02X:%02X:%02X payload=%02X %02X %02X %02X %02X %02X %02X %02X ack_err=%d\n",
                static_cast<unsigned long>(rxCount),
                static_cast<unsigned long>(rxValid),
                static_cast<unsigned long>(rxBadLen),
                static_cast<unsigned long>(gTestRxCount),
                static_cast<unsigned long>(gAckSendCount),
                static_cast<unsigned long>(gAckSendFail),
                static_cast<unsigned long>(lastRxMs),
                lastLen,
                lastMac[0],
                lastMac[1],
                lastMac[2],
                lastMac[3],
                lastMac[4],
                lastMac[5],
                lastPayload[0],
                lastPayload[1],
                lastPayload[2],
                lastPayload[3],
                lastPayload[4],
                lastPayload[5],
                lastPayload[6],
                lastPayload[7],
                static_cast<int>(gLastAckErr));
        }
    }

    vTaskDelay(1);
}
