#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <cstring>

namespace {
constexpr uint32_t kUartBaudrate = 2000000;
constexpr uint8_t kEspNowChannel = 1;
constexpr bool kDebugSerial = true;
constexpr uint32_t kSerialReadyDelayMs = 200;
constexpr size_t kDebugPayloadBytes = 8;

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
volatile uint32_t gRxValid = 0;
volatile uint32_t gRxBadLen = 0;
volatile uint32_t gLastRxMs = 0;
volatile int gLastLen = 0;
uint8_t gLastMac[6] = {};
uint8_t gLocalMac[6] = {};
uint8_t gLastPayload[kDebugPayloadBytes] = {};
volatile esp_err_t gLastChannelErr = ESP_OK;
volatile esp_err_t gLastInitErr = ESP_OK;
uint8_t gCurrentChannel = 0;
wifi_second_chan_t gCurrentSecond = WIFI_SECOND_CHAN_NONE;
portMUX_TYPE gPacketMux = portMUX_INITIALIZER_UNLOCKED;

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
    gLastChannelErr = esp_wifi_set_channel(kEspNowChannel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_get_channel(&gCurrentChannel, &gCurrentSecond);

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
                "# ESP-NOW rx=%lu ok=%lu bad=%lu last_ms=%lu len=%d mac=%02X:%02X:%02X:%02X:%02X:%02X payload=%02X %02X %02X %02X %02X %02X %02X %02X\n",
                static_cast<unsigned long>(rxCount),
                static_cast<unsigned long>(rxValid),
                static_cast<unsigned long>(rxBadLen),
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
                lastPayload[7]);
        }
    }

    vTaskDelay(1);
}
