#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <cstring>

namespace {
constexpr int kUartRxPin = 40;
constexpr int kUartTxPin = 41;
constexpr uint32_t kUartBaudrate = 2000000;

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
portMUX_TYPE gPacketMux = portMUX_INITIALIZER_UNLOCKED;

void onEspNowReceive(const uint8_t* mac, const uint8_t* data, int len) {
    (void)mac;
    if (len != static_cast<int>(sizeof(EncoderPacket))) {
        return;
    }

    portENTER_CRITICAL_ISR(&gPacketMux);
    memcpy(&gPacket, data, sizeof(EncoderPacket));
    gPacketReady = true;
    portEXIT_CRITICAL_ISR(&gPacketMux);
}

bool initEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.disconnect(true, true);

    if (esp_now_init() != ESP_OK) {
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
    Serial.begin(kUartBaudrate, SERIAL_8N1, kUartRxPin, kUartTxPin);
    Serial.setRxBufferSize(512);
    Serial.setTxBufferSize(512);

    if (!initEspNow()) {
        Serial.println("ESP-NOW init failed.");
    }
}

void loop() {
    EncoderPacket local;
    bool hasPacket = false;

    portENTER_CRITICAL(&gPacketMux);
    if (gPacketReady) {
        local = gPacket;
        gPacketReady = false;
        hasPacket = true;
    }
    portEXIT_CRITICAL(&gPacketMux);

    if (hasPacket) {
        sendPacketUart(local);
    }

    vTaskDelay(1);
}
