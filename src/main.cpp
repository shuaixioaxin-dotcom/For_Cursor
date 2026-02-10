#define FLAGE 0

#if FLAGE==0              //master : 1   seriver:0

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
constexpr bool kForwardUartBinary = false;  // 设为 true 时通过 UART 转发二进制帧，
                                            // false 时仅在串口打印可读文本

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

// FIX: 将 esp_now_send (ACK) 移到临界区外。
// 原代码在 portENTER_CRITICAL 内调用 esp_now_send / esp_now_add_peer，
// 而 portENTER_CRITICAL 会禁用中断，WiFi 子系统的操作在此状态下可能
// 死锁或返回错误，导致 ACK 发送失败甚至影响后续数据接收。
void onEspNowReceive(const uint8_t* mac, const uint8_t* data, int len) {
    bool shouldSendAck = false;
    uint8_t ackMac[6];
    uint32_t ackSeq = 0;

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
                shouldSendAck = true;
                memcpy(ackMac, mac, 6);
                ackSeq = test.seq;
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

    // ACK 发送在临界区外执行，避免在中断禁用状态下调用 WiFi API
    if (shouldSendAck) {
        ensurePeer(ackMac);
        AckPacket ack = {kAckMagic, ackSeq};
        gLastAckErr = esp_now_send(ackMac,
                                   reinterpret_cast<const uint8_t*>(&ack),
                                   sizeof(ack));
        if (gLastAckErr == ESP_OK) {
            gAckSendCount++;
        } else {
            gAckSendFail++;
        }
    }
}

// FIX: 修复了 WiFi 初始化序列
// 原代码中 WiFi.mode(WIFI_STA) 之后又调用 esp_wifi_stop()/esp_wifi_start()，
// 这会破坏 Arduino WiFi 库的内部状态，导致后续 esp_wifi_set_channel() 失败。
// 正确做法：使用 WiFi.mode(WIFI_STA) + WiFi.disconnect() 即可，
// 不要手动 stop/start WiFi。
bool initEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.disconnect(false);  // FIX: 不擦除凭据，仅断开连接
    esp_wifi_set_ps(WIFI_PS_NONE);

    // FIX: 移除了 esp_wifi_stop()/esp_wifi_start() 调用
    // 这两个调用会破坏 Arduino WiFi 库已建立的内部状态，
    // 导致后续 esp_wifi_set_channel() 等调用失败。

    delay(10);  // FIX: 添加短暂延迟确保 WiFi 驱动完全就绪

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

void printEncoderPacket(const EncoderPacket& pkt) {
    Serial.printf(">> ENC seq=%lu cnt=%u all_ok=%u\n",
                  static_cast<unsigned long>(pkt.seq),
                  pkt.count,
                  pkt.all_ok);
    for (uint8_t i = 0; i < pkt.count; ++i) {
        Serial.printf("   encoder[%u]: value=%5u  status=%s\n",
                      i, pkt.values[i],
                      pkt.status[i] ? "OK" : "ERR");
    }
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
        if (kForwardUartBinary) {
            sendPacketUart(local);
        }
        printEncoderPacket(local);
    }

    if (kDebugSerial) {
        uint32_t now = millis();
        if (now - lastReportMs >= 1000) {
            lastReportMs = now;
            Serial.printf(
                "-- STAT rx_total=%lu enc_ok=%lu bad_len=%lu last_len=%d from=%02X:%02X:%02X:%02X:%02X:%02X\n",
                static_cast<unsigned long>(rxCount),
                static_cast<unsigned long>(rxValid),
                static_cast<unsigned long>(rxBadLen),
                lastLen,
                lastMac[0],
                lastMac[1],
                lastMac[2],
                lastMac[3],
                lastMac[4],
                lastMac[5]);
        }
    }

    vTaskDelay(1);
}

#else

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <cstring>
#include <esp_err.h>

#include "MultiEncoder.h"

namespace {
constexpr uint8_t kEncoderIds[] = {1, 2, 3, 4, 5, 6};
constexpr uint8_t kEncoderCount = sizeof(kEncoderIds) / sizeof(kEncoderIds[0]);

constexpr int kRs485RxPin = 32;
constexpr int kRs485TxPin = 33;
constexpr int kRs485DeRePin = 25;
constexpr uint32_t kRs485Baudrate = 2500000;

// Update with receiver MAC address (when not using broadcast).
constexpr uint8_t kPeerMac[6] = {0xA4, 0xF0, 0x0F, 0x1F, 0x04, 0xA0};
constexpr bool kUseBroadcastPeer = true;
constexpr uint8_t kBroadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
constexpr uint8_t kEspNowChannel = 1;
constexpr uint32_t kEspNowSendIntervalMs = 5;
constexpr uint32_t kEspNowHeartbeatMs = 1000;
constexpr bool kDebugSerial = true;
constexpr uint32_t kDebugPrintIntervalMs = 1000;
constexpr bool kUseRtosTasks = false;
constexpr bool kTestOnly = false;
constexpr uint32_t kTestIntervalMs = 100;
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

EncoderPacket packet;
uint16_t values[MultiEncoder::kMaxEncoders];
bool status[MultiEncoder::kMaxEncoders];
TestPacket testPacket = {kTestMagic, 0, 0};

volatile uint32_t gTxOkCount = 0;
volatile uint32_t gTxFailCount = 0;
volatile uint32_t gTxQueueFailCount = 0;
volatile uint32_t gTxAttemptCount = 0;
volatile uint32_t gAckRxCount = 0;
volatile uint32_t gLastAckSeq = 0;
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

void onEspNowReceive(const uint8_t* mac, const uint8_t* data, int len) {
    (void)mac;
    if (len == static_cast<int>(sizeof(AckPacket))) {
        AckPacket ack;
        memcpy(&ack, data, sizeof(ack));
        if (ack.magic == kAckMagic) {
            gAckRxCount++;
            gLastAckSeq = ack.seq;
        }
    }
}

// FIX: 修复了 WiFi 初始化序列
// 原代码中 WiFi.mode(WIFI_STA) 之后又调用 esp_wifi_stop()/esp_wifi_start()，
// 这会破坏 Arduino WiFi 库的内部状态，导致 esp_wifi_set_channel() 失败。
// 发送端检查了 gLastChannelErr，失败则 return false，使 gEspNowReady = false，
// 从而导致 trySendPacket() 永远在入口处直接返回，attempt 始终为 0。
bool initEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.disconnect(false);  // FIX: 不擦除凭据，仅断开连接
    esp_wifi_set_ps(WIFI_PS_NONE);

    // FIX: 移除了 esp_wifi_stop()/esp_wifi_start() 调用
    // 原代码在 WiFi.mode(WIFI_STA) 已启动 WiFi 后，又手动 stop/start，
    // 破坏了 Arduino WiFi 库的内部状态管理，导致 set_channel 失败。

    delay(10);  // FIX: 添加短暂延迟确保 WiFi 驱动完全就绪

    gLastChannelErr = esp_wifi_set_channel(kEspNowChannel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_get_channel(&gCurrentChannel, &gCurrentSecond);
    if (gLastChannelErr != ESP_OK) {
        if (kDebugSerial) {
            Serial.printf("# ERROR: esp_wifi_set_channel failed: %d (%s)\n",
                          static_cast<int>(gLastChannelErr),
                          esp_err_to_name(gLastChannelErr));
        }
        return false;
    }

    gLastInitErr = esp_now_init();
    if (gLastInitErr != ESP_OK) {
        if (kDebugSerial) {
            Serial.printf("# ERROR: esp_now_init failed: %d (%s)\n",
                          static_cast<int>(gLastInitErr),
                          esp_err_to_name(gLastInitErr));
        }
        return false;
    }

    esp_now_register_send_cb(onEspNowSend);
    esp_now_register_recv_cb(onEspNowReceive);

    esp_now_peer_info_t peerInfo = {};
    const uint8_t* peerMac = getPeerMac();
    memcpy(peerInfo.peer_addr, peerMac, 6);
    // FIX: 广播 peer 也应该指定信道，确保在正确的信道上发送
    // 原代码中 broadcast 时 channel=0 表示使用当前接口信道，
    // 但如果接口信道未正确设置，会导致通信失败。
    // 显式设置信道更加可靠。
    peerInfo.channel = kEspNowChannel;
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = false;

    gLastPeerErr = esp_now_add_peer(&peerInfo);
    if (gLastPeerErr != ESP_OK && gLastPeerErr != ESP_ERR_ESPNOW_EXIST) {
        if (kDebugSerial) {
            Serial.printf("# ERROR: esp_now_add_peer failed: %d (%s)\n",
                          static_cast<int>(gLastPeerErr),
                          esp_err_to_name(gLastPeerErr));
        }
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

    gTxAttemptCount++;
    if (kTestOnly) {
        gLastSendErr = esp_now_send(getPeerMac(),
                                    reinterpret_cast<const uint8_t*>(&testPacket),
                                    sizeof(testPacket));
    } else {
        gLastSendErr = esp_now_send(getPeerMac(),
                                    reinterpret_cast<const uint8_t*>(&packet),
                                    sizeof(packet));
    }
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

    if (!kTestOnly) {
        gEncoderReady = encoder.begin(Serial2);
    } else {
        gEncoderReady = false;
    }
    if (!gEncoderReady) {
        if (!kTestOnly) {
            Serial.println("Encoder init failed.");
        }
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
        Serial.printf("# ESPNOW test_only=%u espnow_ready=%u\n",
                      kTestOnly ? 1 : 0,
                      gEspNowReady ? 1 : 0);
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
        if (kTestOnly) {
            uint32_t now = millis();
            if (now - lastSendMs >= kTestIntervalMs) {
                testPacket.seq++;
                testPacket.ms = now;
                pending = true;
            }
        }
        trySendPacket(&pending, &lastSendMs);
    }

    if (kDebugSerial) {
        static uint32_t lastReportMs = 0;
        uint32_t now = millis();
        if (now - lastReportMs >= kDebugPrintIntervalMs) {
            lastReportMs = now;
            Serial.printf(
                "-- TX tx_ok=%lu tx_fail=%lu attempt=%lu send_err=%d(%s) espnow_ready=%u enc_ready=%u\n",
                          static_cast<unsigned long>(gTxOkCount),
                          static_cast<unsigned long>(gTxFailCount),
                          static_cast<unsigned long>(gTxAttemptCount),
                          static_cast<int>(gLastSendErr),
                          esp_err_to_name(static_cast<esp_err_t>(gLastSendErr)),
                          gEspNowReady ? 1 : 0,
                          gEncoderReady ? 1 : 0);

            // 打印发送端本地编码器数据，便于确认编码器硬件是否正常工作
            if (!kTestOnly) {
                Serial.printf(">> ENC-TX seq=%lu cnt=%u all_ok=%u\n",
                              static_cast<unsigned long>(packet.seq),
                              packet.count,
                              packet.all_ok);
                for (uint8_t i = 0; i < packet.count; ++i) {
                    Serial.printf("   encoder[%u]: value=%5u  status=%s\n",
                                  i, packet.values[i],
                                  packet.status[i] ? "OK" : "ERR");
                }
            }
        }
    }
    vTaskDelay(1);
}
#endif
