#pragma once

#include <Arduino.h>

class MultiEncoder {
public:
    static constexpr uint8_t kMaxEncoders = 32;

    struct Config {
        uint8_t numEncoders;
        const uint8_t* encoderIds;
        int rxPin;
        int txPin;
        int deRePin;
        uint32_t baudrate;
        uint32_t responseTimeoutUs;
        uint32_t interByteTimeoutUs;
        uint32_t txTurnaroundUs;
        size_t serialRxBufferSize;
        size_t serialTxBufferSize;
        uint32_t idleDelayUs;
        uint16_t yieldEveryBatches;
        uint16_t yieldDelayTicks;
    };

    explicit MultiEncoder(const Config& config);

    bool begin(HardwareSerial& serial);
    void start(uint8_t core, UBaseType_t priority, uint32_t stackSize);

    uint8_t count() const;
    uint32_t currentSequence() const;

    bool copyLatest(uint16_t* values, bool* status, bool* allOk,
                    uint32_t* seq) const;
    bool copyIfNew(uint32_t* lastSeq, uint16_t* values, bool* status,
                   bool* allOk);

private:
    struct EncoderData {
        volatile uint32_t seq;
        volatile bool all_ok;
        volatile uint16_t values[kMaxEncoders];
        volatile uint8_t status[kMaxEncoders];
    };

    static void taskTrampoline(void* arg);
    void taskLoop();
    void processBatch();
    void setTransmitMode();
    void setReceiveMode();
    int readBytesFast(uint8_t* buffer, int length, uint32_t responseTimeoutUs,
                      uint32_t interByteTimeoutUs);
    static uint16_t calculateCRC(uint8_t* buf, int len);

    HardwareSerial* serial_;
    uint8_t numEncoders_;
    int rxPin_;
    int txPin_;
    int deRePin_;
    uint32_t baudrate_;
    uint32_t responseTimeoutUs_;
    uint32_t interByteTimeoutUs_;
    uint32_t txTurnaroundUs_;
    size_t serialRxBufferSize_;
    size_t serialTxBufferSize_;
    uint32_t idleDelayUs_;
    uint16_t yieldEveryBatches_;
    uint16_t yieldDelayTicks_;
    uint16_t batchCounter_;

    volatile uint8_t writeIdx_;
    volatile uint8_t readIdx_;
    volatile uint32_t sequence_;

    bool started_;
    bool configOk_;
    TaskHandle_t taskHandle_;

    uint8_t encoderIds_[kMaxEncoders];
    uint8_t requestFrames_[kMaxEncoders][8];
    EncoderData buffers_[2];

    bool deReHighBank_;
    uint32_t deReMask_;
};
