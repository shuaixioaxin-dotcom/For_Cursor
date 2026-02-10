#include "MultiEncoder.h"

#include <cstring>
#include <soc/gpio_struct.h>

MultiEncoder::MultiEncoder(const Config& config)
    : serial_(nullptr),
      numEncoders_(config.numEncoders),
      rxPin_(config.rxPin),
      txPin_(config.txPin),
      deRePin_(config.deRePin),
      baudrate_(config.baudrate),
      responseTimeoutUs_(config.responseTimeoutUs),
      interByteTimeoutUs_(config.interByteTimeoutUs),
      txTurnaroundUs_(config.txTurnaroundUs),
      serialRxBufferSize_(config.serialRxBufferSize),
      serialTxBufferSize_(config.serialTxBufferSize),
      idleDelayUs_(config.idleDelayUs),
      yieldEveryBatches_(config.yieldEveryBatches),
      yieldDelayTicks_(config.yieldDelayTicks),
      batchCounter_(0),
      writeIdx_(0),
      readIdx_(1),
      sequence_(0),
      started_(false),
      configOk_(false),
      taskHandle_(nullptr),
      deReHighBank_(false),
      deReMask_(0) {
    configOk_ = (config.encoderIds != nullptr) && (config.numEncoders > 0) &&
                (config.numEncoders <= kMaxEncoders);

    if (configOk_) {
        for (uint8_t i = 0; i < numEncoders_; ++i) {
            encoderIds_[i] = config.encoderIds[i];
        }
    } else {
        numEncoders_ = 0;
    }

    if (responseTimeoutUs_ == 0) {
        responseTimeoutUs_ = 300;
    }
    if (interByteTimeoutUs_ == 0) {
        interByteTimeoutUs_ = 80;
    }
    if (txTurnaroundUs_ == 0) {
        txTurnaroundUs_ = 32;
    }
    if (serialRxBufferSize_ == 0) {
        serialRxBufferSize_ = 256;
    }
    if (serialTxBufferSize_ == 0) {
        serialTxBufferSize_ = 256;
    }
    if (yieldEveryBatches_ == 0) {
        yieldEveryBatches_ = 64;
    }
    if (yieldDelayTicks_ == 0) {
        yieldDelayTicks_ = 1;
    }

    if (deRePin_ >= 0) {
        if (deRePin_ < 32) {
            deReHighBank_ = false;
            deReMask_ = (1UL << deRePin_);
        } else {
            deReHighBank_ = true;
            deReMask_ = (1UL << (deRePin_ - 32));
        }
    }
}

bool MultiEncoder::begin(HardwareSerial& serial) {
    if (!configOk_) {
        return false;
    }

    serial_ = &serial;

    pinMode(deRePin_, OUTPUT);
    digitalWrite(deRePin_, LOW);

    serial_->setRxBufferSize(serialRxBufferSize_);
    serial_->setTxBufferSize(serialTxBufferSize_);
    serial_->begin(baudrate_, SERIAL_8N1, rxPin_, txPin_);

    for (uint8_t i = 0; i < numEncoders_; ++i) {
        requestFrames_[i][0] = encoderIds_[i];
        requestFrames_[i][1] = 0x03;
        requestFrames_[i][2] = 0x00;
        requestFrames_[i][3] = 0x01;
        requestFrames_[i][4] = 0x00;
        requestFrames_[i][5] = 0x01;
        uint16_t crc = calculateCRC(requestFrames_[i], 6);
        requestFrames_[i][6] = crc & 0xFF;
        requestFrames_[i][7] = (crc >> 8) & 0xFF;
    }

    for (int i = 0; i < 2; ++i) {
        buffers_[i].all_ok = false;
        buffers_[i].seq = 0;
        for (uint8_t j = 0; j < kMaxEncoders; ++j) {
            buffers_[i].values[j] = 0;
            buffers_[i].status[j] = 0;
        }
    }

    return true;
}

void MultiEncoder::start(uint8_t core, UBaseType_t priority, uint32_t stackSize) {
    if (started_ || serial_ == nullptr) {
        return;
    }

    BaseType_t result = xTaskCreatePinnedToCore(
        taskTrampoline,
        "MultiEnc",
        stackSize,
        this,
        priority,
        &taskHandle_,
        core);

    if (result == pdPASS) {
        started_ = true;
    }
}

uint8_t MultiEncoder::count() const {
    return numEncoders_;
}

uint32_t MultiEncoder::currentSequence() const {
    return buffers_[readIdx_].seq;
}

bool MultiEncoder::copyLatest(uint16_t* values, bool* status, bool* allOk,
                              uint32_t* seq) const {
    if (numEncoders_ == 0) {
        return false;
    }

    for (int attempt = 0; attempt < 3; ++attempt) {
        uint8_t idx = readIdx_;
        const EncoderData& data = buffers_[idx];
        uint32_t seqStart = data.seq;

        if (values != nullptr) {
            memcpy(values, data.values, sizeof(uint16_t) * numEncoders_);
        }
        if (status != nullptr) {
            for (uint8_t i = 0; i < numEncoders_; ++i) {
                status[i] = data.status[i] != 0;
            }
        }
        if (allOk != nullptr) {
            *allOk = data.all_ok;
        }

        uint32_t seqEnd = data.seq;
        if (idx == readIdx_ && seqStart == seqEnd) {
            if (seq != nullptr) {
                *seq = seqEnd;
            }
            return true;
        }
    }

    return false;
}

bool MultiEncoder::copyIfNew(uint32_t* lastSeq, uint16_t* values, bool* status,
                             bool* allOk) {
    uint32_t current = currentSequence();
    if (lastSeq != nullptr && current == *lastSeq) {
        return false;
    }

    uint32_t seq = 0;
    if (!copyLatest(values, status, allOk, &seq)) {
        return false;
    }

    if (lastSeq != nullptr) {
        *lastSeq = seq;
    }
    return true;
}

void MultiEncoder::processOnce() {
    if (serial_ == nullptr) {
        return;
    }
    processBatch();
}

void MultiEncoder::taskTrampoline(void* arg) {
    MultiEncoder* self = static_cast<MultiEncoder*>(arg);
    self->taskLoop();
}

void MultiEncoder::taskLoop() {
    while (true) {
        processBatch();
        ++batchCounter_;
        if (yieldEveryBatches_ > 0 && batchCounter_ >= yieldEveryBatches_) {
            batchCounter_ = 0;
            vTaskDelay(yieldDelayTicks_);
            continue;
        }
        if (idleDelayUs_ > 0) {
            delayMicroseconds(idleDelayUs_);
        }
    }
}

void MultiEncoder::processBatch() {
    EncoderData& data = buffers_[writeIdx_];
    bool currentBatchOk = true;

    for (uint8_t i = 0; i < numEncoders_; ++i) {
        setTransmitMode();
        serial_->write(requestFrames_[i], 8);
        delayMicroseconds(txTurnaroundUs_);
        setReceiveMode();

        uint8_t response[7];
        int len = readBytesFast(response, 7, responseTimeoutUs_, interByteTimeoutUs_);

        if (len == 7 && response[0] == encoderIds_[i] && response[1] == 0x03) {
            data.values[i] = (static_cast<uint16_t>(response[3]) << 8) | response[4];
            data.status[i] = 1;
        } else {
            data.status[i] = 0;
            currentBatchOk = false;
            for (uint8_t clearCount = 0; serial_->available() && clearCount < 16;
                 ++clearCount) {
                serial_->read();
            }
        }
    }

    data.all_ok = currentBatchOk;
    data.seq = ++sequence_;

    uint8_t oldWrite = writeIdx_;
    writeIdx_ = readIdx_;
    readIdx_ = oldWrite;
}

void MultiEncoder::setTransmitMode() {
    if (deReHighBank_) {
        GPIO.out1_w1ts.val = deReMask_;
    } else {
        GPIO.out_w1ts = deReMask_;
    }
}

void MultiEncoder::setReceiveMode() {
    if (deReHighBank_) {
        GPIO.out1_w1tc.val = deReMask_;
    } else {
        GPIO.out_w1tc = deReMask_;
    }
}

int MultiEncoder::readBytesFast(uint8_t* buffer, int length,
                                uint32_t responseTimeoutUs,
                                uint32_t interByteTimeoutUs) {
    int count = 0;
    uint32_t start = micros();

    while (!serial_->available()) {
        if (micros() - start > responseTimeoutUs) {
            return 0;
        }
    }

    while (count < length) {
        int available = serial_->available();
        if (available > 0) {
            int remaining = length - count;
            int toRead = available < remaining ? available : remaining;
            for (int i = 0; i < toRead; ++i) {
                buffer[count++] = serial_->read();
            }
            start = micros();
        } else {
            if (micros() - start > interByteTimeoutUs) {
                break;
            }
        }
    }

    return count;
}

uint16_t MultiEncoder::calculateCRC(uint8_t* buf, int len) {
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; ++pos) {
        crc ^= static_cast<uint16_t>(buf[pos]);
        for (int i = 8; i != 0; --i) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
