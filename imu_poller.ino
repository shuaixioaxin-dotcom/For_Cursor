#include <Arduino.h>

// ================= Pin Configuration =================
#define RS485_RX_PIN 32     // Module RO
#define RS485_TX_PIN 33     // Module DI
#define RS485_DE_RE_PIN 25  // Module DE/RE Control

// ================= Serial Configuration =================
static const uint32_t SERIAL_BAUD = 2000000;
static const uint32_t RS485_BAUD = 921600;

// ================= IMU Configuration =================
#define NUM_IMUS 2
const uint8_t IMU_IDS[NUM_IMUS] = {1, 2};
const uint16_t IMU_POLL_INTERVAL_MS[NUM_IMUS] = {0, 0};

// ================= Modbus RTU Configuration =================
static const uint8_t MODBUS_FUNC_READ_HREG = 0x03;
static const uint16_t MODBUS_REG_START = 0x0034;
static const uint16_t MODBUS_REG_COUNT = 0x0016;
static const uint8_t MODBUS_REQUEST_LEN = 8;
static const uint16_t RESPONSE_BYTE_COUNT = MODBUS_REG_COUNT * 2;
static const uint16_t RESPONSE_LEN = 1 + 1 + 1 + RESPONSE_BYTE_COUNT + 2;
static const uint16_t RESPONSE_TIMEOUT_MS = 100;
static const uint32_t BUS_SILENCE_US = 1000;
static const uint32_t TX_ENABLE_DELAY_US = 30;
static const uint32_t TX_DISABLE_DELAY_US = 60;
static const uint32_t INTER_BYTE_TIMEOUT_US = 2000;

// ================= Reliability Tuning =================
static const uint8_t MAX_RETRIES = 1;
static const uint16_t RETRY_DELAY_MS = 2;
static const uint16_t MIN_POLL_GAP_MS = 2;

// ================= Data Conversion =================
static const float ACC_SCALE = 0.0048828f;
static const float QUAT_SCALE = 0.0001f;

// ================= Frequency Monitoring =================
static const uint32_t FREQ_REPORT_INTERVAL_MS = 1000;

// ================= State Machine & Scheduling =================
enum State {
  STATE_IDLE,
  STATE_SENDING_REQUEST,
  STATE_WAITING_RESPONSE,
  STATE_PROCESSING_DATA
};

enum ParseResult {
  PARSE_OK = 0,
  PARSE_BAD_LEN,
  PARSE_BAD_HEADER,
  PARSE_BAD_COUNT,
  PARSE_BAD_CRC,
  PARSE_BAD_BOUNDS
};

struct ImuData {
  float acc[3];
  float quat[4];
  bool valid;
  uint32_t last_update_ms;
  uint32_t next_poll_ms;
  uint8_t fail_streak;
  uint8_t retries_left;
};

ImuData imu_data[NUM_IMUS];
State current_state = STATE_IDLE;
uint8_t current_imu_index = 0;
uint32_t state_start_ms = 0;
uint8_t response_buf[RESPONSE_LEN];
uint16_t response_pos = 0;
uint32_t last_rx_us = 0;

// Statistics
uint32_t cycle_count = 0;
uint32_t success_count = 0;
uint32_t fail_count = 0;
uint32_t timeout_count = 0;
uint32_t crc_error_count = 0;
uint32_t retry_count = 0;
uint32_t last_bus_activity_us = 0;
uint32_t freq_start_ms = 0;

// Debug flags
static const bool DEBUG_SCHEDULING = false;
static const bool DEBUG_RAW_RESPONSES = false;
static const bool DEBUG_PARSING = false;

// ================= Utility Functions =================
static uint16_t crc16_modbus(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; ++j) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

static void clearRxBuffer() {
  while (Serial2.available() > 0) {
    Serial2.read();
  }
}

static bool busIsIdle() {
  return (micros() - last_bus_activity_us) >= BUS_SILENCE_US;
}

static void zeroImu(ImuData &data) {
  data.acc[0] = 0.0f; data.acc[1] = 0.0f; data.acc[2] = 0.0f;
  data.quat[0] = 0.0f; data.quat[1] = 0.0f; data.quat[2] = 0.0f; data.quat[3] = 0.0f;
  data.valid = false;
}

static uint32_t successIntervalMs(uint8_t idx) {
  uint16_t user_interval = IMU_POLL_INTERVAL_MS[idx];
  uint16_t base_interval = (user_interval > MIN_POLL_GAP_MS) ? user_interval : MIN_POLL_GAP_MS;
  return base_interval;
}

static void buildRequest(uint8_t slave_id, uint8_t *out) {
  out[0] = slave_id;
  out[1] = MODBUS_FUNC_READ_HREG;
  out[2] = (MODBUS_REG_START >> 8) & 0xFF;
  out[3] = MODBUS_REG_START & 0xFF;
  out[4] = (MODBUS_REG_COUNT >> 8) & 0xFF;
  out[5] = MODBUS_REG_COUNT & 0xFF;
  uint16_t crc = crc16_modbus(out, 6);
  out[6] = crc & 0xFF;
  out[7] = (crc >> 8) & 0xFF;
}

// ================= Core Logic Functions =================

void sendRequest(uint8_t slave_id) {
  uint8_t request[MODBUS_REQUEST_LEN];
  buildRequest(slave_id, request);

  if (DEBUG_RAW_RESPONSES) {
    Serial.printf("Sending to IMU %d: ", slave_id);
    for (uint8_t i = 0; i < MODBUS_REQUEST_LEN; i++) {
      Serial.printf("%02X ", request[i]);
    }
    Serial.println();
  }

  clearRxBuffer();
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(TX_ENABLE_DELAY_US);
  Serial2.write(request, sizeof(request));
  Serial2.flush();
  delayMicroseconds(TX_DISABLE_DELAY_US);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  last_bus_activity_us = micros();
}

ParseResult parseResponse(uint8_t slave_id, const uint8_t *buf, size_t len, ImuData &out) {
  if (len != RESPONSE_LEN) {
    return PARSE_BAD_LEN;
  }
  if (buf[0] != slave_id || buf[1] != MODBUS_FUNC_READ_HREG) {
    return PARSE_BAD_HEADER;
  }
  if (buf[2] != RESPONSE_BYTE_COUNT) {
    return PARSE_BAD_COUNT;
  }

  uint16_t crc_calc = crc16_modbus(buf, len - 2);
  uint16_t crc_recv = static_cast<uint16_t>(buf[len - 2]) |
                      static_cast<uint16_t>(buf[len - 1] << 8);
  if (crc_calc != crc_recv) {
    return PARSE_BAD_CRC;
  }

  const size_t data_start = 3;

  int16_t ax = static_cast<int16_t>((buf[data_start] << 8) | buf[data_start + 1]);
  int16_t ay = static_cast<int16_t>((buf[data_start + 2] << 8) | buf[data_start + 3]);
  int16_t az = static_cast<int16_t>((buf[data_start + 4] << 8) | buf[data_start + 5]);

  out.acc[0] = static_cast<float>(ax) * ACC_SCALE;
  out.acc[1] = static_cast<float>(ay) * ACC_SCALE;
  out.acc[2] = static_cast<float>(az) * ACC_SCALE;

  if (buf[2] < 8) {
    out.quat[0] = 0.0f;
    out.quat[1] = 0.0f;
    out.quat[2] = 0.0f;
    out.quat[3] = 0.0f;
  } else {
    size_t quat_offset = data_start + (buf[2] - 8);
    if (quat_offset + 8 > len - 2) {
      return PARSE_BAD_BOUNDS;
    }

    int16_t qw = static_cast<int16_t>((buf[quat_offset] << 8) | buf[quat_offset + 1]);
    int16_t qx = static_cast<int16_t>((buf[quat_offset + 2] << 8) | buf[quat_offset + 3]);
    int16_t qy = static_cast<int16_t>((buf[quat_offset + 4] << 8) | buf[quat_offset + 5]);
    int16_t qz = static_cast<int16_t>((buf[quat_offset + 6] << 8) | buf[quat_offset + 7]);

    out.quat[0] = static_cast<float>(qw) * QUAT_SCALE;
    out.quat[1] = static_cast<float>(qx) * QUAT_SCALE;
    out.quat[2] = static_cast<float>(qy) * QUAT_SCALE;
    out.quat[3] = static_cast<float>(qz) * QUAT_SCALE;
  }

  out.valid = true;
  out.last_update_ms = millis();
  return PARSE_OK;
}

void recordSuccess(uint8_t idx) {
  imu_data[idx].fail_streak = 0;
  imu_data[idx].retries_left = MAX_RETRIES;
  uint32_t interval = successIntervalMs(idx);
  imu_data[idx].next_poll_ms = (interval > 0) ? (millis() + interval) : 0;
}

bool recordFailure(uint8_t idx) {
  if (imu_data[idx].retries_left > 0) {
    imu_data[idx].retries_left--;
    imu_data[idx].next_poll_ms = millis() + RETRY_DELAY_MS;
    retry_count++;
    return false;
  }

  if (imu_data[idx].fail_streak < 4) {
    imu_data[idx].fail_streak++;
  }
  uint32_t backoff = 20 << imu_data[idx].fail_streak;
  imu_data[idx].next_poll_ms = millis() + backoff;
  zeroImu(imu_data[idx]);
  return true;
}

void outputCycleCsv() {
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) Serial.print(",");
    Serial.print(IMU_IDS[i]);
    Serial.print(",");
    if (imu_data[i].valid) {
      Serial.printf("%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f",
                    imu_data[i].acc[0], imu_data[i].acc[1], imu_data[i].acc[2],
                    imu_data[i].quat[0], imu_data[i].quat[1], imu_data[i].quat[2], imu_data[i].quat[3]);
    } else {
      Serial.print("0.000,0.000,0.000,0.0000,0.0000,0.0000,0.0000");
    }
  }
  Serial.println();
}

void reportStatistics() {
  uint32_t now = millis();
  static uint32_t last_report_ms = 0;

  if (now - last_report_ms < FREQ_REPORT_INTERVAL_MS) return;

  last_report_ms = now;

  float freq = 0.0f;
  if (cycle_count > 0) {
    uint32_t elapsed = now - freq_start_ms;
    if (elapsed > 0) {
      freq = (1000.0f * cycle_count) / elapsed;
    }
  }

  Serial.printf("\n--- Statistics ---\n");
  Serial.printf("Update Frequency: %.2f Hz\n", freq);
  Serial.printf("Total Cycles: %lu\n", cycle_count);
  Serial.printf("Success: %lu, Fail: %lu, Timeout: %lu, CRC Error: %lu, Retry: %lu\n",
                success_count, fail_count, timeout_count, crc_error_count, retry_count);

  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    uint32_t wait_ms = (imu_data[i].next_poll_ms > millis())
                         ? (imu_data[i].next_poll_ms - millis())
                         : 0;
    Serial.printf("IMU %d: Fail Streak=%d, Retries Left=%d, Next Poll in %lu ms\n",
                  IMU_IDS[i], imu_data[i].fail_streak, imu_data[i].retries_left, wait_ms);
  }
  Serial.printf("------------------\n\n");

  cycle_count = 0;
  success_count = 0;
  fail_count = 0;
  timeout_count = 0;
  crc_error_count = 0;
  retry_count = 0;
  freq_start_ms = millis();
}

// ================= Setup & Loop =================

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    zeroImu(imu_data[i]);
    imu_data[i].next_poll_ms = 0;
    imu_data[i].fail_streak = 0;
    imu_data[i].retries_left = MAX_RETRIES;
  }

  Serial.println("# Optimized Modbus RTU IMU Poller - Stability Version");
  Serial.print("# IMU IDs: ");
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    Serial.printf("%d", IMU_IDS[i]);
    if (i < NUM_IMUS - 1) Serial.print(",");
  }
  Serial.println();
  Serial.println("# CSV: id,accx,accy,accz,qw,qx,qy,qz (repeated for each IMU)");
  Serial.println("# System ready\n");

  current_state = STATE_IDLE;
  state_start_ms = millis();
  freq_start_ms = millis();
}

void loop() {
  uint32_t now_ms = millis();

  switch (current_state) {
    case STATE_IDLE: {
      bool found_ready = false;

      for (uint8_t offset = 0; offset < NUM_IMUS; offset++) {
        uint8_t idx = (current_imu_index + offset) % NUM_IMUS;
        bool is_ready = (imu_data[idx].next_poll_ms == 0) || (now_ms >= imu_data[idx].next_poll_ms);

        if (is_ready) {
          current_imu_index = idx;
          found_ready = true;
          break;
        }
      }

      if (found_ready && busIsIdle()) {
        current_state = STATE_SENDING_REQUEST;
        state_start_ms = now_ms;
      }
      break;
    }

    case STATE_SENDING_REQUEST:
      response_pos = 0;
      last_rx_us = micros();
      sendRequest(IMU_IDS[current_imu_index]);
      current_state = STATE_WAITING_RESPONSE;
      state_start_ms = now_ms;
      break;

    case STATE_WAITING_RESPONSE:
      if (now_ms - state_start_ms > RESPONSE_TIMEOUT_MS) {
        timeout_count++;
        bool final_fail = recordFailure(current_imu_index);
        if (final_fail) {
          fail_count++;
        }
        current_state = STATE_IDLE;
        break;
      }

      if (response_pos > 0 && (micros() - last_rx_us) > INTER_BYTE_TIMEOUT_US) {
        timeout_count++;
        bool final_fail = recordFailure(current_imu_index);
        if (final_fail) {
          fail_count++;
        }
        current_state = STATE_IDLE;
        break;
      }

      while (Serial2.available() > 0 && response_pos < RESPONSE_LEN) {
        uint8_t byte_in = Serial2.read();
        last_bus_activity_us = micros();
        last_rx_us = last_bus_activity_us;

        if (response_pos == 0 && byte_in != IMU_IDS[current_imu_index]) continue;
        if (response_pos == 1 && byte_in != MODBUS_FUNC_READ_HREG) { response_pos = 0; continue; }
        if (response_pos == 2 && byte_in != RESPONSE_BYTE_COUNT) { response_pos = 0; continue; }

        response_buf[response_pos++] = byte_in;
      }

      if (response_pos >= RESPONSE_LEN) {
        current_state = STATE_PROCESSING_DATA;
      }
      break;

    case STATE_PROCESSING_DATA: {
      ParseResult result = parseResponse(IMU_IDS[current_imu_index], response_buf, response_pos,
                                         imu_data[current_imu_index]);
      if (result == PARSE_OK) {
        success_count++;
        recordSuccess(current_imu_index);
      } else {
        if (result == PARSE_BAD_CRC) {
          crc_error_count++;
        }
        bool final_fail = recordFailure(current_imu_index);
        if (final_fail) {
          fail_count++;
        }
      }

      cycle_count++;
      outputCycleCsv();

      current_imu_index = (current_imu_index + 1) % NUM_IMUS;
      current_state = STATE_IDLE;
      break;
    }
  }

  reportStatistics();
}
