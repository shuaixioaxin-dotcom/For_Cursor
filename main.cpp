#include <Arduino.h>

// ================= Pin Configuration =================
#define RS485_RX_PIN 32     // Module RO
#define RS485_TX_PIN 33     // Module DI
#define RS485_DE_RE_PIN 25  // Module DE/RE Control

// ================= Serial Configuration =================
static const uint32_t SERIAL_BAUD = 921600;
static const uint32_t RS485_BAUD = 921600;

// ================= Output Configuration =================
static const bool OUTPUT_ENABLED = true;
static const uint32_t OUTPUT_MIN_INTERVAL_US = 0;
static const uint8_t OUTPUT_EVERY_N_CYCLES = 1;
static const uint16_t OUTPUT_MIN_FREE_BYTES = 128;
static const bool OUTPUT_QUAT = true;
static const uint8_t ACC_DECIMALS = 3;
static const uint8_t QUAT_DECIMALS = 4;

// ================= IMU Configuration =================
#define NUM_IMUS 2
const uint8_t IMU_IDS[NUM_IMUS] = {1, 2};

// ================= Modbus RTU Configuration =================
static const uint8_t MODBUS_FUNC_READ_HREG = 0x03;
static const uint16_t MODBUS_REG_START = 0x0034;
static const uint16_t MODBUS_REG_COUNT = 0x0016;  // 22 registers
static const uint8_t MODBUS_REQUEST_LEN = 8;
static const uint16_t RESPONSE_BYTE_COUNT = MODBUS_REG_COUNT * 2;
static const uint16_t RESPONSE_LEN = 1 + 1 + 1 + RESPONSE_BYTE_COUNT + 2;
static const uint16_t RESPONSE_TIMEOUT_MS = 20;
static const uint32_t BUS_SILENCE_US = 300;
static const uint32_t TX_ENABLE_DELAY_US = 30;
static const uint32_t TX_DISABLE_DELAY_US = 60;
static const uint32_t INTER_BYTE_TIMEOUT_US = 1500;
static const uint32_t RETRY_DELAY_MS = 2;
static const uint32_t FAIL_COOLDOWN_MS = 10;
static const uint8_t MAX_BACKOFF_SHIFT = 4;
static const uint8_t MAX_RETRIES = 1;

// ================= Data Conversion =================
static const int32_t ACC_SCALE_NUM = 48828;
static const int32_t ACC_SCALE_DEN = 10000;

// ================= Frequency Monitoring =================
static const uint32_t FREQ_REPORT_INTERVAL_MS = 1000;

struct ImuData {
  int32_t acc_milli[3];
  int16_t quat_raw[4];
  bool valid;
  uint32_t last_update_ms;
};

ImuData imu_data[NUM_IMUS];
uint32_t imu_next_allowed_ms[NUM_IMUS];
uint8_t imu_fail_streak[NUM_IMUS];
uint8_t imu_retry_left[NUM_IMUS];
uint8_t request_frames[NUM_IMUS][MODBUS_REQUEST_LEN];

static uint8_t current_imu_index = 0;
static bool waiting_response = false;
static uint32_t request_start_ms = 0;
static uint8_t response_buf[RESPONSE_LEN];
static uint16_t response_pos = 0;

static uint32_t cycle_count = 0;
static uint32_t freq_start_ms = 0;
static uint32_t last_freq_report_ms = 0;
static uint32_t success_count = 0;
static uint32_t fail_count = 0;
static uint32_t last_bus_activity_us = 0;
static uint32_t last_byte_us = 0;
static uint32_t last_output_us = 0;

// ================= Utility =================
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
  if (last_bus_activity_us == 0) {
    return true;
  }
  return (micros() - last_bus_activity_us) >= BUS_SILENCE_US;
}

static void zeroImu(ImuData &data) {
  data.acc_milli[0] = 0;
  data.acc_milli[1] = 0;
  data.acc_milli[2] = 0;
  data.quat_raw[0] = 0;
  data.quat_raw[1] = 0;
  data.quat_raw[2] = 0;
  data.quat_raw[3] = 0;
  data.valid = false;
  data.last_update_ms = 0;
}

static void recordSuccess(uint8_t idx) {
  imu_fail_streak[idx] = 0;
  imu_next_allowed_ms[idx] = 0;
  imu_retry_left[idx] = MAX_RETRIES;
}

static void recordFailure(uint8_t idx) {
  if (imu_fail_streak[idx] < MAX_BACKOFF_SHIFT) {
    imu_fail_streak[idx]++;
  }
  uint32_t backoff = FAIL_COOLDOWN_MS << imu_fail_streak[idx];
  imu_next_allowed_ms[idx] = millis() + backoff;
}

static bool scheduleRetry(uint8_t idx) {
  if (imu_retry_left[idx] > 0) {
    imu_retry_left[idx]--;
    imu_next_allowed_ms[idx] = millis() + RETRY_DELAY_MS;
    return true;
  }
  imu_retry_left[idx] = MAX_RETRIES;
  recordFailure(idx);
  return false;
}

static int32_t scaleAccMilli(int16_t raw) {
  int32_t value = static_cast<int32_t>(raw) * ACC_SCALE_NUM;
  if (value >= 0) {
    value += ACC_SCALE_DEN / 2;
  } else {
    value -= ACC_SCALE_DEN / 2;
  }
  return value / ACC_SCALE_DEN;
}

static void printFixed(int32_t value, uint8_t decimals) {
  static const int32_t pow10_table[] = {1, 10, 100, 1000, 10000};
  if (decimals > 4) {
    decimals = 4;
  }
  if (value < 0) {
    Serial.print('-');
    value = -value;
  }
  int32_t scale = pow10_table[decimals];
  int32_t int_part = value / scale;
  int32_t frac = value % scale;
  Serial.print(int_part);
  if (decimals == 0) {
    return;
  }
  Serial.print('.');
  int32_t divisor = scale / 10;
  for (uint8_t i = 0; i < decimals; ++i) {
    int32_t digit = (frac / divisor) % 10;
    Serial.print(digit);
    divisor /= 10;
  }
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

static void sendRequest(uint8_t imu_index) {
  clearRxBuffer();
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(TX_ENABLE_DELAY_US);
  Serial2.write(request_frames[imu_index], MODBUS_REQUEST_LEN);
  Serial2.flush();
  delayMicroseconds(TX_DISABLE_DELAY_US);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  last_bus_activity_us = micros();
  last_byte_us = 0;
}

static bool parseResponse(uint8_t slave_id, const uint8_t *buf, size_t len, ImuData &out) {
  if (len != RESPONSE_LEN) {
    return false;
  }
  if (buf[0] != slave_id || buf[1] != MODBUS_FUNC_READ_HREG) {
    return false;
  }
  if (buf[2] != RESPONSE_BYTE_COUNT) {
    return false;
  }

  uint16_t crc_calc = crc16_modbus(buf, len - 2);
  uint16_t crc_recv = static_cast<uint16_t>(buf[len - 2]) |
                      static_cast<uint16_t>(buf[len - 1] << 8);
  if (crc_calc != crc_recv) {
    return false;
  }

  const size_t data_start = 3;
  int16_t ax = static_cast<int16_t>((buf[data_start] << 8) | buf[data_start + 1]);
  int16_t ay = static_cast<int16_t>((buf[data_start + 2] << 8) | buf[data_start + 3]);
  int16_t az = static_cast<int16_t>((buf[data_start + 4] << 8) | buf[data_start + 5]);

  out.acc_milli[0] = scaleAccMilli(ax);
  out.acc_milli[1] = scaleAccMilli(ay);
  out.acc_milli[2] = scaleAccMilli(az);

  if (buf[2] < 8) {
    out.quat_raw[0] = 0;
    out.quat_raw[1] = 0;
    out.quat_raw[2] = 0;
    out.quat_raw[3] = 0;
  } else {
    size_t quat_offset = data_start + (buf[2] - 8);
    if (quat_offset + 8 > len - 2) {
      return false;
    }
    int16_t qw = static_cast<int16_t>((buf[quat_offset] << 8) | buf[quat_offset + 1]);
    int16_t qx = static_cast<int16_t>((buf[quat_offset + 2] << 8) | buf[quat_offset + 3]);
    int16_t qy = static_cast<int16_t>((buf[quat_offset + 4] << 8) | buf[quat_offset + 5]);
    int16_t qz = static_cast<int16_t>((buf[quat_offset + 6] << 8) | buf[quat_offset + 7]);

    out.quat_raw[0] = qw;
    out.quat_raw[1] = qx;
    out.quat_raw[2] = qy;
    out.quat_raw[3] = qz;
  }

  out.valid = true;
  out.last_update_ms = millis();
  return true;
}

static void outputCycleCsv() {
  if (!OUTPUT_ENABLED) {
    return;
  }
  if (OUTPUT_EVERY_N_CYCLES > 1 && (cycle_count % OUTPUT_EVERY_N_CYCLES) != 0) {
    return;
  }
  uint32_t now_us = micros();
  if (OUTPUT_MIN_INTERVAL_US > 0 && (now_us - last_output_us) < OUTPUT_MIN_INTERVAL_US) {
    return;
  }
  if (OUTPUT_MIN_FREE_BYTES > 0 && Serial.availableForWrite() < OUTPUT_MIN_FREE_BYTES) {
    return;
  }
  last_output_us = now_us;

  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) {
      Serial.print(",");
    }
    Serial.print(IMU_IDS[i]);
    Serial.print(",");
    if (imu_data[i].valid) {
      printFixed(imu_data[i].acc_milli[0], ACC_DECIMALS);
      Serial.print(",");
      printFixed(imu_data[i].acc_milli[1], ACC_DECIMALS);
      Serial.print(",");
      printFixed(imu_data[i].acc_milli[2], ACC_DECIMALS);
      if (OUTPUT_QUAT) {
        Serial.print(",");
        printFixed(imu_data[i].quat_raw[0], QUAT_DECIMALS);
        Serial.print(",");
        printFixed(imu_data[i].quat_raw[1], QUAT_DECIMALS);
        Serial.print(",");
        printFixed(imu_data[i].quat_raw[2], QUAT_DECIMALS);
        Serial.print(",");
        printFixed(imu_data[i].quat_raw[3], QUAT_DECIMALS);
      }
    } else {
      printFixed(0, ACC_DECIMALS);
      Serial.print(",");
      printFixed(0, ACC_DECIMALS);
      Serial.print(",");
      printFixed(0, ACC_DECIMALS);
      if (OUTPUT_QUAT) {
        Serial.print(",");
        printFixed(0, QUAT_DECIMALS);
        Serial.print(",");
        printFixed(0, QUAT_DECIMALS);
        Serial.print(",");
        printFixed(0, QUAT_DECIMALS);
        Serial.print(",");
        printFixed(0, QUAT_DECIMALS);
      }
    }
  }
  Serial.println();
}

static void reportFrequency() {
  if (freq_start_ms == 0 || cycle_count == 0) {
    return;
  }

  uint32_t elapsed = millis() - freq_start_ms;
  if (elapsed == 0) {
    return;
  }

  float frequency = (static_cast<float>(cycle_count) * 1000.0f) / elapsed;
  Serial.print("# Update Frequency: ");
  Serial.print(frequency, 2);
  Serial.print(" Hz (");
  Serial.print(cycle_count);
  Serial.print(" cycles, ok=");
  Serial.print(success_count);
  Serial.print(", fail=");
  Serial.print(fail_count);
  Serial.println(")");

  cycle_count = 0;
  success_count = 0;
  fail_count = 0;
  freq_start_ms = millis();
}

static void advanceImuIndex() {
  current_imu_index++;
  if (current_imu_index >= NUM_IMUS) {
    current_imu_index = 0;
    cycle_count++;
    if (freq_start_ms == 0) {
      freq_start_ms = millis();
    }
    outputCycleCsv();
  }
}

static bool pickNextImu(uint32_t now_ms) {
  for (uint8_t offset = 0; offset < NUM_IMUS; ++offset) {
    uint8_t idx = (current_imu_index + offset) % NUM_IMUS;
    if (now_ms >= imu_next_allowed_ms[idx]) {
      current_imu_index = idx;
      return true;
    }
  }
  return false;
}

void setup() {
#ifdef ESP32
  Serial.setTxBufferSize(2048);
  Serial2.setRxBufferSize(512);
#endif
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    zeroImu(imu_data[i]);
    imu_next_allowed_ms[i] = 0;
    imu_fail_streak[i] = 0;
    imu_retry_left[i] = MAX_RETRIES;
    buildRequest(IMU_IDS[i], request_frames[i]);
  }

  Serial.println("# Modbus RTU raw read enabled");
  Serial.print("# IMU IDs: ");
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) {
      Serial.print(",");
    }
    Serial.print(IMU_IDS[i]);
  }
  Serial.println();
  if (OUTPUT_QUAT) {
    Serial.println("# CSV order: id,accx,accy,accz,qw,qx,qy,qz (repeat)");
  } else {
    Serial.println("# CSV order: id,accx,accy,accz (repeat)");
  }

  last_freq_report_ms = millis();
}

void loop() {
  if (!waiting_response) {
    uint32_t now_ms = millis();
    if (busIsIdle() && pickNextImu(now_ms)) {
      response_pos = 0;
      sendRequest(current_imu_index);
      waiting_response = true;
      request_start_ms = millis();
    }
  }

  while (waiting_response && Serial2.available() > 0 && response_pos < RESPONSE_LEN) {
    int incoming = Serial2.read();
    if (incoming < 0) {
      break;
    }
    last_bus_activity_us = micros();
    last_byte_us = last_bus_activity_us;
    uint8_t byte_in = static_cast<uint8_t>(incoming);
    if (response_pos == 0 && byte_in != IMU_IDS[current_imu_index]) {
      continue;
    }
    if (response_pos == 1 && byte_in != MODBUS_FUNC_READ_HREG) {
      response_pos = 0;
      continue;
    }
    if (response_pos == 2 && byte_in != RESPONSE_BYTE_COUNT) {
      response_pos = 0;
      continue;
    }
    response_buf[response_pos++] = byte_in;
  }

  if (waiting_response) {
    if (response_pos >= RESPONSE_LEN) {
      bool ok = parseResponse(IMU_IDS[current_imu_index], response_buf, response_pos,
                              imu_data[current_imu_index]);
      if (ok) {
        success_count++;
        recordSuccess(current_imu_index);
      } else {
        fail_count++;
        zeroImu(imu_data[current_imu_index]);
        clearRxBuffer();
        last_bus_activity_us = micros();
        scheduleRetry(current_imu_index);
      }
      waiting_response = false;
      advanceImuIndex();
    } else if (response_pos > 0 && last_byte_us > 0 &&
               (micros() - last_byte_us > INTER_BYTE_TIMEOUT_US)) {
      fail_count++;
      zeroImu(imu_data[current_imu_index]);
      clearRxBuffer();
      last_bus_activity_us = micros();
      scheduleRetry(current_imu_index);
      waiting_response = false;
      advanceImuIndex();
    } else if (millis() - request_start_ms > RESPONSE_TIMEOUT_MS) {
      fail_count++;
      zeroImu(imu_data[current_imu_index]);
      clearRxBuffer();
      last_bus_activity_us = micros();
      scheduleRetry(current_imu_index);
      waiting_response = false;
      advanceImuIndex();
    }
  }

  if (FREQ_REPORT_INTERVAL_MS > 0 &&
      (millis() - last_freq_report_ms >= FREQ_REPORT_INTERVAL_MS)) {
    last_freq_report_ms = millis();
    reportFrequency();
  }
}
