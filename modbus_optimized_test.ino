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

// ================= Modbus RTU Configuration =================
static const uint8_t MODBUS_FUNC_READ_HREG = 0x03;

// ACC寄存器配置: 地址0x0034, 长度3个寄存器(6字节)
static const uint16_t ACC_REG_START = 0x0034;
static const uint16_t ACC_REG_COUNT = 0x0003;
static const uint8_t ACC_BYTE_COUNT = ACC_REG_COUNT * 2;  // 6字节
static const uint8_t ACC_RESPONSE_LEN = 1 + 1 + 1 + ACC_BYTE_COUNT + 2;  // 11字节

// QUAT寄存器配置: 地址0x0046, 长度4个寄存器(8字节)
static const uint16_t QUAT_REG_START = 0x0046;
static const uint16_t QUAT_REG_COUNT = 0x0004;
static const uint8_t QUAT_BYTE_COUNT = QUAT_REG_COUNT * 2;  // 8字节
static const uint8_t QUAT_RESPONSE_LEN = 1 + 1 + 1 + QUAT_BYTE_COUNT + 2;  // 13字节

static const uint8_t MODBUS_REQUEST_LEN = 8;
static const uint8_t MAX_RESPONSE_LEN = 16;  // 足够容纳最大响应

static const uint16_t RESPONSE_TIMEOUT_MS = 20;  // 减少超时时间
static const uint32_t BUS_SILENCE_US = 300;      // 减少总线静默时间
static const uint32_t TX_ENABLE_DELAY_US = 20;
static const uint32_t TX_DISABLE_DELAY_US = 40;
static const uint32_t FAIL_COOLDOWN_MS = 20;
static const uint8_t MAX_BACKOFF_SHIFT = 4;

// ================= Data Conversion =================
static const float ACC_SCALE = 0.0048828f;
static const float QUAT_SCALE = 0.0001f;

// ================= Frequency Monitoring =================
static const uint32_t FREQ_REPORT_INTERVAL_MS = 1000;

// ================= Read State =================
enum ReadPhase {
  PHASE_ACC,
  PHASE_QUAT
};

struct ImuData {
  float acc[3];
  float quat[4];
  bool acc_valid;
  bool quat_valid;
  uint32_t last_update_ms;
};

ImuData imu_data[NUM_IMUS];
uint32_t imu_next_allowed_ms[NUM_IMUS];
uint8_t imu_fail_streak[NUM_IMUS];

static uint8_t current_imu_index = 0;
static ReadPhase current_phase = PHASE_ACC;
static bool waiting_response = false;
static uint32_t request_start_ms = 0;
static uint8_t response_buf[MAX_RESPONSE_LEN];
static uint16_t response_pos = 0;
static uint8_t expected_byte_count = 0;
static uint8_t expected_response_len = 0;

static uint32_t cycle_count = 0;
static uint32_t freq_start_ms = 0;
static uint32_t last_freq_report_ms = 0;
static uint32_t success_count = 0;
static uint32_t fail_count = 0;
static uint32_t last_bus_activity_us = 0;

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

static void zeroImuAcc(ImuData &data) {
  data.acc[0] = 0.0f;
  data.acc[1] = 0.0f;
  data.acc[2] = 0.0f;
  data.acc_valid = false;
}

static void zeroImuQuat(ImuData &data) {
  data.quat[0] = 0.0f;
  data.quat[1] = 0.0f;
  data.quat[2] = 0.0f;
  data.quat[3] = 0.0f;
  data.quat_valid = false;
}

static void zeroImu(ImuData &data) {
  zeroImuAcc(data);
  zeroImuQuat(data);
  data.last_update_ms = 0;
}

static void recordSuccess(uint8_t idx) {
  imu_fail_streak[idx] = 0;
  imu_next_allowed_ms[idx] = 0;
}

static void recordFailure(uint8_t idx) {
  if (imu_fail_streak[idx] < MAX_BACKOFF_SHIFT) {
    imu_fail_streak[idx]++;
  }
  uint32_t backoff = FAIL_COOLDOWN_MS << imu_fail_streak[idx];
  imu_next_allowed_ms[idx] = millis() + backoff;
}

static void buildRequest(uint8_t slave_id, uint16_t reg_start, uint16_t reg_count, uint8_t *out) {
  out[0] = slave_id;
  out[1] = MODBUS_FUNC_READ_HREG;
  out[2] = (reg_start >> 8) & 0xFF;
  out[3] = reg_start & 0xFF;
  out[4] = (reg_count >> 8) & 0xFF;
  out[5] = reg_count & 0xFF;
  uint16_t crc = crc16_modbus(out, 6);
  out[6] = crc & 0xFF;
  out[7] = (crc >> 8) & 0xFF;
}

static void sendRequest(uint8_t slave_id, uint16_t reg_start, uint16_t reg_count) {
  uint8_t request[MODBUS_REQUEST_LEN];
  buildRequest(slave_id, reg_start, reg_count, request);

  clearRxBuffer();
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(TX_ENABLE_DELAY_US);
  Serial2.write(request, sizeof(request));
  Serial2.flush();
  delayMicroseconds(TX_DISABLE_DELAY_US);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  last_bus_activity_us = micros();
}

static bool parseAccResponse(uint8_t slave_id, const uint8_t *buf, size_t len, ImuData &out) {
  if (len != ACC_RESPONSE_LEN) {
    return false;
  }
  if (buf[0] != slave_id || buf[1] != MODBUS_FUNC_READ_HREG) {
    return false;
  }
  if (buf[2] != ACC_BYTE_COUNT) {
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

  out.acc[0] = static_cast<float>(ax) * ACC_SCALE;
  out.acc[1] = static_cast<float>(ay) * ACC_SCALE;
  out.acc[2] = static_cast<float>(az) * ACC_SCALE;
  out.acc_valid = true;

  return true;
}

static bool parseQuatResponse(uint8_t slave_id, const uint8_t *buf, size_t len, ImuData &out) {
  if (len != QUAT_RESPONSE_LEN) {
    return false;
  }
  if (buf[0] != slave_id || buf[1] != MODBUS_FUNC_READ_HREG) {
    return false;
  }
  if (buf[2] != QUAT_BYTE_COUNT) {
    return false;
  }

  uint16_t crc_calc = crc16_modbus(buf, len - 2);
  uint16_t crc_recv = static_cast<uint16_t>(buf[len - 2]) |
                      static_cast<uint16_t>(buf[len - 1] << 8);
  if (crc_calc != crc_recv) {
    return false;
  }

  const size_t data_start = 3;
  int16_t qw = static_cast<int16_t>((buf[data_start] << 8) | buf[data_start + 1]);
  int16_t qx = static_cast<int16_t>((buf[data_start + 2] << 8) | buf[data_start + 3]);
  int16_t qy = static_cast<int16_t>((buf[data_start + 4] << 8) | buf[data_start + 5]);
  int16_t qz = static_cast<int16_t>((buf[data_start + 6] << 8) | buf[data_start + 7]);

  out.quat[0] = static_cast<float>(qw) * QUAT_SCALE;
  out.quat[1] = static_cast<float>(qx) * QUAT_SCALE;
  out.quat[2] = static_cast<float>(qy) * QUAT_SCALE;
  out.quat[3] = static_cast<float>(qz) * QUAT_SCALE;
  out.quat_valid = true;
  out.last_update_ms = millis();

  return true;
}

static void outputCycleCsv() {
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) {
      Serial.print(",");
    }
    Serial.print(IMU_IDS[i]);
    Serial.print(",");
    if (imu_data[i].acc_valid && imu_data[i].quat_valid) {
      Serial.print(imu_data[i].acc[0], 3);
      Serial.print(",");
      Serial.print(imu_data[i].acc[1], 3);
      Serial.print(",");
      Serial.print(imu_data[i].acc[2], 3);
      Serial.print(",");
      Serial.print(imu_data[i].quat[0], 4);
      Serial.print(",");
      Serial.print(imu_data[i].quat[1], 4);
      Serial.print(",");
      Serial.print(imu_data[i].quat[2], 4);
      Serial.print(",");
      Serial.print(imu_data[i].quat[3], 4);
    } else {
      Serial.print("0.000,0.000,0.000,0.0000,0.0000,0.0000,0.0000");
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

static void advanceToNextRead() {
  if (current_phase == PHASE_ACC) {
    // ACC读取完成，接下来读QUAT
    current_phase = PHASE_QUAT;
  } else {
    // QUAT读取完成，移动到下一个IMU
    current_phase = PHASE_ACC;
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
}

static bool pickNextImu(uint32_t now_ms) {
  // 如果当前在QUAT阶段，继续当前IMU
  if (current_phase == PHASE_QUAT) {
    return true;
  }
  
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
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    zeroImu(imu_data[i]);
    imu_next_allowed_ms[i] = 0;
    imu_fail_streak[i] = 0;
  }

  Serial.println("# Modbus RTU Optimized Read Test");
  Serial.println("# Separate ACC and QUAT register reads");
  Serial.print("# ACC: reg=0x");
  Serial.print(ACC_REG_START, HEX);
  Serial.print(", count=");
  Serial.print(ACC_REG_COUNT);
  Serial.print(", response=");
  Serial.print(ACC_RESPONSE_LEN);
  Serial.println(" bytes");
  Serial.print("# QUAT: reg=0x");
  Serial.print(QUAT_REG_START, HEX);
  Serial.print(", count=");
  Serial.print(QUAT_REG_COUNT);
  Serial.print(", response=");
  Serial.print(QUAT_RESPONSE_LEN);
  Serial.println(" bytes");
  Serial.print("# IMU IDs: ");
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) {
      Serial.print(",");
    }
    Serial.print(IMU_IDS[i]);
  }
  Serial.println();
  Serial.println("# CSV order: id,accx,accy,accz,qw,qx,qy,qz (repeat)");

  last_freq_report_ms = millis();
}

void loop() {
  if (!waiting_response) {
    uint32_t now_ms = millis();
    if (busIsIdle() && pickNextImu(now_ms)) {
      response_pos = 0;
      
      if (current_phase == PHASE_ACC) {
        sendRequest(IMU_IDS[current_imu_index], ACC_REG_START, ACC_REG_COUNT);
        expected_byte_count = ACC_BYTE_COUNT;
        expected_response_len = ACC_RESPONSE_LEN;
      } else {
        sendRequest(IMU_IDS[current_imu_index], QUAT_REG_START, QUAT_REG_COUNT);
        expected_byte_count = QUAT_BYTE_COUNT;
        expected_response_len = QUAT_RESPONSE_LEN;
      }
      
      waiting_response = true;
      request_start_ms = millis();
    }
  }

  while (waiting_response && Serial2.available() > 0 && response_pos < expected_response_len) {
    int incoming = Serial2.read();
    if (incoming < 0) {
      break;
    }
    last_bus_activity_us = micros();
    uint8_t byte_in = static_cast<uint8_t>(incoming);
    
    // 验证响应头
    if (response_pos == 0 && byte_in != IMU_IDS[current_imu_index]) {
      continue;
    }
    if (response_pos == 1 && byte_in != MODBUS_FUNC_READ_HREG) {
      response_pos = 0;
      continue;
    }
    if (response_pos == 2 && byte_in != expected_byte_count) {
      response_pos = 0;
      continue;
    }
    response_buf[response_pos++] = byte_in;
  }

  if (waiting_response) {
    if (response_pos >= expected_response_len) {
      bool ok = false;
      
      if (current_phase == PHASE_ACC) {
        ok = parseAccResponse(IMU_IDS[current_imu_index], response_buf, response_pos,
                              imu_data[current_imu_index]);
        if (!ok) {
          zeroImuAcc(imu_data[current_imu_index]);
        }
      } else {
        ok = parseQuatResponse(IMU_IDS[current_imu_index], response_buf, response_pos,
                               imu_data[current_imu_index]);
        if (!ok) {
          zeroImuQuat(imu_data[current_imu_index]);
        }
      }
      
      if (ok) {
        success_count++;
        recordSuccess(current_imu_index);
      } else {
        fail_count++;
        recordFailure(current_imu_index);
      }
      
      waiting_response = false;
      advanceToNextRead();
      
    } else if (millis() - request_start_ms > RESPONSE_TIMEOUT_MS) {
      fail_count++;
      
      if (current_phase == PHASE_ACC) {
        zeroImuAcc(imu_data[current_imu_index]);
      } else {
        zeroImuQuat(imu_data[current_imu_index]);
      }
      
      recordFailure(current_imu_index);
      clearRxBuffer();
      last_bus_activity_us = micros();
      waiting_response = false;
      advanceToNextRead();
    }
  }

  if (millis() - last_freq_report_ms >= FREQ_REPORT_INTERVAL_MS) {
    last_freq_report_ms = millis();
    reportFrequency();
  }
}
