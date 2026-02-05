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
const uint8_t IMU_IDS[NUM_IMUS] = {1,2};

// ================= Modbus RTU Configuration =================
static const uint8_t MODBUS_FUNC_READ_HREG = 0x03;

// ACC 寄存器配置
static const uint16_t MODBUS_REG_ACC_START = 0x0034;
static const uint16_t MODBUS_REG_ACC_COUNT = 0x0003;  // 3 registers (6 bytes)
static const uint16_t ACC_RESPONSE_BYTE_COUNT = MODBUS_REG_ACC_COUNT * 2;
static const uint16_t ACC_RESPONSE_LEN = 1 + 1 + 1 + ACC_RESPONSE_BYTE_COUNT + 2; // 11 bytes

// QUAT 寄存器配置
static const uint16_t MODBUS_REG_QUAT_START = 0x0046;
static const uint16_t MODBUS_REG_QUAT_COUNT = 0x0004;  // 4 registers (8 bytes)
static const uint16_t QUAT_RESPONSE_BYTE_COUNT = MODBUS_REG_QUAT_COUNT * 2;
static const uint16_t QUAT_RESPONSE_LEN = 1 + 1 + 1 + QUAT_RESPONSE_BYTE_COUNT + 2; // 13 bytes

static const uint8_t MODBUS_REQUEST_LEN = 8;
static const uint16_t RESPONSE_TIMEOUT_MS = 30;
static const uint32_t BUS_SILENCE_US = 500;
static const uint32_t TX_ENABLE_DELAY_US = 30;
static const uint32_t TX_DISABLE_DELAY_US = 60;
static const uint32_t FAIL_COOLDOWN_MS = 20;
static const uint8_t MAX_BACKOFF_SHIFT = 4;

// ================= Data Conversion =================
static const float ACC_SCALE = 0.0048828f;
static const float QUAT_SCALE = 0.0001f;

// ================= Frequency Monitoring =================
static const uint32_t FREQ_REPORT_INTERVAL_MS = 1000;

// ================= Read State Machine =================
enum ReadState {
  READ_ACC,     // 读取加速度数据
  READ_QUAT,    // 读取四元数据
  READ_COMPLETE // 完成一个IMU的读取
};

struct ImuData {
  float acc[3];
  float quat[4];
  bool valid;
  uint32_t last_update_ms;
};

ImuData imu_data[NUM_IMUS];
uint32_t imu_next_allowed_ms[NUM_IMUS];
uint8_t imu_fail_streak[NUM_IMUS];

static uint8_t current_imu_index = 0;
static ReadState current_read_state = READ_ACC;
static bool waiting_response = false;
static uint32_t request_start_ms = 0;
static uint8_t response_buf[QUAT_RESPONSE_LEN];  // 使用较大的缓冲区
static uint16_t response_pos = 0;
static uint16_t expected_response_len = 0;

static uint32_t cycle_count = 0;
static uint32_t freq_start_ms = 0;
static uint32_t last_freq_report_ms = 0;
static uint32_t success_count = 0;
static uint32_t fail_count = 0;
static uint32_t last_bus_activity_us = 0;

// 临时存储ACC和QUAT数据
static float temp_acc[3];
static float temp_quat[4];
static bool temp_acc_valid = false;
static bool temp_quat_valid = false;

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
  data.acc[0] = 0.0f;
  data.acc[1] = 0.0f;
  data.acc[2] = 0.0f;
  data.quat[0] = 0.0f;
  data.quat[1] = 0.0f;
  data.quat[2] = 0.0f;
  data.quat[3] = 0.0f;
  data.valid = false;
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

static bool parseAccResponse(uint8_t slave_id, const uint8_t *buf, size_t len, float *acc_out) {
  if (len != ACC_RESPONSE_LEN) {
    return false;
  }
  if (buf[0] != slave_id || buf[1] != MODBUS_FUNC_READ_HREG) {
    return false;
  }
  if (buf[2] != ACC_RESPONSE_BYTE_COUNT) {
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

  acc_out[0] = static_cast<float>(ax) * ACC_SCALE;
  acc_out[1] = static_cast<float>(ay) * ACC_SCALE;
  acc_out[2] = static_cast<float>(az) * ACC_SCALE;

  return true;
}

static bool parseQuatResponse(uint8_t slave_id, const uint8_t *buf, size_t len, float *quat_out) {
  if (len != QUAT_RESPONSE_LEN) {
    return false;
  }
  if (buf[0] != slave_id || buf[1] != MODBUS_FUNC_READ_HREG) {
    return false;
  }
  if (buf[2] != QUAT_RESPONSE_BYTE_COUNT) {
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

  quat_out[0] = static_cast<float>(qw) * QUAT_SCALE;
  quat_out[1] = static_cast<float>(qx) * QUAT_SCALE;
  quat_out[2] = static_cast<float>(qy) * QUAT_SCALE;
  quat_out[3] = static_cast<float>(qz) * QUAT_SCALE;

  return true;
}

static void outputCycleCsv() {
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) {
      Serial.print(",");
    }
    Serial.print(IMU_IDS[i]);
    Serial.print(",");
    if (imu_data[i].valid) {
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
  Serial.print("# 更新频率: ");
  Serial.print(frequency, 2);
  Serial.print(" Hz (");
  Serial.print(cycle_count);
  Serial.print(" 周期, 成功=");
  Serial.print(success_count);
  Serial.print(", 失败=");
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
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    zeroImu(imu_data[i]);
    imu_next_allowed_ms[i] = 0;
    imu_fail_streak[i] = 0;
  }

  Serial.println("# Modbus RTU 优化读取模式（分离ACC和QUAT）");
  Serial.print("# IMU IDs: ");
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) {
      Serial.print(",");
    }
    Serial.print(IMU_IDS[i]);
  }
  Serial.println();
  Serial.println("# ACC寄存器: 0x0034, 长度: 3");
  Serial.println("# QUAT寄存器: 0x0046, 长度: 4");
  Serial.println("# CSV格式: id,accx,accy,accz,qw,qx,qy,qz (重复)");

  last_freq_report_ms = millis();
}

void loop() {
  if (!waiting_response) {
    uint32_t now_ms = millis();
    if (busIsIdle() && pickNextImu(now_ms)) {
      response_pos = 0;
      
      // 根据当前状态发送不同的请求
      if (current_read_state == READ_ACC) {
        sendRequest(IMU_IDS[current_imu_index], MODBUS_REG_ACC_START, MODBUS_REG_ACC_COUNT);
        expected_response_len = ACC_RESPONSE_LEN;
        temp_acc_valid = false;
      } else if (current_read_state == READ_QUAT) {
        sendRequest(IMU_IDS[current_imu_index], MODBUS_REG_QUAT_START, MODBUS_REG_QUAT_COUNT);
        expected_response_len = QUAT_RESPONSE_LEN;
        temp_quat_valid = false;
      }
      
      waiting_response = true;
      request_start_ms = millis();
    }
  }

  // 接收数据
  while (waiting_response && Serial2.available() > 0 && response_pos < expected_response_len) {
    int incoming = Serial2.read();
    if (incoming < 0) {
      break;
    }
    last_bus_activity_us = micros();
    uint8_t byte_in = static_cast<uint8_t>(incoming);
    
    // 验证第一个字节（从站ID）
    if (response_pos == 0 && byte_in != IMU_IDS[current_imu_index]) {
      continue;
    }
    // 验证第二个字节（功能码）
    if (response_pos == 1 && byte_in != MODBUS_FUNC_READ_HREG) {
      response_pos = 0;
      continue;
    }
    // 验证第三个字节（数据长度）
    if (response_pos == 2) {
      uint8_t expected_byte_count = (current_read_state == READ_ACC) ? ACC_RESPONSE_BYTE_COUNT : QUAT_RESPONSE_BYTE_COUNT;
      if (byte_in != expected_byte_count) {
        response_pos = 0;
        continue;
      }
    }
    
    response_buf[response_pos++] = byte_in;
  }

  // 处理响应
  if (waiting_response) {
    if (response_pos >= expected_response_len) {
      bool ok = false;
      
      if (current_read_state == READ_ACC) {
        ok = parseAccResponse(IMU_IDS[current_imu_index], response_buf, response_pos, temp_acc);
        if (ok) {
          temp_acc_valid = true;
          current_read_state = READ_QUAT;  // 继续读取QUAT
        } else {
          fail_count++;
          temp_acc_valid = false;
          zeroImu(imu_data[current_imu_index]);
          recordFailure(current_imu_index);
          current_read_state = READ_ACC;  // 重置到ACC状态
          advanceImuIndex();
        }
      } else if (current_read_state == READ_QUAT) {
        ok = parseQuatResponse(IMU_IDS[current_imu_index], response_buf, response_pos, temp_quat);
        if (ok) {
          temp_quat_valid = true;
        } else {
          temp_quat_valid = false;
        }
        
        // QUAT读取完成，更新IMU数据
        if (temp_acc_valid && temp_quat_valid) {
          imu_data[current_imu_index].acc[0] = temp_acc[0];
          imu_data[current_imu_index].acc[1] = temp_acc[1];
          imu_data[current_imu_index].acc[2] = temp_acc[2];
          imu_data[current_imu_index].quat[0] = temp_quat[0];
          imu_data[current_imu_index].quat[1] = temp_quat[1];
          imu_data[current_imu_index].quat[2] = temp_quat[2];
          imu_data[current_imu_index].quat[3] = temp_quat[3];
          imu_data[current_imu_index].valid = true;
          imu_data[current_imu_index].last_update_ms = millis();
          success_count++;
          recordSuccess(current_imu_index);
        } else {
          fail_count++;
          zeroImu(imu_data[current_imu_index]);
          recordFailure(current_imu_index);
        }
        
        current_read_state = READ_ACC;  // 重置到ACC状态
        advanceImuIndex();
      }
      
      waiting_response = false;
      
    } else if (millis() - request_start_ms > RESPONSE_TIMEOUT_MS) {
      // 超时处理
      fail_count++;
      
      if (current_read_state == READ_ACC) {
        temp_acc_valid = false;
      } else if (current_read_state == READ_QUAT) {
        temp_quat_valid = false;
      }
      
      zeroImu(imu_data[current_imu_index]);
      recordFailure(current_imu_index);
      clearRxBuffer();
      last_bus_activity_us = micros();
      waiting_response = false;
      current_read_state = READ_ACC;  // 重置到ACC状态
      advanceImuIndex();
    }
  }

  // 报告频率
  if (millis() - last_freq_report_ms >= FREQ_REPORT_INTERVAL_MS) {
    last_freq_report_ms = millis();
    reportFrequency();
  }
}
