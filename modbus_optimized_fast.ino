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
static const uint32_t BUS_SILENCE_US = 500;  // 恢复总线静默时间
static const uint32_t TX_ENABLE_DELAY_US = 30;  // 恢复延迟
static const uint32_t TX_DISABLE_DELAY_US = 60;  // 恢复延迟
static const uint8_t MAX_CONSECUTIVE_FAILS = 3;  // 最大连续失败次数

// ================= Data Conversion =================
static const float ACC_SCALE = 0.0048828f;
static const float QUAT_SCALE = 0.0001f;

// ================= Frequency Monitoring =================
static const uint32_t FREQ_REPORT_INTERVAL_MS = 1000;

// ================= Read State Machine =================
enum ReadState {
  READ_ACC,     // 读取加速度数据
  READ_QUAT     // 读取四元数据
};

struct ImuData {
  float acc[3];
  float quat[4];
  bool valid;
};

ImuData imu_data[NUM_IMUS];

static uint8_t current_imu_index = 0;
static ReadState current_read_state = READ_ACC;
static bool waiting_response = false;
static uint32_t request_start_ms = 0;
static uint8_t response_buf[QUAT_RESPONSE_LEN];
static uint16_t response_pos = 0;
static uint16_t expected_response_len = 0;

static uint32_t cycle_count = 0;
static uint32_t freq_start_ms = 0;
static uint32_t last_freq_report_ms = 0;
static uint32_t last_bus_activity_us = 0;

// 临时存储ACC和QUAT数据
static float temp_acc[3];
static float temp_quat[4];
static bool temp_acc_valid = false;

// 失败计数
static uint8_t imu_consecutive_fails[NUM_IMUS];
static uint32_t imu_skip_until_ms[NUM_IMUS];

// ================= Utility =================
static inline uint16_t crc16_modbus(const uint8_t *data, size_t len) {
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

static inline void clearRxBuffer() {
  while (Serial2.available() > 0) {
    Serial2.read();
  }
}

static inline bool busIsIdle() {
  if (last_bus_activity_us == 0) {
    return true;
  }
  return (micros() - last_bus_activity_us) >= BUS_SILENCE_US;
}

static inline void recordImuSuccess(uint8_t idx) {
  imu_consecutive_fails[idx] = 0;
  imu_skip_until_ms[idx] = 0;
}

static inline void recordImuFailure(uint8_t idx) {
  imu_consecutive_fails[idx]++;
  if (imu_consecutive_fails[idx] >= MAX_CONSECUTIVE_FAILS) {
    // 跳过该IMU 10ms
    imu_skip_until_ms[idx] = millis() + 10;
    imu_consecutive_fails[idx] = 0;
  }
}

static inline bool canReadImu(uint8_t idx) {
  uint32_t now = millis();
  return now >= imu_skip_until_ms[idx];
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
  Serial2.write(request, MODBUS_REQUEST_LEN);
  Serial2.flush();
  delayMicroseconds(TX_DISABLE_DELAY_US);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  last_bus_activity_us = micros();
}

static inline bool parseAccResponse(uint8_t slave_id, const uint8_t *buf, size_t len, float *acc_out) {
  if (len != ACC_RESPONSE_LEN) {
    return false;
  }
  if (buf[0] != slave_id || buf[1] != MODBUS_FUNC_READ_HREG || buf[2] != ACC_RESPONSE_BYTE_COUNT) {
    return false;
  }

  uint16_t crc_calc = crc16_modbus(buf, len - 2);
  uint16_t crc_recv = static_cast<uint16_t>(buf[len - 2]) |
                      static_cast<uint16_t>(buf[len - 1] << 8);
  if (crc_calc != crc_recv) {
    return false;
  }

  const uint8_t *data = buf + 3;
  int16_t ax = static_cast<int16_t>((data[0] << 8) | data[1]);
  int16_t ay = static_cast<int16_t>((data[2] << 8) | data[3]);
  int16_t az = static_cast<int16_t>((data[4] << 8) | data[5]);

  acc_out[0] = static_cast<float>(ax) * ACC_SCALE;
  acc_out[1] = static_cast<float>(ay) * ACC_SCALE;
  acc_out[2] = static_cast<float>(az) * ACC_SCALE;

  return true;
}

static inline bool parseQuatResponse(uint8_t slave_id, const uint8_t *buf, size_t len, float *quat_out) {
  if (len != QUAT_RESPONSE_LEN) {
    return false;
  }
  if (buf[0] != slave_id || buf[1] != MODBUS_FUNC_READ_HREG || buf[2] != QUAT_RESPONSE_BYTE_COUNT) {
    return false;
  }

  uint16_t crc_calc = crc16_modbus(buf, len - 2);
  uint16_t crc_recv = static_cast<uint16_t>(buf[len - 2]) |
                      static_cast<uint16_t>(buf[len - 1] << 8);
  if (crc_calc != crc_recv) {
    return false;
  }

  const uint8_t *data = buf + 3;
  int16_t qw = static_cast<int16_t>((data[0] << 8) | data[1]);
  int16_t qx = static_cast<int16_t>((data[2] << 8) | data[3]);
  int16_t qy = static_cast<int16_t>((data[4] << 8) | data[5]);
  int16_t qz = static_cast<int16_t>((data[6] << 8) | data[7]);

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
  uint32_t elapsed = millis() - freq_start_ms;
  if (elapsed == 0 || cycle_count == 0) {
    return;
  }

  float frequency = (static_cast<float>(cycle_count) * 1000.0f) / elapsed;
  Serial.print("# 频率: ");
  Serial.print(frequency, 2);
  Serial.println(" Hz");

  cycle_count = 0;
  freq_start_ms = millis();
}

static inline void advanceImuIndex() {
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

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    imu_data[i].valid = false;
    imu_consecutive_fails[i] = 0;
    imu_skip_until_ms[i] = 0;
  }

  Serial.println("# Modbus 优化读取 - 高性能版本");
  Serial.println("# ACC: 0x0034(3) QUAT: 0x0046(4)");

  last_freq_report_ms = millis();
}

void loop() {
  if (!waiting_response) {
    // 如果当前状态是 READ_QUAT，继续读取同一个 IMU 的 QUAT
    if (current_read_state == READ_QUAT && busIsIdle()) {
      response_pos = 0;
      sendRequest(IMU_IDS[current_imu_index], MODBUS_REG_QUAT_START, MODBUS_REG_QUAT_COUNT);
      expected_response_len = QUAT_RESPONSE_LEN;
      waiting_response = true;
      request_start_ms = millis();
    }
    // 否则，开始读取下一个 IMU 的 ACC（如果该IMU未被跳过）
    else if (current_read_state == READ_ACC && busIsIdle()) {
      // 检查当前IMU是否可读
      if (!canReadImu(current_imu_index)) {
        // 跳过当前IMU，切换到下一个
        advanceImuIndex();
        return;
      }
      
      response_pos = 0;
      sendRequest(IMU_IDS[current_imu_index], MODBUS_REG_ACC_START, MODBUS_REG_ACC_COUNT);
      expected_response_len = ACC_RESPONSE_LEN;
      temp_acc_valid = false;
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
    
    // 快速验证
    if (response_pos == 0 && byte_in != IMU_IDS[current_imu_index]) {
      continue;
    }
    if (response_pos == 1 && byte_in != MODBUS_FUNC_READ_HREG) {
      response_pos = 0;
      continue;
    }
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
      if (current_read_state == READ_ACC) {
        if (parseAccResponse(IMU_IDS[current_imu_index], response_buf, response_pos, temp_acc)) {
          temp_acc_valid = true;
          current_read_state = READ_QUAT;  // 继续读取QUAT
        } else {
          temp_acc_valid = false;
          imu_data[current_imu_index].valid = false;
          recordImuFailure(current_imu_index);
          current_read_state = READ_ACC;
          advanceImuIndex();
        }
      } else if (current_read_state == READ_QUAT) {
        bool quat_ok = parseQuatResponse(IMU_IDS[current_imu_index], response_buf, response_pos, temp_quat);
        
        // QUAT读取完成，更新IMU数据
        if (temp_acc_valid && quat_ok) {
          imu_data[current_imu_index].acc[0] = temp_acc[0];
          imu_data[current_imu_index].acc[1] = temp_acc[1];
          imu_data[current_imu_index].acc[2] = temp_acc[2];
          imu_data[current_imu_index].quat[0] = temp_quat[0];
          imu_data[current_imu_index].quat[1] = temp_quat[1];
          imu_data[current_imu_index].quat[2] = temp_quat[2];
          imu_data[current_imu_index].quat[3] = temp_quat[3];
          imu_data[current_imu_index].valid = true;
          recordImuSuccess(current_imu_index);
        } else {
          imu_data[current_imu_index].valid = false;
          recordImuFailure(current_imu_index);
        }
        
        current_read_state = READ_ACC;
        advanceImuIndex();
      }
      
      waiting_response = false;
      
    } else if (millis() - request_start_ms > RESPONSE_TIMEOUT_MS) {
      // 超时处理
      imu_data[current_imu_index].valid = false;
      recordImuFailure(current_imu_index);
      clearRxBuffer();
      last_bus_activity_us = micros();
      waiting_response = false;
      current_read_state = READ_ACC;
      advanceImuIndex();
    }
  }

  // 报告频率
  if (millis() - last_freq_report_ms >= FREQ_REPORT_INTERVAL_MS) {
    last_freq_report_ms = millis();
    reportFrequency();
  }
}
