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
static const uint16_t MODBUS_REG_START = 0x0034;
static const uint16_t MODBUS_REG_COUNT = 0x0016;  // 22 registers
static const uint8_t MODBUS_REQUEST_LEN = 8;
static const uint16_t RESPONSE_BYTE_COUNT = MODBUS_REG_COUNT * 2;
static const uint16_t RESPONSE_LEN = 1 + 1 + 1 + RESPONSE_BYTE_COUNT + 2;

// ================= 优化后的时序参数 =================
// 关键优化: 减少不必要的延迟
static const uint16_t RESPONSE_TIMEOUT_MS = 5;      // 从30ms减少到5ms
static const uint32_t BUS_SILENCE_US = 100;          // 从500us减少到100us
static const uint32_t TX_ENABLE_DELAY_US = 10;       // 从30us减少到10us
static const uint32_t TX_DISABLE_DELAY_US = 20;      // 从60us减少到20us
static const uint32_t FAIL_COOLDOWN_MS = 10;         // 从20ms减少到10ms
static const uint8_t MAX_BACKOFF_SHIFT = 3;          // 减少最大退避

// ================= 输出控制 (关键优化!) =================
// 设置为0表示每个周期都输出，>0表示按固定间隔输出
static const uint32_t OUTPUT_INTERVAL_MS = 10;       // 每10ms输出一次 = 100Hz输出
// 设置为true使用二进制输出，false使用CSV
static const bool USE_BINARY_OUTPUT = false;

// ================= Data Conversion =================
static const float ACC_SCALE = 0.0048828f;
static const float QUAT_SCALE = 0.0001f;

// ================= Frequency Monitoring =================
static const uint32_t FREQ_REPORT_INTERVAL_MS = 1000;

struct ImuData {
  float acc[3];
  float quat[4];
  bool valid;
  uint32_t last_update_ms;
};

// 二进制输出帧结构
struct __attribute__((packed)) BinaryFrame {
  uint8_t header[2];      // 0xAA 0x55
  uint8_t imu_count;
  struct {
    uint8_t id;
    int16_t acc[3];       // 原始值，节省转换时间
    int16_t quat[4];
  } imus[NUM_IMUS];
  uint8_t checksum;
};

ImuData imu_data[NUM_IMUS];
uint32_t imu_next_allowed_ms[NUM_IMUS];
uint8_t imu_fail_streak[NUM_IMUS];

// 保存原始数据用于二进制输出
int16_t imu_raw_acc[NUM_IMUS][3];
int16_t imu_raw_quat[NUM_IMUS][4];

static uint8_t current_imu_index = 0;
static bool waiting_response = false;
static uint32_t request_start_ms = 0;
static uint8_t response_buf[RESPONSE_LEN];
static uint16_t response_pos = 0;

static uint32_t cycle_count = 0;
static uint32_t freq_start_ms = 0;
static uint32_t last_freq_report_ms = 0;
static uint32_t last_output_ms = 0;
static uint32_t success_count = 0;
static uint32_t fail_count = 0;
static uint32_t last_bus_activity_us = 0;

// 预构建的Modbus请求帧（避免每次重新构建）
static uint8_t prebuilt_requests[NUM_IMUS][MODBUS_REQUEST_LEN];

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

static void zeroImu(uint8_t idx) {
  imu_data[idx].acc[0] = 0.0f;
  imu_data[idx].acc[1] = 0.0f;
  imu_data[idx].acc[2] = 0.0f;
  imu_data[idx].quat[0] = 0.0f;
  imu_data[idx].quat[1] = 0.0f;
  imu_data[idx].quat[2] = 0.0f;
  imu_data[idx].quat[3] = 0.0f;
  imu_data[idx].valid = false;
  imu_data[idx].last_update_ms = 0;
  
  for (int i = 0; i < 3; i++) imu_raw_acc[idx][i] = 0;
  for (int i = 0; i < 4; i++) imu_raw_quat[idx][i] = 0;
}

static inline void recordSuccess(uint8_t idx) {
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

// 预构建所有请求帧
static void prebuildRequests() {
  for (uint8_t i = 0; i < NUM_IMUS; i++) {
    buildRequest(IMU_IDS[i], prebuilt_requests[i]);
  }
}

static inline void sendRequest(uint8_t imu_idx) {
  clearRxBuffer();
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(TX_ENABLE_DELAY_US);
  Serial2.write(prebuilt_requests[imu_idx], MODBUS_REQUEST_LEN);
  Serial2.flush();
  delayMicroseconds(TX_DISABLE_DELAY_US);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  last_bus_activity_us = micros();
}

static bool parseResponse(uint8_t imu_idx, const uint8_t *buf, size_t len) {
  uint8_t slave_id = IMU_IDS[imu_idx];
  
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
  
  // 保存原始值
  imu_raw_acc[imu_idx][0] = static_cast<int16_t>((buf[data_start] << 8) | buf[data_start + 1]);
  imu_raw_acc[imu_idx][1] = static_cast<int16_t>((buf[data_start + 2] << 8) | buf[data_start + 3]);
  imu_raw_acc[imu_idx][2] = static_cast<int16_t>((buf[data_start + 4] << 8) | buf[data_start + 5]);

  // 转换为浮点
  imu_data[imu_idx].acc[0] = static_cast<float>(imu_raw_acc[imu_idx][0]) * ACC_SCALE;
  imu_data[imu_idx].acc[1] = static_cast<float>(imu_raw_acc[imu_idx][1]) * ACC_SCALE;
  imu_data[imu_idx].acc[2] = static_cast<float>(imu_raw_acc[imu_idx][2]) * ACC_SCALE;

  if (buf[2] >= 8) {
    size_t quat_offset = data_start + (buf[2] - 8);
    if (quat_offset + 8 <= len - 2) {
      imu_raw_quat[imu_idx][0] = static_cast<int16_t>((buf[quat_offset] << 8) | buf[quat_offset + 1]);
      imu_raw_quat[imu_idx][1] = static_cast<int16_t>((buf[quat_offset + 2] << 8) | buf[quat_offset + 3]);
      imu_raw_quat[imu_idx][2] = static_cast<int16_t>((buf[quat_offset + 4] << 8) | buf[quat_offset + 5]);
      imu_raw_quat[imu_idx][3] = static_cast<int16_t>((buf[quat_offset + 6] << 8) | buf[quat_offset + 7]);

      imu_data[imu_idx].quat[0] = static_cast<float>(imu_raw_quat[imu_idx][0]) * QUAT_SCALE;
      imu_data[imu_idx].quat[1] = static_cast<float>(imu_raw_quat[imu_idx][1]) * QUAT_SCALE;
      imu_data[imu_idx].quat[2] = static_cast<float>(imu_raw_quat[imu_idx][2]) * QUAT_SCALE;
      imu_data[imu_idx].quat[3] = static_cast<float>(imu_raw_quat[imu_idx][3]) * QUAT_SCALE;
    }
  }

  imu_data[imu_idx].valid = true;
  imu_data[imu_idx].last_update_ms = millis();
  return true;
}

// 优化的CSV输出 - 使用snprintf减少Serial.print调用次数
static char output_buffer[256];

static void outputCycleCsv() {
  char *ptr = output_buffer;
  
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) {
      *ptr++ = ',';
    }
    
    if (imu_data[i].valid) {
      ptr += sprintf(ptr, "%d,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f",
        IMU_IDS[i],
        imu_data[i].acc[0], imu_data[i].acc[1], imu_data[i].acc[2],
        imu_data[i].quat[0], imu_data[i].quat[1], 
        imu_data[i].quat[2], imu_data[i].quat[3]);
    } else {
      ptr += sprintf(ptr, "%d,0.000,0.000,0.000,0.0000,0.0000,0.0000,0.0000", IMU_IDS[i]);
    }
  }
  *ptr++ = '\n';
  *ptr = '\0';
  
  Serial.print(output_buffer);
}

// 二进制输出 - 更高效
static void outputBinaryFrame() {
  BinaryFrame frame;
  frame.header[0] = 0xAA;
  frame.header[1] = 0x55;
  frame.imu_count = NUM_IMUS;
  
  for (uint8_t i = 0; i < NUM_IMUS; i++) {
    frame.imus[i].id = IMU_IDS[i];
    frame.imus[i].acc[0] = imu_raw_acc[i][0];
    frame.imus[i].acc[1] = imu_raw_acc[i][1];
    frame.imus[i].acc[2] = imu_raw_acc[i][2];
    frame.imus[i].quat[0] = imu_raw_quat[i][0];
    frame.imus[i].quat[1] = imu_raw_quat[i][1];
    frame.imus[i].quat[2] = imu_raw_quat[i][2];
    frame.imus[i].quat[3] = imu_raw_quat[i][3];
  }
  
  // 简单校验和
  uint8_t checksum = 0;
  uint8_t *data = (uint8_t*)&frame;
  for (size_t i = 0; i < sizeof(frame) - 1; i++) {
    checksum ^= data[i];
  }
  frame.checksum = checksum;
  
  Serial.write((uint8_t*)&frame, sizeof(frame));
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
  Serial.print("# Freq: ");
  Serial.print(frequency, 1);
  Serial.print("Hz (ok=");
  Serial.print(success_count);
  Serial.print(",fail=");
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
    
    // 关键优化: 控制输出频率
    uint32_t now = millis();
    if (OUTPUT_INTERVAL_MS == 0 || (now - last_output_ms) >= OUTPUT_INTERVAL_MS) {
      last_output_ms = now;
      if (USE_BINARY_OUTPUT) {
        outputBinaryFrame();
      } else {
        outputCycleCsv();
      }
    }
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

  // 预构建请求帧
  prebuildRequests();

  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    zeroImu(i);
    imu_next_allowed_ms[i] = 0;
    imu_fail_streak[i] = 0;
  }

  Serial.println("# Optimized Modbus RTU IMU Reader");
  Serial.print("# IMUs: ");
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) Serial.print(",");
    Serial.print(IMU_IDS[i]);
  }
  Serial.println();
  Serial.print("# Output mode: ");
  Serial.println(USE_BINARY_OUTPUT ? "Binary" : "CSV");
  Serial.print("# Output interval: ");
  Serial.print(OUTPUT_INTERVAL_MS);
  Serial.println("ms");

  last_freq_report_ms = millis();
  last_output_ms = millis();
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

  // 优化: 使用批量读取
  while (waiting_response && Serial2.available() > 0 && response_pos < RESPONSE_LEN) {
    int incoming = Serial2.read();
    if (incoming < 0) {
      break;
    }
    last_bus_activity_us = micros();
    uint8_t byte_in = static_cast<uint8_t>(incoming);
    
    // 帧同步检查
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
      bool ok = parseResponse(current_imu_index, response_buf, response_pos);
      if (ok) {
        success_count++;
        recordSuccess(current_imu_index);
      } else {
        fail_count++;
        zeroImu(current_imu_index);
        recordFailure(current_imu_index);
      }
      waiting_response = false;
      advanceImuIndex();
    } else if (millis() - request_start_ms > RESPONSE_TIMEOUT_MS) {
      fail_count++;
      zeroImu(current_imu_index);
      recordFailure(current_imu_index);
      clearRxBuffer();
      last_bus_activity_us = micros();
      waiting_response = false;
      advanceImuIndex();
    }
  }

  if (millis() - last_freq_report_ms >= FREQ_REPORT_INTERVAL_MS) {
    last_freq_report_ms = millis();
    reportFrequency();
  }
}
