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

// ================= OPTIMIZED Timing Parameters =================
// 原始值 -> 优化值 (说明)
static const uint16_t RESPONSE_TIMEOUT_MS = 5;      // 30ms -> 5ms (足够接收49字节)
static const uint32_t BUS_SILENCE_US = 50;          // 500us -> 50us (Modbus最小3.5字符时间@921600≈38us)
static const uint32_t TX_ENABLE_DELAY_US = 10;      // 30us -> 10us (RS485芯片典型<1us)
static const uint32_t TX_DISABLE_DELAY_US = 20;     // 60us -> 20us (等待最后字节发送完成)
static const uint32_t FAIL_COOLDOWN_MS = 5;         // 20ms -> 5ms (更快重试)
static const uint8_t MAX_BACKOFF_SHIFT = 2;         // 4 -> 2 (减少最大退避时间)

// ================= Data Conversion =================
static const float ACC_SCALE = 0.0048828f;
static const float QUAT_SCALE = 0.0001f;

// ================= Frequency Monitoring =================
static const uint32_t FREQ_REPORT_INTERVAL_MS = 1000;

// ================= Output Configuration =================
// 设置为 true 可关闭每周期CSV输出以获得最高性能
static const bool DISABLE_CSV_OUTPUT = false;
// 设置CSV输出间隔 (1=每周期, 5=每5周期输出一次)
static const uint8_t CSV_OUTPUT_DIVIDER = 1;

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
static bool waiting_response = false;
static uint32_t request_start_ms = 0;
static uint32_t request_start_us = 0;  // 微秒级计时用于更精确的超时
static uint8_t response_buf[RESPONSE_LEN];
static uint16_t response_pos = 0;

static uint32_t cycle_count = 0;
static uint32_t freq_start_ms = 0;
static uint32_t last_freq_report_ms = 0;
static uint32_t success_count = 0;
static uint32_t fail_count = 0;
static uint32_t last_bus_activity_us = 0;

// 预计算的请求帧缓存
static uint8_t request_cache[NUM_IMUS][MODBUS_REQUEST_LEN];

// CSV输出缓冲区 (避免多次Serial.print调用)
static char csv_buffer[256];

// ================= CRC16查找表 (比循环计算快约10倍) =================
static const uint16_t crc16_table[256] PROGMEM = {
  0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
  0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
  0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
  0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
  0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
  0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
  0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
  0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
  0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
  0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
  0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
  0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
  0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
  0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
  0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
  0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
  0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
  0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
  0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
  0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
  0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
  0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
  0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
  0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
  0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
  0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
  0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
  0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
  0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
  0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
  0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
  0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

static uint16_t crc16_modbus_fast(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    uint8_t index = (crc ^ data[i]) & 0xFF;
    crc = (crc >> 8) ^ pgm_read_word(&crc16_table[index]);
  }
  return crc;
}

// 保留原始CRC函数用于兼容性
static uint16_t crc16_modbus(const uint8_t *data, size_t len) {
  return crc16_modbus_fast(data, len);
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

// 预构建请求帧 (在setup中调用一次)
static void buildRequestCache() {
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    uint8_t *out = request_cache[i];
    out[0] = IMU_IDS[i];
    out[1] = MODBUS_FUNC_READ_HREG;
    out[2] = (MODBUS_REG_START >> 8) & 0xFF;
    out[3] = MODBUS_REG_START & 0xFF;
    out[4] = (MODBUS_REG_COUNT >> 8) & 0xFF;
    out[5] = MODBUS_REG_COUNT & 0xFF;
    uint16_t crc = crc16_modbus(out, 6);
    out[6] = crc & 0xFF;
    out[7] = (crc >> 8) & 0xFF;
  }
}

static inline void sendRequest(uint8_t imu_index) {
  clearRxBuffer();
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(TX_ENABLE_DELAY_US);
  Serial2.write(request_cache[imu_index], MODBUS_REQUEST_LEN);
  Serial2.flush();
  delayMicroseconds(TX_DISABLE_DELAY_US);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  last_bus_activity_us = micros();
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
      return false;
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
  return true;
}

// 优化的CSV输出 - 使用sprintf一次性构建字符串
static void outputCycleCsv() {
  if (DISABLE_CSV_OUTPUT) {
    return;
  }
  
  // 使用分频器减少输出频率
  static uint8_t output_counter = 0;
  if (++output_counter < CSV_OUTPUT_DIVIDER) {
    return;
  }
  output_counter = 0;
  
  char *ptr = csv_buffer;
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
  *ptr = '\0';
  Serial.println(csv_buffer);
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
  Serial.print(" fail=");
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
  
  // 增加Serial2的RX缓冲区大小 (如果平台支持)
  Serial2.setRxBufferSize(256);
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    zeroImu(imu_data[i]);
    imu_next_allowed_ms[i] = 0;
    imu_fail_streak[i] = 0;
  }

  // 预构建所有请求帧
  buildRequestCache();

  Serial.println("# Modbus RTU Optimized for 200Hz+");
  Serial.print("# IMU IDs: ");
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) Serial.print(",");
    Serial.print(IMU_IDS[i]);
  }
  Serial.println();
  Serial.print("# Timing: silence=");
  Serial.print(BUS_SILENCE_US);
  Serial.print("us, timeout=");
  Serial.print(RESPONSE_TIMEOUT_MS);
  Serial.println("ms");
  Serial.println("# CSV: id,ax,ay,az,qw,qx,qy,qz (repeat)");

  last_freq_report_ms = millis();
}

void loop() {
  uint32_t now_ms = millis();
  
  if (!waiting_response) {
    if (busIsIdle() && pickNextImu(now_ms)) {
      response_pos = 0;
      sendRequest(current_imu_index);
      waiting_response = true;
      request_start_ms = now_ms;
      request_start_us = micros();
    }
  }

  // 批量读取可用数据
  while (waiting_response && Serial2.available() > 0 && response_pos < RESPONSE_LEN) {
    int incoming = Serial2.read();
    if (incoming < 0) {
      break;
    }
    last_bus_activity_us = micros();
    uint8_t byte_in = static_cast<uint8_t>(incoming);
    
    // 帧同步验证
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
      // 完整响应接收
      bool ok = parseResponse(IMU_IDS[current_imu_index], response_buf, response_pos,
                              imu_data[current_imu_index]);
      if (ok) {
        success_count++;
        recordSuccess(current_imu_index);
      } else {
        fail_count++;
        zeroImu(imu_data[current_imu_index]);
        recordFailure(current_imu_index);
      }
      waiting_response = false;
      advanceImuIndex();
    } else if (millis() - request_start_ms >= RESPONSE_TIMEOUT_MS) {
      // 超时处理
      fail_count++;
      zeroImu(imu_data[current_imu_index]);
      recordFailure(current_imu_index);
      clearRxBuffer();
      last_bus_activity_us = micros();
      waiting_response = false;
      advanceImuIndex();
    }
  }

  // 频率报告
  if (now_ms - last_freq_report_ms >= FREQ_REPORT_INTERVAL_MS) {
    last_freq_report_ms = now_ms;
    reportFrequency();
  }
}
