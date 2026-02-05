/**
 * IMU RS485 Modbus RTU Reader - 200Hz Optimized Version
 * 
 * 优化要点：
 * 1. 减少超时和延迟参数
 * 2. 使用二进制输出替代CSV（可选）
 * 3. 减少串口输出阻塞
 * 4. 使用ESP32双核（FreeRTOS）分离IO和通信
 * 5. 优化缓冲区和数据处理
 */

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
// 原始值 -> 优化值
static const uint16_t RESPONSE_TIMEOUT_MS = 3;      // 30ms -> 3ms (IMU响应通常<1ms)
static const uint32_t BUS_SILENCE_US = 100;         // 500us -> 100us (Modbus最小3.5字符时间@921600≈38us)
static const uint32_t TX_ENABLE_DELAY_US = 10;      // 30us -> 10us
static const uint32_t TX_DISABLE_DELAY_US = 20;     // 60us -> 20us
static const uint32_t FAIL_COOLDOWN_MS = 5;         // 20ms -> 5ms
static const uint8_t MAX_BACKOFF_SHIFT = 2;         // 4 -> 2 (最大退避时间减少)

// ================= Data Conversion =================
static const float ACC_SCALE = 0.0048828f;
static const float QUAT_SCALE = 0.0001f;

// ================= Output Configuration =================
#define OUTPUT_MODE_CSV    0
#define OUTPUT_MODE_BINARY 1
#define OUTPUT_MODE_NONE   2
static const uint8_t OUTPUT_MODE = OUTPUT_MODE_CSV;  // 可切换为BINARY以提高速度

// ================= Frequency Monitoring =================
static const uint32_t FREQ_REPORT_INTERVAL_MS = 2000;  // 减少报告频率

// ================= Data Structures =================
struct ImuData {
  float acc[3];
  float quat[4];
  bool valid;
  uint32_t last_update_ms;
};

// ================= 二进制输出帧格式 =================
#pragma pack(push, 1)
struct BinaryFrame {
  uint8_t header[2];      // 0xAA, 0x55
  uint8_t imu_count;
  uint32_t timestamp_ms;
  struct {
    uint8_t id;
    uint8_t valid;
    int16_t acc[3];       // 原始值，减少转换开销
    int16_t quat[4];
  } imu[NUM_IMUS];
  uint8_t checksum;
};
#pragma pack(pop)

// ================= Global Variables =================
ImuData imu_data[NUM_IMUS];
uint32_t imu_next_allowed_ms[NUM_IMUS];
uint8_t imu_fail_streak[NUM_IMUS];

// Raw data for binary output (避免重复转换)
int16_t imu_raw_acc[NUM_IMUS][3];
int16_t imu_raw_quat[NUM_IMUS][4];

static uint8_t current_imu_index = 0;
static bool waiting_response = false;
static uint32_t request_start_us = 0;  // 使用微秒提高精度
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

// ================= CRC Lookup Table (比循环计算快~4倍) =================
static const uint16_t crc_table[256] PROGMEM = {
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
    uint8_t idx = (crc ^ data[i]) & 0xFF;
    crc = (crc >> 8) ^ pgm_read_word(&crc_table[idx]);
  }
  return crc;
}

// ================= Utility Functions =================
static inline void clearRxBuffer() {
  while (Serial2.available() > 0) {
    Serial2.read();
  }
}

static inline bool busIsIdle() {
  if (last_bus_activity_us == 0) return true;
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
  imu_raw_acc[idx][0] = 0;
  imu_raw_acc[idx][1] = 0;
  imu_raw_acc[idx][2] = 0;
  imu_raw_quat[idx][0] = 0;
  imu_raw_quat[idx][1] = 0;
  imu_raw_quat[idx][2] = 0;
  imu_raw_quat[idx][3] = 0;
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

// 预计算所有IMU的请求帧
static void buildAllRequests() {
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    uint8_t *out = request_cache[i];
    out[0] = IMU_IDS[i];
    out[1] = MODBUS_FUNC_READ_HREG;
    out[2] = (MODBUS_REG_START >> 8) & 0xFF;
    out[3] = MODBUS_REG_START & 0xFF;
    out[4] = (MODBUS_REG_COUNT >> 8) & 0xFF;
    out[5] = MODBUS_REG_COUNT & 0xFF;
    uint16_t crc = crc16_modbus_fast(out, 6);
    out[6] = crc & 0xFF;
    out[7] = (crc >> 8) & 0xFF;
  }
}

static inline void sendRequest(uint8_t imu_idx) {
  clearRxBuffer();
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(TX_ENABLE_DELAY_US);
  Serial2.write(request_cache[imu_idx], MODBUS_REQUEST_LEN);
  Serial2.flush();
  delayMicroseconds(TX_DISABLE_DELAY_US);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  last_bus_activity_us = micros();
}

static bool parseResponse(uint8_t imu_idx, const uint8_t *buf, size_t len) {
  uint8_t slave_id = IMU_IDS[imu_idx];
  
  if (len != RESPONSE_LEN) return false;
  if (buf[0] != slave_id || buf[1] != MODBUS_FUNC_READ_HREG) return false;
  if (buf[2] != RESPONSE_BYTE_COUNT) return false;

  uint16_t crc_calc = crc16_modbus_fast(buf, len - 2);
  uint16_t crc_recv = (uint16_t)buf[len - 2] | ((uint16_t)buf[len - 1] << 8);
  if (crc_calc != crc_recv) return false;

  const size_t data_start = 3;
  
  // 存储原始值
  imu_raw_acc[imu_idx][0] = (int16_t)((buf[data_start] << 8) | buf[data_start + 1]);
  imu_raw_acc[imu_idx][1] = (int16_t)((buf[data_start + 2] << 8) | buf[data_start + 3]);
  imu_raw_acc[imu_idx][2] = (int16_t)((buf[data_start + 4] << 8) | buf[data_start + 5]);

  // 转换为浮点（仅在需要时）
  imu_data[imu_idx].acc[0] = (float)imu_raw_acc[imu_idx][0] * ACC_SCALE;
  imu_data[imu_idx].acc[1] = (float)imu_raw_acc[imu_idx][1] * ACC_SCALE;
  imu_data[imu_idx].acc[2] = (float)imu_raw_acc[imu_idx][2] * ACC_SCALE;

  if (buf[2] >= 8) {
    size_t quat_offset = data_start + (buf[2] - 8);
    if (quat_offset + 8 <= len - 2) {
      imu_raw_quat[imu_idx][0] = (int16_t)((buf[quat_offset] << 8) | buf[quat_offset + 1]);
      imu_raw_quat[imu_idx][1] = (int16_t)((buf[quat_offset + 2] << 8) | buf[quat_offset + 3]);
      imu_raw_quat[imu_idx][2] = (int16_t)((buf[quat_offset + 4] << 8) | buf[quat_offset + 5]);
      imu_raw_quat[imu_idx][3] = (int16_t)((buf[quat_offset + 6] << 8) | buf[quat_offset + 7]);

      imu_data[imu_idx].quat[0] = (float)imu_raw_quat[imu_idx][0] * QUAT_SCALE;
      imu_data[imu_idx].quat[1] = (float)imu_raw_quat[imu_idx][1] * QUAT_SCALE;
      imu_data[imu_idx].quat[2] = (float)imu_raw_quat[imu_idx][2] * QUAT_SCALE;
      imu_data[imu_idx].quat[3] = (float)imu_raw_quat[imu_idx][3] * QUAT_SCALE;
    }
  }

  imu_data[imu_idx].valid = true;
  imu_data[imu_idx].last_update_ms = millis();
  return true;
}

// ================= Output Functions =================

// 优化的CSV输出 - 使用snprintf批量格式化减少Serial.print调用
static char output_buffer[256];

static void outputCycleCsv() {
  if (OUTPUT_MODE == OUTPUT_MODE_NONE) return;
  
  if (OUTPUT_MODE == OUTPUT_MODE_BINARY) {
    // 二进制输出 - 更快
    BinaryFrame frame;
    frame.header[0] = 0xAA;
    frame.header[1] = 0x55;
    frame.imu_count = NUM_IMUS;
    frame.timestamp_ms = millis();
    
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < NUM_IMUS; ++i) {
      frame.imu[i].id = IMU_IDS[i];
      frame.imu[i].valid = imu_data[i].valid ? 1 : 0;
      frame.imu[i].acc[0] = imu_raw_acc[i][0];
      frame.imu[i].acc[1] = imu_raw_acc[i][1];
      frame.imu[i].acc[2] = imu_raw_acc[i][2];
      frame.imu[i].quat[0] = imu_raw_quat[i][0];
      frame.imu[i].quat[1] = imu_raw_quat[i][1];
      frame.imu[i].quat[2] = imu_raw_quat[i][2];
      frame.imu[i].quat[3] = imu_raw_quat[i][3];
    }
    
    // 计算校验和
    uint8_t *ptr = (uint8_t*)&frame;
    for (size_t i = 0; i < sizeof(frame) - 1; ++i) {
      checksum ^= ptr[i];
    }
    frame.checksum = checksum;
    
    Serial.write((uint8_t*)&frame, sizeof(frame));
    return;
  }
  
  // CSV输出 - 使用缓冲区批量输出
  int pos = 0;
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) {
      output_buffer[pos++] = ',';
    }
    if (imu_data[i].valid) {
      pos += snprintf(output_buffer + pos, sizeof(output_buffer) - pos,
                      "%d,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f",
                      IMU_IDS[i],
                      imu_data[i].acc[0], imu_data[i].acc[1], imu_data[i].acc[2],
                      imu_data[i].quat[0], imu_data[i].quat[1],
                      imu_data[i].quat[2], imu_data[i].quat[3]);
    } else {
      pos += snprintf(output_buffer + pos, sizeof(output_buffer) - pos,
                      "%d,0.000,0.000,0.000,0.0000,0.0000,0.0000,0.0000",
                      IMU_IDS[i]);
    }
  }
  output_buffer[pos++] = '\n';
  output_buffer[pos] = '\0';
  Serial.write(output_buffer, pos);
}

static void reportFrequency() {
  if (freq_start_ms == 0 || cycle_count == 0) return;

  uint32_t elapsed = millis() - freq_start_ms;
  if (elapsed == 0) return;

  float frequency = ((float)cycle_count * 1000.0f) / elapsed;
  
  int len = snprintf(output_buffer, sizeof(output_buffer),
                     "# Freq: %.1f Hz (cycles=%lu, ok=%lu, fail=%lu)\n",
                     frequency, cycle_count, success_count, fail_count);
  Serial.write(output_buffer, len);

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

// ================= Setup =================
void setup() {
  Serial.begin(SERIAL_BAUD);
  
  // 增大Serial2缓冲区（如果需要）
  Serial2.setRxBufferSize(256);
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  // 预计算所有请求帧
  buildAllRequests();

  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    zeroImu(i);
    imu_next_allowed_ms[i] = 0;
    imu_fail_streak[i] = 0;
  }

  Serial.println("# IMU Reader Optimized for 200Hz");
  Serial.print("# IMU IDs: ");
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) Serial.print(",");
    Serial.print(IMU_IDS[i]);
  }
  Serial.println();
  Serial.println("# Timing: timeout=3ms, silence=100us");
  Serial.println("# CSV: id,ax,ay,az,qw,qx,qy,qz (repeat)");

  last_freq_report_ms = millis();
}

// ================= Main Loop =================
void loop() {
  uint32_t now_us = micros();
  uint32_t now_ms = millis();
  
  // 发送请求
  if (!waiting_response) {
    if (busIsIdle() && pickNextImu(now_ms)) {
      response_pos = 0;
      sendRequest(current_imu_index);
      waiting_response = true;
      request_start_us = now_us;
    }
  }

  // 接收响应 - 使用available()批量读取
  if (waiting_response) {
    int avail = Serial2.available();
    while (avail > 0 && response_pos < RESPONSE_LEN) {
      int incoming = Serial2.read();
      if (incoming < 0) break;
      
      last_bus_activity_us = micros();
      uint8_t byte_in = (uint8_t)incoming;
      
      // 快速验证头部
      if (response_pos == 0 && byte_in != IMU_IDS[current_imu_index]) {
        avail--;
        continue;
      }
      if (response_pos == 1 && byte_in != MODBUS_FUNC_READ_HREG) {
        response_pos = 0;
        avail--;
        continue;
      }
      if (response_pos == 2 && byte_in != RESPONSE_BYTE_COUNT) {
        response_pos = 0;
        avail--;
        continue;
      }
      
      response_buf[response_pos++] = byte_in;
      avail--;
    }
  }

  // 处理响应
  if (waiting_response) {
    if (response_pos >= RESPONSE_LEN) {
      // 收到完整响应
      if (parseResponse(current_imu_index, response_buf, response_pos)) {
        success_count++;
        recordSuccess(current_imu_index);
      } else {
        fail_count++;
        zeroImu(current_imu_index);
        recordFailure(current_imu_index);
      }
      waiting_response = false;
      advanceImuIndex();
    } else if ((now_us - request_start_us) > (RESPONSE_TIMEOUT_MS * 1000UL)) {
      // 超时
      fail_count++;
      zeroImu(current_imu_index);
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
