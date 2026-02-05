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
// 降低超时时间 - 根据实际响应时间调整
static const uint16_t RESPONSE_TIMEOUT_US = 2000;  // 2ms超时（原30ms太长）

// 降低总线静默时间 - Modbus规范要求3.5字符时间
// 921600波特率下，1字符≈10.85μs，3.5字符≈38μs，留余量用50μs
static const uint32_t BUS_SILENCE_US = 50;

// RS485方向切换延迟 - 根据芯片规格调整，MAX485约需要几μs
static const uint32_t TX_ENABLE_DELAY_US = 5;
static const uint32_t TX_DISABLE_DELAY_US = 10;

// 失败后的冷却时间
static const uint32_t FAIL_COOLDOWN_US = 1000;  // 1ms
static const uint8_t MAX_BACKOFF_SHIFT = 3;

// ================= Data Conversion =================
static const float ACC_SCALE = 0.0048828f;
static const float QUAT_SCALE = 0.0001f;

// ================= Output Configuration =================
// 使用二进制输出以获得最高性能，设为false则用CSV
static const bool USE_BINARY_OUTPUT = true;

// 输出降频 - 每N个周期输出一次数据（1=每次都输出）
static const uint8_t OUTPUT_DECIMATION = 1;

// 频率报告间隔
static const uint32_t FREQ_REPORT_INTERVAL_MS = 1000;

// ================= Data Structures =================
struct ImuData {
  float acc[3];
  float quat[4];
  bool valid;
  uint32_t last_update_us;
};

#pragma pack(push, 1)
struct BinaryPacket {
  uint8_t header;      // 0xAA
  uint8_t imu_count;   // 2
  uint32_t timestamp;  // micros()
  struct {
    uint8_t id;
    uint8_t valid;
    int16_t acc[3];    // 原始值，减少转换开销
    int16_t quat[4];   // 原始值
  } imu[NUM_IMUS];
  uint8_t checksum;
  uint8_t footer;      // 0x55
};
#pragma pack(pop)

// ================= Global Variables =================
ImuData imu_data[NUM_IMUS];
uint32_t imu_next_allowed_us[NUM_IMUS];
uint8_t imu_fail_streak[NUM_IMUS];

// 保存原始值用于二进制输出
int16_t imu_raw_acc[NUM_IMUS][3];
int16_t imu_raw_quat[NUM_IMUS][4];

static uint8_t current_imu_index = 0;
static bool waiting_response = false;
static uint32_t request_start_us = 0;
static uint8_t response_buf[RESPONSE_LEN];
static uint16_t response_pos = 0;

static uint32_t cycle_count = 0;
static uint32_t output_cycle_count = 0;
static uint32_t freq_start_ms = 0;
static uint32_t last_freq_report_ms = 0;
static uint32_t success_count = 0;
static uint32_t fail_count = 0;
static uint32_t last_bus_activity_us = 0;

// 预计算的请求帧缓存
static uint8_t cached_requests[NUM_IMUS][MODBUS_REQUEST_LEN];

// ================= CRC16 Lookup Table =================
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

// 使用查表法的快速CRC16
static inline uint16_t crc16_modbus_fast(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc = (crc >> 8) ^ pgm_read_word(&crc16_table[(crc ^ *data++) & 0xFF]);
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
  imu_next_allowed_us[idx] = 0;
}

static inline void recordFailure(uint8_t idx) {
  if (imu_fail_streak[idx] < MAX_BACKOFF_SHIFT) {
    imu_fail_streak[idx]++;
  }
  uint32_t backoff = FAIL_COOLDOWN_US << imu_fail_streak[idx];
  imu_next_allowed_us[idx] = micros() + backoff;
}

// 预先构建所有请求帧
static void buildAllRequests() {
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    cached_requests[i][0] = IMU_IDS[i];
    cached_requests[i][1] = MODBUS_FUNC_READ_HREG;
    cached_requests[i][2] = (MODBUS_REG_START >> 8) & 0xFF;
    cached_requests[i][3] = MODBUS_REG_START & 0xFF;
    cached_requests[i][4] = (MODBUS_REG_COUNT >> 8) & 0xFF;
    cached_requests[i][5] = MODBUS_REG_COUNT & 0xFF;
    uint16_t crc = crc16_modbus_fast(cached_requests[i], 6);
    cached_requests[i][6] = crc & 0xFF;
    cached_requests[i][7] = (crc >> 8) & 0xFF;
  }
}

// 发送请求 - 使用预缓存的请求帧
static inline void sendRequest(uint8_t imu_idx) {
  clearRxBuffer();
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(TX_ENABLE_DELAY_US);
  Serial2.write(cached_requests[imu_idx], MODBUS_REQUEST_LEN);
  Serial2.flush();
  delayMicroseconds(TX_DISABLE_DELAY_US);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  last_bus_activity_us = micros();
}

// 解析响应
static bool parseResponse(uint8_t imu_idx, const uint8_t *buf, size_t len) {
  if (len != RESPONSE_LEN) return false;
  
  uint8_t slave_id = IMU_IDS[imu_idx];
  if (buf[0] != slave_id || buf[1] != MODBUS_FUNC_READ_HREG) return false;
  if (buf[2] != RESPONSE_BYTE_COUNT) return false;

  uint16_t crc_calc = crc16_modbus_fast(buf, len - 2);
  uint16_t crc_recv = (uint16_t)buf[len - 2] | ((uint16_t)buf[len - 1] << 8);
  if (crc_calc != crc_recv) return false;

  const size_t data_start = 3;
  
  // 提取原始值
  int16_t ax = (int16_t)((buf[data_start] << 8) | buf[data_start + 1]);
  int16_t ay = (int16_t)((buf[data_start + 2] << 8) | buf[data_start + 3]);
  int16_t az = (int16_t)((buf[data_start + 4] << 8) | buf[data_start + 5]);

  imu_raw_acc[imu_idx][0] = ax;
  imu_raw_acc[imu_idx][1] = ay;
  imu_raw_acc[imu_idx][2] = az;
  
  imu_data[imu_idx].acc[0] = (float)ax * ACC_SCALE;
  imu_data[imu_idx].acc[1] = (float)ay * ACC_SCALE;
  imu_data[imu_idx].acc[2] = (float)az * ACC_SCALE;

  // 四元数在数据末尾
  size_t quat_offset = data_start + (buf[2] - 8);
  if (quat_offset + 8 > len - 2) return false;
  
  int16_t qw = (int16_t)((buf[quat_offset] << 8) | buf[quat_offset + 1]);
  int16_t qx = (int16_t)((buf[quat_offset + 2] << 8) | buf[quat_offset + 3]);
  int16_t qy = (int16_t)((buf[quat_offset + 4] << 8) | buf[quat_offset + 5]);
  int16_t qz = (int16_t)((buf[quat_offset + 6] << 8) | buf[quat_offset + 7]);

  imu_raw_quat[imu_idx][0] = qw;
  imu_raw_quat[imu_idx][1] = qx;
  imu_raw_quat[imu_idx][2] = qy;
  imu_raw_quat[imu_idx][3] = qz;
  
  imu_data[imu_idx].quat[0] = (float)qw * QUAT_SCALE;
  imu_data[imu_idx].quat[1] = (float)qx * QUAT_SCALE;
  imu_data[imu_idx].quat[2] = (float)qy * QUAT_SCALE;
  imu_data[imu_idx].quat[3] = (float)qz * QUAT_SCALE;

  imu_data[imu_idx].valid = true;
  imu_data[imu_idx].last_update_us = micros();
  return true;
}

// 二进制输出 - 非常快速
static void outputBinary() {
  BinaryPacket pkt;
  pkt.header = 0xAA;
  pkt.imu_count = NUM_IMUS;
  pkt.timestamp = micros();
  
  uint8_t checksum = pkt.header ^ pkt.imu_count;
  checksum ^= (pkt.timestamp & 0xFF);
  checksum ^= ((pkt.timestamp >> 8) & 0xFF);
  checksum ^= ((pkt.timestamp >> 16) & 0xFF);
  checksum ^= ((pkt.timestamp >> 24) & 0xFF);
  
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    pkt.imu[i].id = IMU_IDS[i];
    pkt.imu[i].valid = imu_data[i].valid ? 1 : 0;
    pkt.imu[i].acc[0] = imu_raw_acc[i][0];
    pkt.imu[i].acc[1] = imu_raw_acc[i][1];
    pkt.imu[i].acc[2] = imu_raw_acc[i][2];
    pkt.imu[i].quat[0] = imu_raw_quat[i][0];
    pkt.imu[i].quat[1] = imu_raw_quat[i][1];
    pkt.imu[i].quat[2] = imu_raw_quat[i][2];
    pkt.imu[i].quat[3] = imu_raw_quat[i][3];
    
    checksum ^= pkt.imu[i].id ^ pkt.imu[i].valid;
    for (int j = 0; j < 3; ++j) {
      checksum ^= (pkt.imu[i].acc[j] & 0xFF);
      checksum ^= ((pkt.imu[i].acc[j] >> 8) & 0xFF);
    }
    for (int j = 0; j < 4; ++j) {
      checksum ^= (pkt.imu[i].quat[j] & 0xFF);
      checksum ^= ((pkt.imu[i].quat[j] >> 8) & 0xFF);
    }
  }
  
  pkt.checksum = checksum;
  pkt.footer = 0x55;
  
  Serial.write((uint8_t*)&pkt, sizeof(pkt));
}

// CSV输出 - 使用缓冲区减少Serial.print调用次数
static char csv_buffer[256];

static void outputCsv() {
  char* ptr = csv_buffer;
  
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) *ptr++ = ',';
    
    // 使用整数输出然后手动处理小数点，比dtostrf更快
    ptr += sprintf(ptr, "%d,", IMU_IDS[i]);
    
    if (imu_data[i].valid) {
      // 加速度 - 3位小数
      ptr += sprintf(ptr, "%.3f,%.3f,%.3f,",
                     imu_data[i].acc[0], imu_data[i].acc[1], imu_data[i].acc[2]);
      // 四元数 - 4位小数
      ptr += sprintf(ptr, "%.4f,%.4f,%.4f,%.4f",
                     imu_data[i].quat[0], imu_data[i].quat[1],
                     imu_data[i].quat[2], imu_data[i].quat[3]);
    } else {
      ptr += sprintf(ptr, "0.000,0.000,0.000,0.0000,0.0000,0.0000,0.0000");
    }
  }
  *ptr++ = '\n';
  *ptr = '\0';
  
  Serial.write(csv_buffer, ptr - csv_buffer);
}

static void outputData() {
  if (USE_BINARY_OUTPUT) {
    outputBinary();
  } else {
    outputCsv();
  }
}

static void reportFrequency() {
  if (freq_start_ms == 0 || cycle_count == 0) return;

  uint32_t elapsed = millis() - freq_start_ms;
  if (elapsed == 0) return;

  float frequency = ((float)cycle_count * 1000.0f) / elapsed;
  
  // 频率报告使用文本格式
  Serial.print("# Freq: ");
  Serial.print(frequency, 1);
  Serial.print(" Hz (cycles=");
  Serial.print(cycle_count);
  Serial.print(", ok=");
  Serial.print(success_count);
  Serial.print(", fail=");
  Serial.print(fail_count);
  Serial.println(")");

  cycle_count = 0;
  success_count = 0;
  fail_count = 0;
  freq_start_ms = millis();
}

static inline void advanceImuIndex() {
  current_imu_index++;
  if (current_imu_index >= NUM_IMUS) {
    current_imu_index = 0;
    cycle_count++;
    output_cycle_count++;
    
    if (freq_start_ms == 0) {
      freq_start_ms = millis();
    }
    
    // 根据降频设置输出数据
    if (output_cycle_count >= OUTPUT_DECIMATION) {
      output_cycle_count = 0;
      outputData();
    }
  }
}

static inline bool pickNextImu(uint32_t now_us) {
  for (uint8_t offset = 0; offset < NUM_IMUS; ++offset) {
    uint8_t idx = (current_imu_index + offset) % NUM_IMUS;
    if (now_us >= imu_next_allowed_us[idx] || 
        (imu_next_allowed_us[idx] - now_us) > 0x80000000UL) {  // 处理溢出
      current_imu_index = idx;
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  
  // 配置Serial2的接收缓冲区大小（如果平台支持）
  Serial2.setRxBufferSize(256);
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  // 初始化
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    zeroImu(i);
    imu_next_allowed_us[i] = 0;
    imu_fail_streak[i] = 0;
  }
  
  // 预构建请求帧
  buildAllRequests();
  
  last_bus_activity_us = micros();

  Serial.println("# IMU Reader Optimized for 200Hz+");
  Serial.print("# IMU IDs: ");
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) Serial.print(",");
    Serial.print(IMU_IDS[i]);
  }
  Serial.println();
  
  if (USE_BINARY_OUTPUT) {
    Serial.println("# Output: Binary (Header:0xAA, Footer:0x55)");
    Serial.print("# Packet size: ");
    Serial.print(sizeof(BinaryPacket));
    Serial.println(" bytes");
  } else {
    Serial.println("# Output: CSV (id,ax,ay,az,qw,qx,qy,qz,...)");
  }
  
  Serial.print("# Output decimation: 1/");
  Serial.println(OUTPUT_DECIMATION);

  last_freq_report_ms = millis();
}

void loop() {
  uint32_t now_us = micros();
  
  // 状态机：非阻塞轮询
  if (!waiting_response) {
    if (busIsIdle() && pickNextImu(now_us)) {
      response_pos = 0;
      sendRequest(current_imu_index);
      waiting_response = true;
      request_start_us = now_us;
    }
  }

  // 接收数据 - 使用available()检查避免阻塞
  while (waiting_response && Serial2.available() > 0 && response_pos < RESPONSE_LEN) {
    int incoming = Serial2.read();
    if (incoming < 0) break;
    
    last_bus_activity_us = micros();
    uint8_t byte_in = (uint8_t)incoming;
    
    // 帧头验证
    if (response_pos == 0 && byte_in != IMU_IDS[current_imu_index]) continue;
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

  // 检查响应完成或超时
  if (waiting_response) {
    if (response_pos >= RESPONSE_LEN) {
      // 完整响应
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
    } else if ((micros() - request_start_us) > RESPONSE_TIMEOUT_US) {
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

  // 定期报告频率
  if (millis() - last_freq_report_ms >= FREQ_REPORT_INTERVAL_MS) {
    last_freq_report_ms = millis();
    reportFrequency();
  }
}
