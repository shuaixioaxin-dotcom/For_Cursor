#include <Arduino.h>

// ================= Pin Configuration =================
#define RS485_RX_PIN 32     // Module RO
#define RS485_TX_PIN 33     // Module DI
#define RS485_DE_RE_PIN 25  // Module DE/RE Control

// ================= Serial Configuration =================
static const uint32_t SERIAL_BAUD = 2000000;
static const uint32_t RS485_BAUD  = 921600;

// ================= IMU Configuration =================
#define NUM_IMUS 2
const uint8_t IMU_IDS[NUM_IMUS] = {1, 2};

// ================= Modbus RTU Configuration =================
static const uint8_t  MODBUS_FUNC_READ_HREG = 0x03;
static const uint16_t MODBUS_REG_START      = 0x0034;
static const uint16_t MODBUS_REG_COUNT      = 0x0016;  // 22 registers
static const uint8_t  MODBUS_REQUEST_LEN    = 8;

static const uint16_t RESPONSE_BYTE_COUNT   = MODBUS_REG_COUNT * 2;                 // 44
static const uint16_t RESPONSE_LEN          = 1 + 1 + 1 + RESPONSE_BYTE_COUNT + 2;  // id+func+bc+data+crc = 49

// ================= Timing Configuration for 200Hz =================
// 200Hz = 5ms周期，2个IMU意味着每2.5ms需要完成一个IMU的轮询
static const uint32_t TARGET_FREQ_HZ        = 200;
static const uint32_t OUTPUT_INTERVAL_US    = 1000000 / TARGET_FREQ_HZ;  // 5000us = 5ms

// 优化后的超时和延迟参数
// 通信时间计算: 57字节 * 10位 / 921600 ≈ 0.62ms
// 预留处理时间，设置超时为2ms
static const uint16_t RESPONSE_TIMEOUT_MS   = 2;
static const uint32_t BUS_SILENCE_US        = 50;      // 减少总线静默时间
static const uint32_t TX_ENABLE_DELAY_US    = 10;      // 减少方向切换延迟
static const uint32_t TX_DISABLE_DELAY_US   = 20;      // 减少方向切换延迟
static const uint32_t FAIL_COOLDOWN_MS      = 5;       // 减少失败冷却时间
static const uint8_t  MAX_BACKOFF_SHIFT     = 2;       // 减少最大退避

// ================= Data Conversion =================
static const float ACC_SCALE  = 0.0048828f;
static const float QUAT_SCALE = 0.0001f;

// ================= State =================
struct ImuData {
  float acc[3];
  float quat[4];
  bool valid;
  uint32_t last_update_ms;
};

ImuData imu_data[NUM_IMUS];
uint32_t imu_next_allowed_ms[NUM_IMUS];
uint8_t imu_fail_streak[NUM_IMUS];

static uint8_t  current_imu_index   = 0;
static bool     waiting_response    = false;
static uint32_t request_start_ms    = 0;
static uint32_t last_bus_activity_us = 0;
static uint32_t last_output_us      = 0;  // 使用微秒定时提高精度

// 性能统计
static uint32_t success_count = 0;
static uint32_t fail_count = 0;
static uint32_t output_count = 0;
static uint32_t stats_start_ms = 0;

// ================= RX Ring Buffer =================
static const size_t RX_RING_SZ = 128;
static uint8_t rx_ring[RX_RING_SZ];
static volatile size_t rx_head = 0;
static volatile size_t rx_tail = 0;

static inline size_t rxCount() {
  return (rx_head + RX_RING_SZ - rx_tail) % RX_RING_SZ;
}

static inline void rxDropOne() {
  rx_tail = (rx_tail + 1) % RX_RING_SZ;
}

static void rxPush(uint8_t b) {
  size_t next = (rx_head + 1) % RX_RING_SZ;
  if (next == rx_tail) {
    rxDropOne();  // 溢出时丢弃最旧数据
  }
  rx_ring[rx_head] = b;
  rx_head = next;
}

static bool rxReadN(size_t n, uint8_t *out) {
  if (rxCount() < n) return false;
  for (size_t i = 0; i < n; ++i) {
    out[i] = rx_ring[rx_tail];
    rxDropOne();
  }
  return true;
}

// ================= Utility =================
// 优化的CRC16计算 - 使用查表法 (ESP32无需PROGMEM)
static const uint16_t crc_table[256] = {
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
    crc = (crc >> 8) ^ crc_table[idx];
  }
  return crc;
}

static void clearUartRxBuffer() {
  while (Serial2.available() > 0) {
    (void)Serial2.read();
  }
}

static void clearAllRx() {
  clearUartRxBuffer();
  rx_head = rx_tail = 0;
}

static inline bool busIsIdle() {
  if (last_bus_activity_us == 0) return true;
  return (micros() - last_bus_activity_us) >= BUS_SILENCE_US;
}

static void zeroImu(ImuData &data) {
  data.acc[0] = data.acc[1] = data.acc[2] = 0.0f;
  data.quat[0] = data.quat[1] = data.quat[2] = data.quat[3] = 0.0f;
  data.valid = false;
  data.last_update_ms = 0;
}

static inline void recordSuccess(uint8_t idx) {
  imu_fail_streak[idx] = 0;
  imu_next_allowed_ms[idx] = 0;
  success_count++;
}

static inline void recordFailure(uint8_t idx) {
  if (imu_fail_streak[idx] < MAX_BACKOFF_SHIFT) imu_fail_streak[idx]++;
  uint32_t backoff = FAIL_COOLDOWN_MS << imu_fail_streak[idx];
  imu_next_allowed_ms[idx] = millis() + backoff;
  fail_count++;
}

static void buildRequest(uint8_t slave_id, uint8_t *out) {
  out[0] = slave_id;
  out[1] = MODBUS_FUNC_READ_HREG;
  out[2] = (MODBUS_REG_START >> 8) & 0xFF;
  out[3] = MODBUS_REG_START & 0xFF;
  out[4] = (MODBUS_REG_COUNT >> 8) & 0xFF;
  out[5] = MODBUS_REG_COUNT & 0xFF;
  uint16_t crc = crc16_modbus_fast(out, 6);
  out[6] = crc & 0xFF;
  out[7] = (crc >> 8) & 0xFF;
}

// ================= Cached requests =================
static uint8_t req_cache[NUM_IMUS][MODBUS_REQUEST_LEN];

static void buildAllRequests() {
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    buildRequest(IMU_IDS[i], req_cache[i]);
  }
}

static inline void sendRequestCached(uint8_t imu_idx) {
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(TX_ENABLE_DELAY_US);
  Serial2.write(req_cache[imu_idx], MODBUS_REQUEST_LEN);
  Serial2.flush();
  delayMicroseconds(TX_DISABLE_DELAY_US);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  last_bus_activity_us = micros();
}

// ================= Frame extraction =================
static bool tryExtractFrame(uint8_t slave_id, uint8_t *frame_out) {
  size_t available = rxCount();
  if (available < RESPONSE_LEN) return false;
  
  // 在环形缓冲区中查找帧起始
  for (size_t start_pos = 0; start_pos <= available - RESPONSE_LEN; ++start_pos) {
    // 直接检查三个关键字节
    size_t idx0 = (rx_tail + start_pos) % RX_RING_SZ;
    size_t idx1 = (idx0 + 1) % RX_RING_SZ;
    size_t idx2 = (idx0 + 2) % RX_RING_SZ;
    
    if (rx_ring[idx0] == slave_id && 
        rx_ring[idx1] == MODBUS_FUNC_READ_HREG && 
        rx_ring[idx2] == RESPONSE_BYTE_COUNT) {
      
      // 提取完整帧
      for (size_t i = 0; i < RESPONSE_LEN; ++i) {
        size_t idx = (rx_tail + start_pos + i) % RX_RING_SZ;
        frame_out[i] = rx_ring[idx];
      }
      
      // 验证CRC
      uint16_t crc_calc = crc16_modbus_fast(frame_out, RESPONSE_LEN - 2);
      uint16_t crc_recv = (uint16_t)frame_out[RESPONSE_LEN - 2] | 
                          ((uint16_t)frame_out[RESPONSE_LEN - 1] << 8);
      
      if (crc_calc != crc_recv) {
        // CRC错误，跳过这个起始位置
        continue;
      }
      
      // 移除处理过的数据
      size_t drop_count = start_pos + RESPONSE_LEN;
      for (size_t i = 0; i < drop_count; ++i) {
        rxDropOne();
      }
      
      return true;
    }
  }
  
  return false;
}

static bool parseResponse(uint8_t slave_id, const uint8_t *buf, size_t len, ImuData &out) {
  if (len != RESPONSE_LEN) return false;
  if (buf[0] != slave_id || buf[1] != MODBUS_FUNC_READ_HREG) return false;
  if (buf[2] != RESPONSE_BYTE_COUNT) return false;

  // CRC已在tryExtractFrame中验证

  const size_t data_start = 3;

  int16_t ax = (int16_t)((buf[data_start] << 8) | buf[data_start + 1]);
  int16_t ay = (int16_t)((buf[data_start + 2] << 8) | buf[data_start + 3]);
  int16_t az = (int16_t)((buf[data_start + 4] << 8) | buf[data_start + 5]);

  out.acc[0] = (float)ax * ACC_SCALE;
  out.acc[1] = (float)ay * ACC_SCALE;
  out.acc[2] = (float)az * ACC_SCALE;

  // 四元数: 数据区的最后8字节
  size_t quat_offset = data_start + (buf[2] - 8);
  if (quat_offset + 8 > len - 2) return false;

  int16_t qw = (int16_t)((buf[quat_offset] << 8) | buf[quat_offset + 1]);
  int16_t qx = (int16_t)((buf[quat_offset + 2] << 8) | buf[quat_offset + 3]);
  int16_t qy = (int16_t)((buf[quat_offset + 4] << 8) | buf[quat_offset + 5]);
  int16_t qz = (int16_t)((buf[quat_offset + 6] << 8) | buf[quat_offset + 7]);

  out.quat[0] = (float)qw * QUAT_SCALE;
  out.quat[1] = (float)qx * QUAT_SCALE;
  out.quat[2] = (float)qy * QUAT_SCALE;
  out.quat[3] = (float)qz * QUAT_SCALE;

  out.valid = true;
  out.last_update_ms = millis();
  return true;
}

// ================= Output =================
// 使用固定格式输出，减少Serial.print调用次数
static char output_buffer[256];

// 快速浮点转字符串（比sprintf快很多）
static char* ftoa_fast(char* buf, float val, uint8_t decimals) {
  if (val < 0) {
    *buf++ = '-';
    val = -val;
  }
  
  int32_t mult = 1;
  for (uint8_t i = 0; i < decimals; i++) mult *= 10;
  
  int32_t ival = (int32_t)(val * mult + 0.5f);
  int32_t integer_part = ival / mult;
  int32_t decimal_part = ival % mult;
  
  // 整数部分
  char temp[12];
  int idx = 0;
  if (integer_part == 0) {
    temp[idx++] = '0';
  } else {
    while (integer_part > 0) {
      temp[idx++] = '0' + (integer_part % 10);
      integer_part /= 10;
    }
  }
  while (idx > 0) *buf++ = temp[--idx];
  
  // 小数部分
  *buf++ = '.';
  for (int8_t i = decimals - 1; i >= 0; i--) {
    int32_t d = 1;
    for (int8_t j = 0; j < i; j++) d *= 10;
    *buf++ = '0' + (decimal_part / d) % 10;
  }
  
  return buf;
}

static void outputImuData() {
  char* ptr = output_buffer;
  
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) *ptr++ = ',';
    
    if (imu_data[i].valid) {
      // 加速度 (3位小数)
      ptr = ftoa_fast(ptr, imu_data[i].acc[0], 3); *ptr++ = ',';
      ptr = ftoa_fast(ptr, imu_data[i].acc[1], 3); *ptr++ = ',';
      ptr = ftoa_fast(ptr, imu_data[i].acc[2], 3); *ptr++ = ',';
      // 四元数 (4位小数)
      ptr = ftoa_fast(ptr, imu_data[i].quat[0], 4); *ptr++ = ',';
      ptr = ftoa_fast(ptr, imu_data[i].quat[1], 4); *ptr++ = ',';
      ptr = ftoa_fast(ptr, imu_data[i].quat[2], 4); *ptr++ = ',';
      ptr = ftoa_fast(ptr, imu_data[i].quat[3], 4);
    } else {
      memcpy(ptr, "0.000,0.000,0.000,0.0000,0.0000,0.0000,0.0000", 44);
      ptr += 44;
    }
  }
  *ptr++ = '\n';
  *ptr = '\0';
  
  Serial.write(output_buffer, ptr - output_buffer);
  output_count++;
}

// 输出性能统计（每10秒）
static void outputStats() {
  uint32_t now = millis();
  if (now - stats_start_ms >= 10000) {
    float elapsed_sec = (now - stats_start_ms) / 1000.0f;
    float actual_freq = output_count / elapsed_sec;
    float success_rate = (success_count + fail_count > 0) ? 
                         (100.0f * success_count / (success_count + fail_count)) : 0.0f;
    
    Serial.print("# STATS: freq=");
    Serial.print(actual_freq, 1);
    Serial.print("Hz, success_rate=");
    Serial.print(success_rate, 1);
    Serial.print("%, success=");
    Serial.print(success_count);
    Serial.print(", fail=");
    Serial.println(fail_count);
    
    // 重置统计
    stats_start_ms = now;
    output_count = 0;
    success_count = 0;
    fail_count = 0;
  }
}

// ================= Scheduling =================
static inline bool pickNextImu(uint32_t now_ms) {
  for (uint8_t offset = 0; offset < NUM_IMUS; ++offset) {
    uint8_t idx = (current_imu_index + offset) % NUM_IMUS;
    if (now_ms >= imu_next_allowed_ms[idx]) {
      current_imu_index = idx;
      return true;
    }
  }
  return false;
}

// ================= Arduino =================
void setup() {
  Serial.begin(SERIAL_BAUD);
  
  // 配置Serial2，增加RX缓冲区
  Serial2.setRxBufferSize(256);
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    zeroImu(imu_data[i]);
    imu_next_allowed_ms[i] = 0;
    imu_fail_streak[i] = 0;
  }

  buildAllRequests();
  
  stats_start_ms = millis();
  last_output_us = micros();
  
  Serial.println("# IMU Reader Started - Target: 200Hz");
  Serial.print("# NUM_IMUS=");
  Serial.print(NUM_IMUS);
  Serial.print(", RS485_BAUD=");
  Serial.print(RS485_BAUD);
  Serial.print(", TIMEOUT=");
  Serial.print(RESPONSE_TIMEOUT_MS);
  Serial.println("ms");
}

void loop() {
  static uint8_t response_buf[RESPONSE_LEN];
  uint32_t now_ms = millis();
  
  // 1) 批接收：将UART数据读入环形缓冲区
  while (Serial2.available() > 0) {
    int v = Serial2.read();
    if (v < 0) break;
    last_bus_activity_us = micros();
    rxPush((uint8_t)v);
  }

  // 2) 处理等待响应
  if (waiting_response) {
    uint8_t slave_id = IMU_IDS[current_imu_index];
    
    if (tryExtractFrame(slave_id, response_buf)) {
      if (parseResponse(slave_id, response_buf, RESPONSE_LEN, imu_data[current_imu_index])) {
        recordSuccess(current_imu_index);
      } else {
        zeroImu(imu_data[current_imu_index]);
        recordFailure(current_imu_index);
      }
      waiting_response = false;
      current_imu_index = (current_imu_index + 1) % NUM_IMUS;
    } 
    else if (now_ms - request_start_ms > RESPONSE_TIMEOUT_MS) {
      zeroImu(imu_data[current_imu_index]);
      recordFailure(current_imu_index);
      clearAllRx();
      waiting_response = false;
      current_imu_index = (current_imu_index + 1) % NUM_IMUS;
    }
  }

  // 3) 发送新请求
  if (!waiting_response && busIsIdle()) {
    if (pickNextImu(now_ms)) {
      sendRequestCached(current_imu_index);
      waiting_response = true;
      request_start_ms = now_ms;
      clearAllRx();
    }
  }

  // 4) 固定频率输出数据（200Hz = 每5000us输出一次）
  uint32_t now_us = micros();
  if (now_us - last_output_us >= OUTPUT_INTERVAL_US) {
    last_output_us += OUTPUT_INTERVAL_US;  // 保持精确间隔，避免漂移
    outputImuData();
  }
  
  // 5) 定期输出性能统计
  outputStats();
}
