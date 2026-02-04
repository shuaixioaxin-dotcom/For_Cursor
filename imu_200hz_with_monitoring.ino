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
const uint8_t IMU_IDS[NUM_IMUS] = {1,2};

// ================= Modbus RTU Configuration =================
static const uint8_t  MODBUS_FUNC_READ_HREG = 0x03;
static const uint16_t MODBUS_REG_START      = 0x0034;
static const uint16_t MODBUS_REG_COUNT      = 0x0016;  // 22 registers
static const uint8_t  MODBUS_REQUEST_LEN    = 8;

static const uint16_t RESPONSE_BYTE_COUNT   = MODBUS_REG_COUNT * 2;                 // 44
static const uint16_t RESPONSE_LEN          = 1 + 1 + 1 + RESPONSE_BYTE_COUNT + 2;  // id+func+bc+data+crc = 49

// ================= 200Hz Optimization =================
static const uint16_t RESPONSE_TIMEOUT_MS   = 3;
static const uint32_t BUS_SILENCE_US        = 100;
static const uint32_t TX_ENABLE_DELAY_US    = 10;
static const uint32_t TX_DISABLE_DELAY_US   = 20;
static const uint32_t FAIL_COOLDOWN_MS      = 5;
static const uint8_t  MAX_BACKOFF_SHIFT     = 3;

// ================= Performance Monitoring =================
static const uint32_t STATS_INTERVAL_MS     = 1000;

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

// ================= Performance Stats =================
struct PerfStats {
  uint32_t total_requests;
  uint32_t successful_reads;
  uint32_t timeouts;
  uint32_t crc_errors;
  uint32_t loop_count;
  uint32_t last_stats_ms;
  uint32_t min_loop_us;
  uint32_t max_loop_us;
  uint32_t last_loop_us;
} perf_stats = {0, 0, 0, 0, 0, 0, 999999, 0, 0};

// ================= RX Ring Buffer =================
static const size_t RX_RING_SZ = 128;
static uint8_t rx_ring[RX_RING_SZ];
static size_t rx_head = 0;
static size_t rx_tail = 0;

static inline size_t rxCount() {
  return (rx_head + RX_RING_SZ - rx_tail) % RX_RING_SZ;
}

static inline void rxDropOne() {
  rx_tail = (rx_tail + 1) % RX_RING_SZ;
}

static void rxPush(uint8_t b) {
  size_t next = (rx_head + 1) % RX_RING_SZ;
  if (next == rx_tail) {
    rxDropOne();
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
static uint16_t crc16_modbus(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; ++j) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
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

static bool busIsIdle() {
  if (last_bus_activity_us == 0) return true;
  return (micros() - last_bus_activity_us) >= BUS_SILENCE_US;
}

static void zeroImu(ImuData &data) {
  data.acc[0] = data.acc[1] = data.acc[2] = 0.0f;
  data.quat[0] = data.quat[1] = data.quat[2] = data.quat[3] = 0.0f;
  data.valid = false;
  data.last_update_ms = 0;
}

static void recordSuccess(uint8_t idx) {
  imu_fail_streak[idx] = 0;
  imu_next_allowed_ms[idx] = 0;
  perf_stats.successful_reads++;
}

static void recordFailure(uint8_t idx) {
  if (imu_fail_streak[idx] < MAX_BACKOFF_SHIFT) imu_fail_streak[idx]++;
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

// ================= Cached requests =================
static uint8_t req_cache[NUM_IMUS][MODBUS_REQUEST_LEN];

static void buildAllRequests() {
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    buildRequest(IMU_IDS[i], req_cache[i]);
  }
}

static void sendRequestCached(uint8_t imu_idx) {
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(TX_ENABLE_DELAY_US);
  Serial2.write(req_cache[imu_idx], MODBUS_REQUEST_LEN);
  Serial2.flush();
  delayMicroseconds(TX_DISABLE_DELAY_US);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  last_bus_activity_us = micros();
  perf_stats.total_requests++;
}

// ================= Frame extraction =================
static bool tryExtractFrame(uint8_t slave_id, uint8_t *frame_out) {
  size_t available = rxCount();
  if (available < RESPONSE_LEN) return false;
  
  for (size_t start_pos = 0; start_pos <= available - RESPONSE_LEN; ++start_pos) {
    size_t idx0 = (rx_tail + start_pos) % RX_RING_SZ;
    size_t idx1 = (idx0 + 1) % RX_RING_SZ;
    size_t idx2 = (idx0 + 2) % RX_RING_SZ;
    
    if (rx_ring[idx0] == slave_id && 
        rx_ring[idx1] == MODBUS_FUNC_READ_HREG && 
        rx_ring[idx2] == RESPONSE_BYTE_COUNT) {
      
      for (size_t i = 0; i < RESPONSE_LEN; ++i) {
        size_t idx = (rx_tail + start_pos + i) % RX_RING_SZ;
        frame_out[i] = rx_ring[idx];
      }
      
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

  uint16_t crc_calc = crc16_modbus(buf, len - 2);
  uint16_t crc_recv = (uint16_t)buf[len - 2] | ((uint16_t)buf[len - 1] << 8);
  if (crc_calc != crc_recv) {
    perf_stats.crc_errors++;
    return false;
  }

  const size_t data_start = 3;

  int16_t ax = (int16_t)((buf[data_start] << 8) | buf[data_start + 1]);
  int16_t ay = (int16_t)((buf[data_start + 2] << 8) | buf[data_start + 3]);
  int16_t az = (int16_t)((buf[data_start + 4] << 8) | buf[data_start + 5]);

  out.acc[0] = (float)ax * ACC_SCALE;
  out.acc[1] = (float)ay * ACC_SCALE;
  out.acc[2] = (float)az * ACC_SCALE;

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
static void outputImuData() {
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) Serial.print(",");
    if (imu_data[i].valid) {
      Serial.print(imu_data[i].acc[0], 3);  Serial.print(",");
      Serial.print(imu_data[i].acc[1], 3);  Serial.print(",");
      Serial.print(imu_data[i].acc[2], 3);  Serial.print(",");
      Serial.print(imu_data[i].quat[0], 4); Serial.print(",");
      Serial.print(imu_data[i].quat[1], 4); Serial.print(",");
      Serial.print(imu_data[i].quat[2], 4); Serial.print(",");
      Serial.print(imu_data[i].quat[3], 4);
    } else {
      Serial.print("0.000,0.000,0.000,0.0000,0.0000,0.0000,0.0000");
    }
  }
  Serial.println();
}

// ================= Performance Reporting =================
static void printStats() {
  Serial.println("\n========== Performance Statistics ==========");
  
  uint32_t elapsed_ms = millis() - perf_stats.last_stats_ms;
  if (elapsed_ms == 0) elapsed_ms = 1;
  
  float sample_rate = (perf_stats.successful_reads * 1000.0f) / elapsed_ms;
  float request_rate = (perf_stats.total_requests * 1000.0f) / elapsed_ms;
  float success_rate = perf_stats.total_requests > 0 ? 
    (perf_stats.successful_reads * 100.0f) / perf_stats.total_requests : 0.0f;
  
  Serial.print("Sample Rate: ");
  Serial.print(sample_rate, 1);
  Serial.println(" Hz");
  
  Serial.print("Request Rate: ");
  Serial.print(request_rate, 1);
  Serial.println(" Hz");
  
  Serial.print("Success Rate: ");
  Serial.print(success_rate, 1);
  Serial.println(" %");
  
  Serial.print("Total Requests: ");
  Serial.println(perf_stats.total_requests);
  
  Serial.print("Successful: ");
  Serial.println(perf_stats.successful_reads);
  
  Serial.print("Timeouts: ");
  Serial.println(perf_stats.timeouts);
  
  Serial.print("CRC Errors: ");
  Serial.println(perf_stats.crc_errors);
  
  Serial.print("Loop Time (min/max/last): ");
  Serial.print(perf_stats.min_loop_us);
  Serial.print(" / ");
  Serial.print(perf_stats.max_loop_us);
  Serial.print(" / ");
  Serial.print(perf_stats.last_loop_us);
  Serial.println(" μs");
  
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    Serial.print("IMU");
    Serial.print(IMU_IDS[i]);
    Serial.print(" - Fail Streak: ");
    Serial.print(imu_fail_streak[i]);
    Serial.print(", Valid: ");
    Serial.println(imu_data[i].valid ? "YES" : "NO");
  }
  
  Serial.println("============================================\n");
  
  // Reset counters
  perf_stats.total_requests = 0;
  perf_stats.successful_reads = 0;
  perf_stats.timeouts = 0;
  perf_stats.crc_errors = 0;
  perf_stats.min_loop_us = 999999;
  perf_stats.max_loop_us = 0;
  perf_stats.last_stats_ms = millis();
}

// ================= Scheduling =================
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

// ================= Arduino =================
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

  buildAllRequests();
  perf_stats.last_stats_ms = millis();
  
  Serial.println("IMU 200Hz Monitor - Starting...");
  Serial.println("Performance stats will print every second");
  Serial.println("==========================================\n");
}

void loop() {
  uint32_t loop_start_us = micros();
  static uint8_t response_buf[RESPONSE_LEN];
  
  // 1) 批接收
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
      outputImuData();
      waiting_response = false;
      current_imu_index = (current_imu_index + 1) % NUM_IMUS;
    } 
    else if (millis() - request_start_ms > RESPONSE_TIMEOUT_MS) {
      zeroImu(imu_data[current_imu_index]);
      recordFailure(current_imu_index);
      clearAllRx();
      waiting_response = false;
      current_imu_index = (current_imu_index + 1) % NUM_IMUS;
      perf_stats.timeouts++;
    }
  }

  // 3) 发送新请求
  if (!waiting_response && busIsIdle()) {
    uint32_t now_ms = millis();
    if (pickNextImu(now_ms)) {
      sendRequestCached(current_imu_index);
      waiting_response = true;
      request_start_ms = now_ms;
      clearAllRx();
    }
  }

  // 4) 性能统计
  perf_stats.loop_count++;
  uint32_t loop_time_us = micros() - loop_start_us;
  perf_stats.last_loop_us = loop_time_us;
  if (loop_time_us < perf_stats.min_loop_us) perf_stats.min_loop_us = loop_time_us;
  if (loop_time_us > perf_stats.max_loop_us) perf_stats.max_loop_us = loop_time_us;
  
  if (millis() - perf_stats.last_stats_ms >= STATS_INTERVAL_MS) {
    printStats();
  }
}
