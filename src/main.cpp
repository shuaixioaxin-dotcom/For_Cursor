#include <Arduino.h>

// ================= Pin Configuration =================
#define RS485_RX_PIN 32     // Module RO
#define RS485_TX_PIN 33     // Module DI
#define RS485_DE_RE_PIN 25  // Module DE/RE Control

// ================= Serial Configuration =================
static const uint32_t SERIAL_BAUD = 921600;
static const uint32_t RS485_BAUD  = 921600;

// ================= IMU Configuration =================
#define NUM_IMUS 2
const uint8_t IMU_IDS[NUM_IMUS] = {1, 2};

// ================= Optimization Configuration =================
// 批量输出配置：收集BATCH_SIZE个完整样本后再输出
// 对于200Hz目标：BATCH_SIZE=4时，输出频率50Hz，采集频率200Hz
#define BATCH_SIZE 4              // 每批样本数（可调整：2-10）
#define OUTPUT_BINARY false        // true=二进制输出, false=文本输出

// ================= Modbus RTU Configuration =================
static const uint8_t  MODBUS_FUNC_READ_HREG = 0x03;
static const uint16_t MODBUS_REG_START      = 0x0034;
static const uint16_t MODBUS_REG_COUNT      = 0x0016;  // 22 registers
static const uint8_t  MODBUS_REQUEST_LEN    = 8;

static const uint16_t RESPONSE_BYTE_COUNT   = MODBUS_REG_COUNT * 2;                 // 44
static const uint16_t RESPONSE_LEN          = 1 + 1 + 1 + RESPONSE_BYTE_COUNT + 2;  // 49

// ================= Timing Optimization =================
static const uint16_t RESPONSE_TIMEOUT_MS   = 15;   // 降低超时(30->15ms)
static const uint32_t BUS_SILENCE_US        = 200;  // 降低总线静默(500->200us)
static const uint32_t TX_ENABLE_DELAY_US    = 20;   // 降低发送延迟(30->20us)
static const uint32_t TX_DISABLE_DELAY_US   = 30;   // 降低延迟(60->30us)

// ================= Data Conversion =================
static const float ACC_SCALE  = 0.0048828f;
static const float QUAT_SCALE = 0.0001f;

// ================= State =================
struct ImuData {
  float acc[3];
  float quat[4];
  bool valid;
};

// 批量缓冲区：存储BATCH_SIZE组完整的IMU数据
struct BatchBuffer {
  ImuData samples[NUM_IMUS * BATCH_SIZE];
  uint16_t count;  // 当前已收集的样本组数
} batch_buffer;

static uint8_t  current_imu_index   = 0;
static bool     waiting_response    = false;
static uint32_t request_start_ms    = 0;
static uint32_t last_bus_activity_us = 0;

// 性能统计
static uint32_t sample_count = 0;
static uint32_t last_stats_ms = 0;

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
    rxDropOne();  // 溢出时丢弃最旧数据
  }
  rx_ring[rx_head] = b;
  rx_head = next;
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

static void clearAllRx() {
  while (Serial2.available() > 0) {
    (void)Serial2.read();
  }
  rx_head = rx_tail = 0;
}

static bool busIsIdle() {
  if (last_bus_activity_us == 0) return true;
  return (micros() - last_bus_activity_us) >= BUS_SILENCE_US;
}

// ================= Cached requests =================
static uint8_t req_cache[NUM_IMUS][MODBUS_REQUEST_LEN];

static void buildAllRequests() {
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    uint8_t *out = req_cache[i];
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

static void sendRequestCached(uint8_t imu_idx) {
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
      
      // 移除处理过的数据
      for (size_t i = 0; i < start_pos + RESPONSE_LEN; ++i) {
        rxDropOne();
      }
      
      return true;
    }
  }
  
  return false;
}

static bool parseResponse(uint8_t slave_id, const uint8_t *buf, ImuData &out) {
  if (buf[0] != slave_id || buf[1] != MODBUS_FUNC_READ_HREG) return false;
  if (buf[2] != RESPONSE_BYTE_COUNT) return false;

  uint16_t crc_calc = crc16_modbus(buf, RESPONSE_LEN - 2);
  uint16_t crc_recv = (uint16_t)buf[RESPONSE_LEN - 2] | ((uint16_t)buf[RESPONSE_LEN - 1] << 8);
  if (crc_calc != crc_recv) return false;

  const size_t data_start = 3;

  int16_t ax = (int16_t)((buf[data_start] << 8) | buf[data_start + 1]);
  int16_t ay = (int16_t)((buf[data_start + 2] << 8) | buf[data_start + 3]);
  int16_t az = (int16_t)((buf[data_start + 4] << 8) | buf[data_start + 5]);

  out.acc[0] = (float)ax * ACC_SCALE;
  out.acc[1] = (float)ay * ACC_SCALE;
  out.acc[2] = (float)az * ACC_SCALE;

  // 四元数: 数据区的最后8字节
  size_t quat_offset = data_start + (buf[2] - 8);

  int16_t qw = (int16_t)((buf[quat_offset] << 8) | buf[quat_offset + 1]);
  int16_t qx = (int16_t)((buf[quat_offset + 2] << 8) | buf[quat_offset + 3]);
  int16_t qy = (int16_t)((buf[quat_offset + 4] << 8) | buf[quat_offset + 5]);
  int16_t qz = (int16_t)((buf[quat_offset + 6] << 8) | buf[quat_offset + 7]);

  out.quat[0] = (float)qw * QUAT_SCALE;
  out.quat[1] = (float)qx * QUAT_SCALE;
  out.quat[2] = (float)qy * QUAT_SCALE;
  out.quat[3] = (float)qz * QUAT_SCALE;

  out.valid = true;
  return true;
}

// ================= Batch Output =================
static void outputBatchBinary() {
  // 二进制格式：更高效，适合高频率传输
  // 格式: [0xFF][0xAA][count][IMU1_data...][IMU2_data...]...
  Serial.write(0xFF);  // 帧头
  Serial.write(0xAA);
  Serial.write((uint8_t)batch_buffer.count);
  
  for (uint16_t i = 0; i < batch_buffer.count * NUM_IMUS; ++i) {
    ImuData &d = batch_buffer.samples[i];
    if (d.valid) {
      Serial.write((uint8_t*)&d.acc[0], sizeof(float) * 3);
      Serial.write((uint8_t*)&d.quat[0], sizeof(float) * 4);
    } else {
      float zeros[7] = {0};
      Serial.write((uint8_t*)zeros, sizeof(float) * 7);
    }
  }
  Serial.write(0xBB);  // 帧尾
  Serial.write(0xCC);
}

static void outputBatchText() {
  // 文本格式：便于调试，但较慢
  // 每批输出多行，每行格式: IMU1_ax,ay,az,qw,qx,qy,qz,IMU2_ax,ay,az,qw,qx,qy,qz
  for (uint16_t sample = 0; sample < batch_buffer.count; ++sample) {
    for (uint8_t imu = 0; imu < NUM_IMUS; ++imu) {
      if (imu > 0) Serial.print(',');
      
      ImuData &d = batch_buffer.samples[sample * NUM_IMUS + imu];
      if (d.valid) {
        Serial.print(d.acc[0], 3);  Serial.print(',');
        Serial.print(d.acc[1], 3);  Serial.print(',');
        Serial.print(d.acc[2], 3);  Serial.print(',');
        Serial.print(d.quat[0], 4); Serial.print(',');
        Serial.print(d.quat[1], 4); Serial.print(',');
        Serial.print(d.quat[2], 4); Serial.print(',');
        Serial.print(d.quat[3], 4);
      } else {
        Serial.print("0.000,0.000,0.000,0.0000,0.0000,0.0000,0.0000");
      }
    }
    Serial.println();
  }
}

static void addToBatch(uint8_t imu_idx, const ImuData &data) {
  // 将数据添加到批缓冲区
  uint16_t pos = batch_buffer.count * NUM_IMUS + imu_idx;
  batch_buffer.samples[pos] = data;
  
  // 当所有IMU数据都收集完成时，增加计数
  if (imu_idx == NUM_IMUS - 1) {
    batch_buffer.count++;
    sample_count++;
    
    // 批次满，输出
    if (batch_buffer.count >= BATCH_SIZE) {
      if (OUTPUT_BINARY) {
        outputBatchBinary();
      } else {
        outputBatchText();
      }
      batch_buffer.count = 0;
    }
  }
}

// ================= Performance Stats =================
static void updateStats() {
  uint32_t now = millis();
  if (now - last_stats_ms >= 1000) {
    // 每秒输出一次频率统计（注释掉以避免干扰数据流）
    // float hz = sample_count / ((now - last_stats_ms) / 1000.0f);
    // Serial.print("# Freq: "); Serial.print(hz, 1); Serial.println(" Hz");
    sample_count = 0;
    last_stats_ms = now;
  }
}

// ================= Arduino =================
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  batch_buffer.count = 0;
  buildAllRequests();
  
  last_stats_ms = millis();
  
  delay(100);  // 启动延迟
}

void loop() {
  static uint8_t response_buf[RESPONSE_LEN];
  
  // 1) 批量接收：将UART数据读入环形缓冲区
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
      ImuData temp_data = {0};
      if (parseResponse(slave_id, response_buf, temp_data)) {
        addToBatch(current_imu_index, temp_data);
      } else {
        // 解析失败，添加无效数据
        temp_data.valid = false;
        addToBatch(current_imu_index, temp_data);
      }
      
      waiting_response = false;
      current_imu_index = (current_imu_index + 1) % NUM_IMUS;
    } 
    else if (millis() - request_start_ms > RESPONSE_TIMEOUT_MS) {
      // 超时，添加无效数据
      ImuData temp_data = {0};
      temp_data.valid = false;
      addToBatch(current_imu_index, temp_data);
      
      clearAllRx();
      waiting_response = false;
      current_imu_index = (current_imu_index + 1) % NUM_IMUS;
    }
  }

  // 3) 发送新请求
  if (!waiting_response && busIsIdle()) {
    sendRequestCached(current_imu_index);
    waiting_response = true;
    request_start_ms = millis();
  }
  
  // 4) 性能统计（可选）
  updateStats();
}
