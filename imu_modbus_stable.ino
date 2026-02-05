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
const uint16_t IMU_POLL_INTERVAL_MS[NUM_IMUS] = {0, 0}; 

// ================= Modbus RTU Configuration =================
static const uint8_t MODBUS_FUNC_READ_HREG = 0x03;
static const uint16_t MODBUS_REG_START = 0x0034;
static const uint16_t MODBUS_REG_COUNT = 0x0016;
static const uint8_t MODBUS_REQUEST_LEN = 8;
static const uint16_t RESPONSE_BYTE_COUNT = MODBUS_REG_COUNT * 2;
static const uint16_t RESPONSE_LEN = 1 + 1 + 1 + RESPONSE_BYTE_COUNT + 2;

// ================= 优化1: 增加超时和时序参数 =================
// 原值: RESPONSE_TIMEOUT_MS = 100, 增加到150ms提高容错
static const uint16_t RESPONSE_TIMEOUT_MS = 150;

// 原值: BUS_SILENCE_US = 500, 增加到800us确保总线稳定
// 在921600波特率下，1字节约11us，800us约73字节时间，足够安全
static const uint32_t BUS_SILENCE_US = 800;

// 原值: TX_ENABLE_DELAY_US = 30, 增加到50us
static const uint32_t TX_ENABLE_DELAY_US = 50;

// 原值: TX_DISABLE_DELAY_US = 60, 增加到150us
// 这是关键改进：确保最后一个字节完全发送后再切换到接收模式
static const uint32_t TX_DISABLE_DELAY_US = 150;

// ================= 优化2: 添加重试机制 =================
static const uint8_t MAX_RETRY_COUNT = 2;  // 最多重试2次（共3次尝试）
static const uint32_t RETRY_DELAY_US = 300; // 重试前等待300us

// ================= 优化3: 帧间延迟 =================
// 在两个IMU请求之间增加延迟，避免总线冲突
static const uint32_t INTER_FRAME_DELAY_US = 500;

// ================= Data Conversion =================
static const float ACC_SCALE = 0.0048828f;
static const float QUAT_SCALE = 0.0001f;

// ================= Frequency Monitoring =================
static const uint32_t FREQ_REPORT_INTERVAL_MS = 1000;

// ================= State Machine & Scheduling =================
enum State {
  STATE_IDLE,
  STATE_INTER_FRAME_DELAY,  // 新增：帧间延迟状态
  STATE_SENDING_REQUEST,
  STATE_WAITING_RESPONSE,
  STATE_PROCESSING_DATA
};

struct ImuData {
  float acc[3];
  float quat[4];
  bool valid;
  uint32_t last_update_ms;
  uint32_t next_poll_ms;
  uint8_t fail_streak;
};

ImuData imu_data[NUM_IMUS];
State current_state = STATE_IDLE;
uint8_t current_imu_index = 0;
uint32_t state_start_ms = 0;
uint32_t state_start_us = 0;  // 用于微秒级延迟
uint8_t response_buf[RESPONSE_LEN + 16];  // 优化4: 增加缓冲区余量
uint16_t response_pos = 0;
uint8_t retry_count = 0;  // 当前重试次数

// Statistics
uint32_t cycle_count = 0;
uint32_t success_count = 0;
uint32_t fail_count = 0;
uint32_t timeout_count = 0;
uint32_t crc_error_count = 0;
uint32_t retry_success_count = 0;  // 新增：重试后成功的次数
uint32_t last_bus_activity_us = 0;
uint32_t freq_start_ms = 0;

// Debug flags - 设为false以减少输出
static const bool DEBUG_SCHEDULING = false;
static const bool DEBUG_RAW_RESPONSES = false;
static const bool DEBUG_PARSING = false;
static const bool DEBUG_RETRY = false;  // 重试调试信息

// ================= Utility Functions =================
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
  // 优化5: 增加短延迟确保所有数据到达
  delayMicroseconds(50);
  while (Serial2.available() > 0) {
    Serial2.read();
  }
}

static bool busIsIdle() {
  return (micros() - last_bus_activity_us) >= BUS_SILENCE_US;
}

static void zeroImu(ImuData &data) {
  data.acc[0] = 0.0f; data.acc[1] = 0.0f; data.acc[2] = 0.0f;
  data.quat[0] = 0.0f; data.quat[1] = 0.0f; data.quat[2] = 0.0f; data.quat[3] = 0.0f;
  data.valid = false;
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

// ================= Core Logic Functions =================

void sendRequest(uint8_t slave_id) {
  uint8_t request[MODBUS_REQUEST_LEN];
  buildRequest(slave_id, request);

  if (DEBUG_RAW_RESPONSES) {
    Serial.printf("Sending to IMU %d: ", slave_id);
    for (uint8_t i = 0; i < MODBUS_REQUEST_LEN; i++) {
      Serial.printf("%02X ", request[i]);
    }
    Serial.println();
  }

  clearRxBuffer();
  
  // 优化6: 更稳健的RS485发送流程
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(TX_ENABLE_DELAY_US);
  
  // 分段发送，每段后检查
  Serial2.write(request, sizeof(request));
  Serial2.flush();  // 确保所有数据发送完成
  
  // 优化7: 增加额外延迟确保最后一个字节完全发送
  // 在921600波特率下，1字节约11us，加上安全余量
  delayMicroseconds(TX_DISABLE_DELAY_US);
  
  digitalWrite(RS485_DE_RE_PIN, LOW);
  last_bus_activity_us = micros();
}

// 优化8: 改进的响应解析函数，不依赖早期过滤
bool parseResponse(uint8_t slave_id, const uint8_t *buf, size_t len, ImuData &out) {
  if (len < RESPONSE_LEN) {
    if (DEBUG_PARSING) {
      Serial.printf("Parse fail: len=%d, expected=%d\n", len, RESPONSE_LEN);
    }
    return false;
  }
  
  // 在缓冲区中查找有效帧起始位置
  size_t start_pos = 0;
  bool found_valid_start = false;
  
  for (size_t i = 0; i <= len - RESPONSE_LEN; i++) {
    if (buf[i] == slave_id && buf[i+1] == MODBUS_FUNC_READ_HREG && buf[i+2] == RESPONSE_BYTE_COUNT) {
      start_pos = i;
      found_valid_start = true;
      break;
    }
  }
  
  if (!found_valid_start) {
    if (DEBUG_PARSING) {
      Serial.printf("Parse fail: no valid frame header found\n");
    }
    return false;
  }
  
  const uint8_t *frame = buf + start_pos;
  size_t frame_len = len - start_pos;
  
  if (frame_len < RESPONSE_LEN) {
    return false;
  }

  uint16_t crc_calc = crc16_modbus(frame, RESPONSE_LEN - 2);
  uint16_t crc_recv = static_cast<uint16_t>(frame[RESPONSE_LEN - 2]) |
                      static_cast<uint16_t>(frame[RESPONSE_LEN - 1] << 8);
  if (crc_calc != crc_recv) {
    crc_error_count++;
    if (DEBUG_PARSING) {
      Serial.printf("CRC fail: calc=%04X, recv=%04X\n", crc_calc, crc_recv);
    }
    return false;
  }

  const size_t data_start = 3;
  
  // 解析加速度
  int16_t ax = static_cast<int16_t>((frame[data_start] << 8) | frame[data_start + 1]);
  int16_t ay = static_cast<int16_t>((frame[data_start + 2] << 8) | frame[data_start + 3]);
  int16_t az = static_cast<int16_t>((frame[data_start + 4] << 8) | frame[data_start + 5]);

  out.acc[0] = static_cast<float>(ax) * ACC_SCALE;
  out.acc[1] = static_cast<float>(ay) * ACC_SCALE;
  out.acc[2] = static_cast<float>(az) * ACC_SCALE;

  // 解析四元数
  if (frame[2] < 8) {
    out.quat[0] = 0.0f;
    out.quat[1] = 0.0f;
    out.quat[2] = 0.0f;
    out.quat[3] = 0.0f;
  } else {
    size_t quat_offset = data_start + (frame[2] - 8);
    
    if (quat_offset + 8 > RESPONSE_LEN - 2) {
      return false;
    }
    
    int16_t qw = static_cast<int16_t>((frame[quat_offset] << 8) | frame[quat_offset + 1]);
    int16_t qx = static_cast<int16_t>((frame[quat_offset + 2] << 8) | frame[quat_offset + 3]);
    int16_t qy = static_cast<int16_t>((frame[quat_offset + 4] << 8) | frame[quat_offset + 5]);
    int16_t qz = static_cast<int16_t>((frame[quat_offset + 6] << 8) | frame[quat_offset + 7]);

    out.quat[0] = static_cast<float>(qw) * QUAT_SCALE;
    out.quat[1] = static_cast<float>(qx) * QUAT_SCALE;
    out.quat[2] = static_cast<float>(qy) * QUAT_SCALE;
    out.quat[3] = static_cast<float>(qz) * QUAT_SCALE;
  }

  out.valid = true;
  out.last_update_ms = millis();
  return true;
}

void recordSuccess(uint8_t idx) {
  imu_data[idx].fail_streak = 0;
  if (IMU_POLL_INTERVAL_MS[idx] > 0) {
    imu_data[idx].next_poll_ms = millis() + IMU_POLL_INTERVAL_MS[idx];
  } else {
    imu_data[idx].next_poll_ms = 0;
  }
}

void recordFailure(uint8_t idx) {
  if (imu_data[idx].fail_streak < 4) {
    imu_data[idx].fail_streak++;
  }
  uint32_t backoff = 20 << imu_data[idx].fail_streak;
  imu_data[idx].next_poll_ms = millis() + backoff;
  zeroImu(imu_data[idx]);
}

void outputCycleCsv() {
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) Serial.print(",");
    Serial.print(IMU_IDS[i]);
    Serial.print(",");
    if (imu_data[i].valid) {
      Serial.printf("%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f",
                    imu_data[i].acc[0], imu_data[i].acc[1], imu_data[i].acc[2],
                    imu_data[i].quat[0], imu_data[i].quat[1], imu_data[i].quat[2], imu_data[i].quat[3]);
    } else {
      Serial.print("0.000,0.000,0.000,0.0000,0.0000,0.0000,0.0000");
    }
  }
  Serial.println();
}

void reportStatistics() {
  uint32_t now = millis();
  static uint32_t last_report_ms = 0;
  
  if (now - last_report_ms < FREQ_REPORT_INTERVAL_MS) return;
  
  last_report_ms = now;

  float freq = 0.0f;
  if (cycle_count > 0) {
    uint32_t elapsed = now - freq_start_ms;
    if (elapsed > 0) {
      freq = (1000.0f * cycle_count) / elapsed;
    }
  }
  
  float success_rate = (cycle_count > 0) ? (100.0f * success_count / cycle_count) : 0.0f;
  
  Serial.printf("\n--- Statistics ---\n");
  Serial.printf("Update Frequency: %.2f Hz\n", freq);
  Serial.printf("Total Cycles: %lu, Success Rate: %.1f%%\n", cycle_count, success_rate);
  Serial.printf("Success: %lu (Retry Success: %lu), Fail: %lu, Timeout: %lu, CRC Error: %lu\n",
                success_count, retry_success_count, fail_count, timeout_count, crc_error_count);
  
  for(uint8_t i=0; i<NUM_IMUS; ++i) {
    Serial.printf("IMU %d: Fail Streak=%d, Next Poll in %lu ms\n", 
                  IMU_IDS[i], imu_data[i].fail_streak, 
                  imu_data[i].next_poll_ms > millis() ? imu_data[i].next_poll_ms - millis() : 0);
  }
  Serial.printf("------------------\n\n");

  cycle_count = 0;
  success_count = 0;
  retry_success_count = 0;
  fail_count = 0;
  timeout_count = 0;
  crc_error_count = 0;
  freq_start_ms = millis();
}

// ================= Setup & Loop =================

void setup() {
  Serial.begin(SERIAL_BAUD);
  
  // 优化9: 增加串口缓冲区大小
  Serial2.setRxBufferSize(256);
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    zeroImu(imu_data[i]);
    imu_data[i].next_poll_ms = 0;
    imu_data[i].fail_streak = 0;
  }

  Serial.println("# Optimized Modbus RTU IMU Poller - Stability Enhanced Version");
  Serial.println("# Key Optimizations:");
  Serial.println("#   - Increased TX_DISABLE_DELAY: 60us -> 150us");
  Serial.println("#   - Added retry mechanism: max 2 retries per request");
  Serial.println("#   - Improved frame detection in response buffer");
  Serial.println("#   - Added inter-frame delay: 500us");
  Serial.println("#   - Increased RX buffer size: 256 bytes");
  Serial.print("# IMU IDs: ");
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    Serial.printf("%d", IMU_IDS[i]);
    if (i < NUM_IMUS - 1) Serial.print(",");
  }
  Serial.println();
  Serial.println("# CSV: id,accx,accy,accz,qw,qx,qy,qz (repeated for each IMU)");
  Serial.println("# System ready\n");
  
  current_state = STATE_IDLE;
  state_start_ms = millis();
  freq_start_ms = millis();
}

void loop() {
  uint32_t now_ms = millis();
  uint32_t now_us = micros();

  switch (current_state) {
    case STATE_IDLE: {
      bool found_ready = false;
      
      // 轮询调度：按顺序检查每个IMU
      for (uint8_t offset = 0; offset < NUM_IMUS; offset++) {
        uint8_t idx = (current_imu_index + offset) % NUM_IMUS;
        bool is_ready = (imu_data[idx].next_poll_ms == 0) || (now_ms >= imu_data[idx].next_poll_ms);
        
        if (is_ready) {
          current_imu_index = idx;
          found_ready = true;
          break;
        }
      }

      if (found_ready && busIsIdle()) {
        retry_count = 0;  // 重置重试计数
        current_state = STATE_SENDING_REQUEST;
        state_start_ms = now_ms;
      }
      break;
    }

    case STATE_INTER_FRAME_DELAY:
      // 帧间延迟状态
      if ((now_us - state_start_us) >= INTER_FRAME_DELAY_US) {
        // 轮询到下一个设备
        current_imu_index = (current_imu_index + 1) % NUM_IMUS;
        current_state = STATE_IDLE;
      }
      break;

    case STATE_SENDING_REQUEST:
      response_pos = 0;
      memset(response_buf, 0, sizeof(response_buf));  // 清空缓冲区
      sendRequest(IMU_IDS[current_imu_index]);
      current_state = STATE_WAITING_RESPONSE;
      state_start_ms = now_ms;
      break;

    case STATE_WAITING_RESPONSE:
      if (now_ms - state_start_ms > RESPONSE_TIMEOUT_MS) {
        timeout_count++;
        
        // 优化10: 超时后尝试重试
        if (retry_count < MAX_RETRY_COUNT) {
          retry_count++;
          if (DEBUG_RETRY) {
            Serial.printf("Timeout for IMU %d, retry %d/%d\n", 
                          IMU_IDS[current_imu_index], retry_count, MAX_RETRY_COUNT);
          }
          delayMicroseconds(RETRY_DELAY_US);
          current_state = STATE_SENDING_REQUEST;
        } else {
          recordFailure(current_imu_index);
          current_state = STATE_INTER_FRAME_DELAY;
          state_start_us = now_us;
        }
        break;
      }

      // 优化11: 改进的响应接收 - 不进行早期过滤，收集所有数据
      while (Serial2.available() > 0 && response_pos < sizeof(response_buf) - 1) {
        uint8_t byte_in = Serial2.read();
        last_bus_activity_us = micros();
        response_buf[response_pos++] = byte_in;
      }

      // 检查是否收到足够的数据
      if (response_pos >= RESPONSE_LEN) {
        current_state = STATE_PROCESSING_DATA;
      }
      break;

    case STATE_PROCESSING_DATA: {
      bool parse_success = parseResponse(IMU_IDS[current_imu_index], response_buf, response_pos, imu_data[current_imu_index]);
      
      if (parse_success) {
        success_count++;
        if (retry_count > 0) {
          retry_success_count++;  // 统计重试后成功的次数
        }
        recordSuccess(current_imu_index);
      } else {
        // 优化12: 解析失败后尝试重试
        if (retry_count < MAX_RETRY_COUNT) {
          retry_count++;
          if (DEBUG_RETRY) {
            Serial.printf("Parse fail for IMU %d, retry %d/%d\n", 
                          IMU_IDS[current_imu_index], retry_count, MAX_RETRY_COUNT);
          }
          delayMicroseconds(RETRY_DELAY_US);
          current_state = STATE_SENDING_REQUEST;
          break;  // 跳出switch，不进入下面的代码
        }
        fail_count++;
        recordFailure(current_imu_index);
      }
      
      cycle_count++;
      outputCycleCsv();
      
      // 进入帧间延迟状态
      current_state = STATE_INTER_FRAME_DELAY;
      state_start_us = now_us;
      break;
    }
  }

  reportStatistics();
}
