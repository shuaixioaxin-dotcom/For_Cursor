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

// ================= 优化后的时序参数 =================
// 增加响应超时，给IMU更多处理时间
static const uint16_t RESPONSE_TIMEOUT_MS = 150;

// 增加总线静默时间，确保帧间隔足够
// Modbus RTU标准：3.5字符时间，在921600波特率下约38us，但实际需要更多
static const uint32_t BUS_SILENCE_US = 1000;

// 增加RS485收发切换延迟
static const uint32_t TX_ENABLE_DELAY_US = 100;   // 原30us -> 100us
static const uint32_t TX_DISABLE_DELAY_US = 150;  // 原60us -> 150us

// 请求间最小延迟（毫秒），确保总线稳定
static const uint32_t MIN_REQUEST_GAP_MS = 2;

// ================= 重试配置 =================
static const uint8_t MAX_RETRIES = 2;           // 每次请求最多重试次数
static const uint32_t RETRY_DELAY_MS = 5;       // 重试前延迟

// ================= Data Conversion =================
static const float ACC_SCALE = 0.0048828f;
static const float QUAT_SCALE = 0.0001f;

// ================= Frequency Monitoring =================
static const uint32_t FREQ_REPORT_INTERVAL_MS = 1000;

// ================= State Machine & Scheduling =================
enum State {
  STATE_IDLE,
  STATE_PRE_REQUEST_DELAY,   // 新增：请求前延迟
  STATE_SENDING_REQUEST,
  STATE_WAITING_RESPONSE,
  STATE_PROCESSING_DATA,
  STATE_RETRY_DELAY          // 新增：重试延迟
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
uint8_t response_buf[RESPONSE_LEN + 16];  // 增加缓冲区余量
uint16_t response_pos = 0;

// 重试相关
uint8_t current_retry_count = 0;
uint32_t last_request_ms = 0;

// Statistics
uint32_t cycle_count = 0;
uint32_t success_count = 0;
uint32_t fail_count = 0;
uint32_t timeout_count = 0;
uint32_t crc_error_count = 0;
uint32_t retry_count = 0;           // 新增：重试次数统计
uint32_t last_bus_activity_us = 0;
uint32_t freq_start_ms = 0;

// Debug flags - 设为false以减少输出
static const bool DEBUG_SCHEDULING = false;
static const bool DEBUG_RAW_RESPONSES = false;
static const bool DEBUG_PARSING = false;
static const bool DEBUG_RETRIES = true;  // 重试调试信息

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
  // 增加延迟确保接收完成
  delayMicroseconds(100);
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
    Serial.printf("Sending to IMU %d (retry %d): ", slave_id, current_retry_count);
    for (uint8_t i = 0; i < MODBUS_REQUEST_LEN; i++) {
      Serial.printf("%02X ", request[i]);
    }
    Serial.println();
  }

  // 确保接收缓冲区为空
  clearRxBuffer();
  
  // 切换到发送模式，增加延迟确保稳定
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(TX_ENABLE_DELAY_US);
  
  // 发送数据
  Serial2.write(request, sizeof(request));
  Serial2.flush();  // 等待发送完成
  
  // 额外等待，确保最后一个字节完全发送
  // 在921600波特率下，1字节约11us
  delayMicroseconds(TX_DISABLE_DELAY_US);
  
  // 切换到接收模式
  digitalWrite(RS485_DE_RE_PIN, LOW);
  
  // 等待RS485芯片稳定
  delayMicroseconds(50);
  
  last_bus_activity_us = micros();
  last_request_ms = millis();
}

// 改进的响应解析：更宽松的帧接收
bool receiveResponse(uint32_t timeout_ms) {
  uint32_t start_ms = millis();
  response_pos = 0;
  bool header_valid = false;
  
  while (millis() - start_ms < timeout_ms) {
    if (Serial2.available() > 0) {
      uint8_t byte_in = Serial2.read();
      last_bus_activity_us = micros();
      
      // 帧同步：寻找正确的起始
      if (response_pos == 0) {
        if (byte_in == IMU_IDS[current_imu_index]) {
          response_buf[response_pos++] = byte_in;
        }
        // 忽略非目标地址的字节，继续等待
        continue;
      }
      
      // 验证功能码
      if (response_pos == 1) {
        if (byte_in == MODBUS_FUNC_READ_HREG) {
          response_buf[response_pos++] = byte_in;
        } else if (byte_in == (MODBUS_FUNC_READ_HREG | 0x80)) {
          // Modbus异常响应
          if (DEBUG_RETRIES) {
            Serial.printf("IMU %d: Modbus exception\n", IMU_IDS[current_imu_index]);
          }
          response_pos = 0;
          continue;
        } else {
          response_pos = 0;
          continue;
        }
        continue;
      }
      
      // 验证字节数
      if (response_pos == 2) {
        if (byte_in == RESPONSE_BYTE_COUNT) {
          response_buf[response_pos++] = byte_in;
          header_valid = true;
        } else {
          response_pos = 0;
          continue;
        }
        continue;
      }
      
      // 接收数据和CRC
      if (header_valid && response_pos < RESPONSE_LEN) {
        response_buf[response_pos++] = byte_in;
        
        // 完整帧接收完成
        if (response_pos >= RESPONSE_LEN) {
          return true;
        }
      }
    }
    
    // 短暂让出CPU，避免忙等待
    delayMicroseconds(10);
  }
  
  return false;
}

bool parseResponse(uint8_t slave_id, const uint8_t *buf, size_t len, ImuData &out) {
  if (len != RESPONSE_LEN) {
    if (DEBUG_PARSING) {
      Serial.printf("Parse error: wrong length %d (expected %d)\n", len, RESPONSE_LEN);
    }
    return false;
  }
  if (buf[0] != slave_id || buf[1] != MODBUS_FUNC_READ_HREG) {
    if (DEBUG_PARSING) {
      Serial.printf("Parse error: header mismatch\n");
    }
    return false;
  }
  if (buf[2] != RESPONSE_BYTE_COUNT) {
    if (DEBUG_PARSING) {
      Serial.printf("Parse error: byte count mismatch\n");
    }
    return false;
  }

  uint16_t crc_calc = crc16_modbus(buf, len - 2);
  uint16_t crc_recv = static_cast<uint16_t>(buf[len - 2]) |
                      static_cast<uint16_t>(buf[len - 1] << 8);
  if (crc_calc != crc_recv) {
    crc_error_count++;
    if (DEBUG_PARSING) {
      Serial.printf("CRC error: calc=0x%04X recv=0x%04X\n", crc_calc, crc_recv);
    }
    return false;
  }

  const size_t data_start = 3;
  
  // 解析加速度
  int16_t ax = static_cast<int16_t>((buf[data_start] << 8) | buf[data_start + 1]);
  int16_t ay = static_cast<int16_t>((buf[data_start + 2] << 8) | buf[data_start + 3]);
  int16_t az = static_cast<int16_t>((buf[data_start + 4] << 8) | buf[data_start + 5]);

  out.acc[0] = static_cast<float>(ax) * ACC_SCALE;
  out.acc[1] = static_cast<float>(ay) * ACC_SCALE;
  out.acc[2] = static_cast<float>(az) * ACC_SCALE;

  // 解析四元数
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
  // 减小退避时间，提高响应速度
  uint32_t backoff = 10 << imu_data[idx].fail_streak;  // 原20ms -> 10ms基础
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
  
  Serial.printf("\n--- Statistics ---\n");
  Serial.printf("Update Frequency: %.2f Hz\n", freq);
  Serial.printf("Total Cycles: %lu\n", cycle_count);
  Serial.printf("Success: %lu, Fail: %lu, Timeout: %lu, CRC Error: %lu, Retries: %lu\n",
                success_count, fail_count, timeout_count, crc_error_count, retry_count);
  
  // 计算成功率
  uint32_t total_attempts = success_count + fail_count;
  if (total_attempts > 0) {
    float success_rate = (100.0f * success_count) / total_attempts;
    Serial.printf("Success Rate: %.1f%%\n", success_rate);
  }
  
  for(uint8_t i=0; i<NUM_IMUS; ++i) {
    Serial.printf("IMU %d: Fail Streak=%d, Valid=%s\n", 
                  IMU_IDS[i], imu_data[i].fail_streak,
                  imu_data[i].valid ? "Yes" : "No");
  }
  Serial.printf("------------------\n\n");

  cycle_count = 0;
  success_count = 0;
  fail_count = 0;
  timeout_count = 0;
  crc_error_count = 0;
  retry_count = 0;
  freq_start_ms = millis();
}

// ================= Setup & Loop =================

void setup() {
  Serial.begin(SERIAL_BAUD);
  
  // 配置Serial2，增加接收缓冲区
  Serial2.setRxBufferSize(512);  // 增加接收缓冲区
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    zeroImu(imu_data[i]);
    imu_data[i].next_poll_ms = 0;
    imu_data[i].fail_streak = 0;
  }

  Serial.println("# Optimized Modbus RTU IMU Poller - Stability Enhanced Version");
  Serial.println("# Improvements: Retry mechanism, increased timing margins, larger RX buffer");
  Serial.print("# IMU IDs: ");
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    Serial.printf("%d", IMU_IDS[i]);
    if (i < NUM_IMUS - 1) Serial.print(",");
  }
  Serial.println();
  Serial.printf("# Max Retries: %d, Response Timeout: %d ms\n", MAX_RETRIES, RESPONSE_TIMEOUT_MS);
  Serial.println("# CSV: id,accx,accy,accz,qw,qx,qy,qz (repeated for each IMU)");
  Serial.println("# System ready\n");
  
  current_state = STATE_IDLE;
  state_start_ms = millis();
  freq_start_ms = millis();
  last_request_ms = 0;
}

void loop() {
  uint32_t now_ms = millis();

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
        // 检查是否需要请求间延迟
        if (now_ms - last_request_ms < MIN_REQUEST_GAP_MS) {
          current_state = STATE_PRE_REQUEST_DELAY;
          state_start_ms = now_ms;
        } else {
          current_retry_count = 0;
          current_state = STATE_SENDING_REQUEST;
          state_start_ms = now_ms;
        }
      }
      break;
    }

    case STATE_PRE_REQUEST_DELAY:
      // 等待最小请求间隔
      if (now_ms - last_request_ms >= MIN_REQUEST_GAP_MS) {
        current_retry_count = 0;
        current_state = STATE_SENDING_REQUEST;
        state_start_ms = now_ms;
      }
      break;

    case STATE_SENDING_REQUEST:
      response_pos = 0;
      sendRequest(IMU_IDS[current_imu_index]);
      current_state = STATE_WAITING_RESPONSE;
      state_start_ms = now_ms;
      break;

    case STATE_WAITING_RESPONSE: {
      // 使用改进的接收函数
      bool received = false;
      
      // 检查超时
      if (now_ms - state_start_ms > RESPONSE_TIMEOUT_MS) {
        timeout_count++;
        
        // 检查是否需要重试
        if (current_retry_count < MAX_RETRIES) {
          current_retry_count++;
          retry_count++;
          if (DEBUG_RETRIES) {
            Serial.printf("IMU %d: Timeout, retry %d/%d\n", 
                          IMU_IDS[current_imu_index], current_retry_count, MAX_RETRIES);
          }
          current_state = STATE_RETRY_DELAY;
          state_start_ms = now_ms;
        } else {
          recordFailure(current_imu_index);
          current_imu_index = (current_imu_index + 1) % NUM_IMUS;
          current_state = STATE_IDLE;
        }
        break;
      }

      // 读取响应数据
      while (Serial2.available() > 0 && response_pos < RESPONSE_LEN) {
        uint8_t byte_in = Serial2.read();
        last_bus_activity_us = micros();

        // 帧同步逻辑
        if (response_pos == 0) {
          if (byte_in == IMU_IDS[current_imu_index]) {
            response_buf[response_pos++] = byte_in;
          }
          continue;
        }
        
        if (response_pos == 1) {
          if (byte_in == MODBUS_FUNC_READ_HREG) {
            response_buf[response_pos++] = byte_in;
          } else {
            response_pos = 0;
          }
          continue;
        }
        
        if (response_pos == 2) {
          if (byte_in == RESPONSE_BYTE_COUNT) {
            response_buf[response_pos++] = byte_in;
          } else {
            response_pos = 0;
          }
          continue;
        }
        
        response_buf[response_pos++] = byte_in;
      }

      if (response_pos >= RESPONSE_LEN) {
        current_state = STATE_PROCESSING_DATA;
      }
      break;
    }

    case STATE_RETRY_DELAY:
      // 重试前等待
      if (now_ms - state_start_ms >= RETRY_DELAY_MS) {
        clearRxBuffer();
        current_state = STATE_SENDING_REQUEST;
        state_start_ms = now_ms;
      }
      break;

    case STATE_PROCESSING_DATA:
      if (parseResponse(IMU_IDS[current_imu_index], response_buf, response_pos, imu_data[current_imu_index])) {
        success_count++;
        recordSuccess(current_imu_index);
        
        if (DEBUG_RETRIES && current_retry_count > 0) {
          Serial.printf("IMU %d: Success after %d retries\n", 
                        IMU_IDS[current_imu_index], current_retry_count);
        }
      } else {
        // CRC错误等，尝试重试
        if (current_retry_count < MAX_RETRIES) {
          current_retry_count++;
          retry_count++;
          if (DEBUG_RETRIES) {
            Serial.printf("IMU %d: Parse error, retry %d/%d\n", 
                          IMU_IDS[current_imu_index], current_retry_count, MAX_RETRIES);
          }
          current_state = STATE_RETRY_DELAY;
          state_start_ms = now_ms;
          break;
        }
        
        fail_count++;
        recordFailure(current_imu_index);
      }
      
      cycle_count++;
      outputCycleCsv();
      
      // 轮询到下一个设备
      current_imu_index = (current_imu_index + 1) % NUM_IMUS;
      current_state = STATE_IDLE;
      break;
  }

  reportStatistics();
}
