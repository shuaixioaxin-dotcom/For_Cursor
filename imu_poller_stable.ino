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

// ================= 稳定性优化参数 =================
static const uint16_t RESPONSE_TIMEOUT_MS = 200;        // 增加超时时间: 100->200ms
static const uint32_t BUS_SILENCE_US = 2000;            // 增加总线空闲时间: 500->2000us
static const uint32_t TX_ENABLE_DELAY_US = 100;         // 增加发送使能延迟: 30->100us
static const uint32_t TX_DISABLE_DELAY_US = 150;        // 增加发送禁用延迟: 60->150us
static const uint32_t INTER_FRAME_DELAY_US = 1500;      // 帧间延迟
static const uint8_t MAX_RETRIES = 3;                   // 立即重试次数
static const uint32_t RX_SETTLE_DELAY_US = 50;          // RX稳定延迟

// ================= Data Conversion =================
static const float ACC_SCALE = 0.0048828f;
static const float QUAT_SCALE = 0.0001f;

// ================= Frequency Monitoring =================
static const uint32_t FREQ_REPORT_INTERVAL_MS = 1000;

// ================= State Machine & Scheduling =================
enum State {
  STATE_IDLE,
  STATE_WAITING_BUS_READY,
  STATE_SENDING_REQUEST,
  STATE_WAITING_RESPONSE,
  STATE_PROCESSING_DATA,
  STATE_RETRY_DELAY
};

struct ImuData {
  float acc[3];
  float quat[4];
  bool valid;
  uint32_t last_update_ms;
  uint32_t next_poll_ms;
  uint8_t fail_streak;
  uint8_t retry_count;          // 当前重试次数
  uint32_t total_retries;       // 总重试次数统计
};

ImuData imu_data[NUM_IMUS];
State current_state = STATE_IDLE;
uint8_t current_imu_index = 0;
uint32_t state_start_ms = 0;
uint8_t response_buf[RESPONSE_LEN];
uint16_t response_pos = 0;

// Statistics
uint32_t cycle_count = 0;
uint32_t success_count = 0;
uint32_t fail_count = 0;
uint32_t timeout_count = 0;
uint32_t crc_error_count = 0;
uint32_t frame_error_count = 0;
uint32_t last_bus_activity_us = 0;
uint32_t freq_start_ms = 0;

// Debug flags
static const bool DEBUG_SCHEDULING = false;
static const bool DEBUG_RAW_RESPONSES = false;
static const bool DEBUG_PARSING = false;
static const bool DEBUG_RETRIES = true;

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
  delay(1); // 等待可能的残留数据到达
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

  // 彻底清空接收缓冲区
  clearRxBuffer();
  
  // 切换到发送模式（增加延迟确保稳定）
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(TX_ENABLE_DELAY_US);
  
  // 发送数据
  Serial2.write(request, sizeof(request));
  Serial2.flush();
  
  // 切换回接收模式（增加延迟确保数据发送完成）
  delayMicroseconds(TX_DISABLE_DELAY_US);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  
  // 等待RX稳定
  delayMicroseconds(RX_SETTLE_DELAY_US);
  
  last_bus_activity_us = micros();
}

bool parseResponse(uint8_t slave_id, const uint8_t *buf, size_t len, ImuData &out) {
  if (len != RESPONSE_LEN) {
    if (DEBUG_PARSING) {
      Serial.printf("Parse failed: length mismatch (got %d, expected %d)\n", len, RESPONSE_LEN);
    }
    return false;
  }
  
  if (buf[0] != slave_id) {
    if (DEBUG_PARSING) {
      Serial.printf("Parse failed: slave ID mismatch (got %d, expected %d)\n", buf[0], slave_id);
    }
    return false;
  }
  
  if (buf[1] != MODBUS_FUNC_READ_HREG) {
    if (DEBUG_PARSING) {
      Serial.printf("Parse failed: function code mismatch (got 0x%02X, expected 0x%02X)\n", buf[1], MODBUS_FUNC_READ_HREG);
    }
    return false;
  }
  
  if (buf[2] != RESPONSE_BYTE_COUNT) {
    if (DEBUG_PARSING) {
      Serial.printf("Parse failed: byte count mismatch (got %d, expected %d)\n", buf[2], RESPONSE_BYTE_COUNT);
    }
    return false;
  }

  uint16_t crc_calc = crc16_modbus(buf, len - 2);
  uint16_t crc_recv = static_cast<uint16_t>(buf[len - 2]) |
                      static_cast<uint16_t>(buf[len - 1] << 8);
  if (crc_calc != crc_recv) {
    if (DEBUG_PARSING) {
      Serial.printf("Parse failed: CRC mismatch (calc 0x%04X, recv 0x%04X)\n", crc_calc, crc_recv);
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
      if (DEBUG_PARSING) {
        Serial.printf("Parse failed: insufficient data for quaternion\n");
      }
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
  imu_data[idx].retry_count = 0;
  
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
  uint32_t backoff = 50 << imu_data[idx].fail_streak; // 50, 100, 200, 400ms
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
  Serial.printf("Success: %lu, Fail: %lu, Timeout: %lu, CRC: %lu, Frame: %lu\n",
                success_count, fail_count, timeout_count, crc_error_count, frame_error_count);
  
  for(uint8_t i=0; i<NUM_IMUS; ++i) {
    Serial.printf("IMU %d: FailStreak=%d, Retries=%lu, NextPoll=%lu ms\n", 
                  IMU_IDS[i], 
                  imu_data[i].fail_streak,
                  imu_data[i].total_retries,
                  imu_data[i].next_poll_ms > millis() ? imu_data[i].next_poll_ms - millis() : 0);
  }
  Serial.printf("------------------\n\n");

  cycle_count = 0;
  success_count = 0;
  fail_count = 0;
  timeout_count = 0;
  crc_error_count = 0;
  frame_error_count = 0;
  freq_start_ms = millis();
}

// ================= Setup & Loop =================

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    zeroImu(imu_data[i]);
    imu_data[i].next_poll_ms = 0;
    imu_data[i].fail_streak = 0;
    imu_data[i].retry_count = 0;
    imu_data[i].total_retries = 0;
  }

  Serial.println("# Optimized Modbus RTU IMU Poller - Stability Enhanced Version");
  Serial.println("# Improvements: Extended timeouts, retry mechanism, enhanced bus timing");
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

      if (found_ready) {
        current_state = STATE_WAITING_BUS_READY;
        state_start_ms = now_ms;
      }
      break;
    }

    case STATE_WAITING_BUS_READY:
      // 确保总线空闲后才发送
      if (busIsIdle()) {
        current_state = STATE_SENDING_REQUEST;
        state_start_ms = now_ms;
      } else if (now_ms - state_start_ms > 50) {
        // 如果等待总线空闲超过50ms，强制清空并继续
        clearRxBuffer();
        last_bus_activity_us = micros() - BUS_SILENCE_US;
        current_state = STATE_SENDING_REQUEST;
      }
      break;

    case STATE_SENDING_REQUEST:
      response_pos = 0;
      sendRequest(IMU_IDS[current_imu_index]);
      current_state = STATE_WAITING_RESPONSE;
      state_start_ms = now_ms;
      break;

    case STATE_WAITING_RESPONSE: {
      if (now_ms - state_start_ms > RESPONSE_TIMEOUT_MS) {
        timeout_count++;
        
        if (DEBUG_RETRIES) {
          Serial.printf("# IMU %d timeout (retry %d/%d)\n", 
                       IMU_IDS[current_imu_index], 
                       imu_data[current_imu_index].retry_count + 1, 
                       MAX_RETRIES);
        }
        
        // 重试逻辑
        if (imu_data[current_imu_index].retry_count < MAX_RETRIES) {
          imu_data[current_imu_index].retry_count++;
          imu_data[current_imu_index].total_retries++;
          current_state = STATE_RETRY_DELAY;
          state_start_ms = now_ms;
        } else {
          // 重试次数用尽
          recordFailure(current_imu_index);
          imu_data[current_imu_index].retry_count = 0;
          cycle_count++;
          outputCycleCsv();
          current_imu_index = (current_imu_index + 1) % NUM_IMUS;
          current_state = STATE_IDLE;
        }
        break;
      }

      // 接收响应数据
      while (Serial2.available() > 0 && response_pos < RESPONSE_LEN) {
        uint8_t byte_in = Serial2.read();
        last_bus_activity_us = micros();

        // 帧同步：检查帧头
        if (response_pos == 0 && byte_in != IMU_IDS[current_imu_index]) {
          if (DEBUG_RAW_RESPONSES) {
            Serial.printf("# Frame sync: expected ID %d, got %d\n", IMU_IDS[current_imu_index], byte_in);
          }
          continue;
        }
        if (response_pos == 1 && byte_in != MODBUS_FUNC_READ_HREG) {
          if (DEBUG_RAW_RESPONSES) {
            Serial.printf("# Frame sync: invalid function code 0x%02X\n", byte_in);
          }
          response_pos = 0;
          frame_error_count++;
          continue;
        }
        if (response_pos == 2 && byte_in != RESPONSE_BYTE_COUNT) {
          if (DEBUG_RAW_RESPONSES) {
            Serial.printf("# Frame sync: invalid byte count %d\n", byte_in);
          }
          response_pos = 0;
          frame_error_count++;
          continue;
        }
        
        response_buf[response_pos++] = byte_in;
      }

      if (response_pos >= RESPONSE_LEN) {
        current_state = STATE_PROCESSING_DATA;
      }
      break;
    }

    case STATE_PROCESSING_DATA: {
      bool parse_ok = parseResponse(IMU_IDS[current_imu_index], response_buf, response_pos, imu_data[current_imu_index]);
      
      if (parse_ok) {
        success_count++;
        recordSuccess(current_imu_index);
        
        if (DEBUG_RETRIES && imu_data[current_imu_index].retry_count > 0) {
          Serial.printf("# IMU %d success after %d retries\n", 
                       IMU_IDS[current_imu_index], 
                       imu_data[current_imu_index].retry_count);
        }
      } else {
        crc_error_count++;
        
        if (DEBUG_RETRIES) {
          Serial.printf("# IMU %d CRC error (retry %d/%d)\n", 
                       IMU_IDS[current_imu_index], 
                       imu_data[current_imu_index].retry_count + 1, 
                       MAX_RETRIES);
        }
        
        // CRC错误也进行重试
        if (imu_data[current_imu_index].retry_count < MAX_RETRIES) {
          imu_data[current_imu_index].retry_count++;
          imu_data[current_imu_index].total_retries++;
          current_state = STATE_RETRY_DELAY;
          state_start_ms = now_ms;
          break;
        } else {
          recordFailure(current_imu_index);
        }
      }
      
      cycle_count++;
      outputCycleCsv();
      
      // 轮询到下一个设备
      current_imu_index = (current_imu_index + 1) % NUM_IMUS;
      current_state = STATE_IDLE;
      break;
    }

    case STATE_RETRY_DELAY:
      // 短暂延迟后重试（等待总线稳定）
      if (now_ms - state_start_ms >= 5) { // 5ms延迟
        clearRxBuffer();
        current_state = STATE_WAITING_BUS_READY;
        state_start_ms = now_ms;
      }
      break;
  }

  reportStatistics();
}
