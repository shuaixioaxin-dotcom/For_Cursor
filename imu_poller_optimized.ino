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

// ================= 优化的时序参数 =================
// 原值: RESPONSE_TIMEOUT_MS = 100
// 优化: 增加到150ms，给设备更多响应时间
static const uint16_t RESPONSE_TIMEOUT_MS = 150;

// 原值: BUS_SILENCE_US = 500 (0.5ms)
// 优化: 增加到1500us (1.5ms)，确保总线完全静默
// 根据Modbus RTU标准，应至少等待3.5个字符时间
// 在921600波特率下，1字符约10.85us，3.5字符约38us
// 但考虑到硬件延迟，增加到1.5ms更安全
static const uint32_t BUS_SILENCE_US = 1500;

// 原值: TX_ENABLE_DELAY_US = 30
// 优化: 增加到100us，确保RS485驱动器完全进入发送模式
static const uint32_t TX_ENABLE_DELAY_US = 100;

// 原值: TX_DISABLE_DELAY_US = 60
// 优化: 增加到150us，确保最后一个字节完全发送完毕
static const uint32_t TX_DISABLE_DELAY_US = 150;

// 新增: 请求前额外延迟，避免总线冲突
static const uint32_t PRE_REQUEST_DELAY_US = 200;

// 新增: 快速重试机制
static const uint8_t MAX_IMMEDIATE_RETRIES = 2;  // 失败后立即重试2次
static const uint32_t RETRY_DELAY_MS = 5;        // 重试间隔5ms

// ================= Data Conversion =================
static const float ACC_SCALE = 0.0048828f;
static const float QUAT_SCALE = 0.0001f;

// ================= Frequency Monitoring =================
static const uint32_t FREQ_REPORT_INTERVAL_MS = 1000;

// ================= State Machine & Scheduling =================
enum State {
  STATE_IDLE,
  STATE_PRE_REQUEST_DELAY,  // 新增：请求前延迟状态
  STATE_SENDING_REQUEST,
  STATE_WAITING_RESPONSE,
  STATE_PROCESSING_DATA,
  STATE_RETRY_DELAY         // 新增：重试延迟状态
};

struct ImuData {
  float acc[3];
  float quat[4];
  bool valid;
  uint32_t last_update_ms;
  uint32_t next_poll_ms;
  uint8_t fail_streak;
  uint8_t retry_count;  // 新增：当前重试次数
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
uint32_t retry_success_count = 0;  // 新增：重试成功计数
uint32_t last_bus_activity_us = 0;
uint32_t freq_start_ms = 0;

// Debug flags - 设为false以减少输出
static const bool DEBUG_SCHEDULING = false;
static const bool DEBUG_RAW_RESPONSES = false;
static const bool DEBUG_PARSING = false;
static const bool DEBUG_RETRY = true;  // 新增：重试调试信息

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

// 优化的缓冲区清理函数
static void clearRxBuffer() {
  // 多次读取确保缓冲区完全清空
  uint32_t start = millis();
  while (Serial2.available() > 0 && (millis() - start) < 10) {
    Serial2.read();
  }
  // 额外延迟确保硬件缓冲区清空
  delayMicroseconds(100);
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
    Serial.printf("Sending to IMU %d (Attempt %d): ", slave_id, imu_data[current_imu_index].retry_count + 1);
    for (uint8_t i = 0; i < MODBUS_REQUEST_LEN; i++) {
      Serial.printf("%02X ", request[i]);
    }
    Serial.println();
  }

  // 彻底清空接收缓冲区
  clearRxBuffer();
  
  // 确保总线静默
  while (!busIsIdle()) {
    delayMicroseconds(100);
  }
  
  // 切换到发送模式
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(TX_ENABLE_DELAY_US);
  
  // 发送数据
  Serial2.write(request, sizeof(request));
  Serial2.flush();  // 等待发送完成
  
  // 切换到接收模式前等待
  delayMicroseconds(TX_DISABLE_DELAY_US);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  
  // 额外延迟确保模式切换完成
  delayMicroseconds(50);
  
  last_bus_activity_us = micros();
}

bool parseResponse(uint8_t slave_id, const uint8_t *buf, size_t len, ImuData &out) {
  if (len != RESPONSE_LEN) {
    if (DEBUG_PARSING) {
      Serial.printf("Parse error: Wrong length %d (expected %d)\n", len, RESPONSE_LEN);
    }
    return false;
  }
  if (buf[0] != slave_id || buf[1] != MODBUS_FUNC_READ_HREG) {
    if (DEBUG_PARSING) {
      Serial.printf("Parse error: Wrong header ID=%d FUNC=%02X\n", buf[0], buf[1]);
    }
    return false;
  }
  if (buf[2] != RESPONSE_BYTE_COUNT) {
    if (DEBUG_PARSING) {
      Serial.printf("Parse error: Wrong byte count %d (expected %d)\n", buf[2], RESPONSE_BYTE_COUNT);
    }
    return false;
  }

  uint16_t crc_calc = crc16_modbus(buf, len - 2);
  uint16_t crc_recv = static_cast<uint16_t>(buf[len - 2]) |
                      static_cast<uint16_t>(buf[len - 1] << 8);
  if (crc_calc != crc_recv) {
    if (DEBUG_PARSING) {
      Serial.printf("Parse error: CRC mismatch calc=%04X recv=%04X\n", crc_calc, crc_recv);
    }
    crc_error_count++;
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

  // 解析四元数 - 修正后的逻辑
  if (buf[2] < 8) {
    // 字节数不足，没有四元数数据
    out.quat[0] = 0.0f;
    out.quat[1] = 0.0f;
    out.quat[2] = 0.0f;
    out.quat[3] = 0.0f;
  } else {
    // 有四元数数据，从加速度数据后开始（偏移16字节）
    size_t quat_offset = data_start + (buf[2] - 8);
    
    // 安全检查：确保有足够的字节
    if (quat_offset + 8 > len - 2) {
      if (DEBUG_PARSING) {
        Serial.printf("Parse error: Not enough bytes for quaternion\n");
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
  // 如果是重试后成功，记录统计
  if (imu_data[idx].retry_count > 0 && DEBUG_RETRY) {
    Serial.printf("IMU %d: Success after %d retries\n", IMU_IDS[idx], imu_data[idx].retry_count);
    retry_success_count++;
  }
  
  imu_data[idx].fail_streak = 0;
  imu_data[idx].retry_count = 0;  // 重置重试计数
  
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
  
  Serial.printf("\n--- Statistics (Optimized) ---\n");
  Serial.printf("Update Frequency: %.2f Hz\n", freq);
  Serial.printf("Total Cycles: %lu\n", cycle_count);
  Serial.printf("Success: %lu (Retry Success: %lu), Fail: %lu, Timeout: %lu, CRC Error: %lu\n",
                success_count, retry_success_count, fail_count, timeout_count, crc_error_count);
  
  for(uint8_t i=0; i<NUM_IMUS; ++i) {
    Serial.printf("IMU %d: Fail Streak=%d, Next Poll in %lu ms\n", 
                  IMU_IDS[i], imu_data[i].fail_streak, 
                  imu_data[i].next_poll_ms > millis() ? imu_data[i].next_poll_ms - millis() : 0);
  }
  Serial.printf("------------------------------\n\n");

  cycle_count = 0;
  success_count = 0;
  fail_count = 0;
  timeout_count = 0;
  crc_error_count = 0;
  retry_success_count = 0;
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
  }

  Serial.println("# Optimized Modbus RTU IMU Poller - Enhanced Stability Version");
  Serial.println("# Optimizations:");
  Serial.printf("#   - Bus silence: %lu us (was 500 us)\n", BUS_SILENCE_US);
  Serial.printf("#   - TX enable delay: %lu us (was 30 us)\n", TX_ENABLE_DELAY_US);
  Serial.printf("#   - TX disable delay: %lu us (was 60 us)\n", TX_DISABLE_DELAY_US);
  Serial.printf("#   - Response timeout: %u ms (was 100 ms)\n", RESPONSE_TIMEOUT_MS);
  Serial.printf("#   - Max retries: %u with %lu ms delay\n", MAX_IMMEDIATE_RETRIES, RETRY_DELAY_MS);
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

      if (found_ready && busIsIdle()) {
        // 进入请求前延迟状态
        current_state = STATE_PRE_REQUEST_DELAY;
        state_start_ms = micros();  // 使用微秒级时间
      }
      break;
    }

    case STATE_PRE_REQUEST_DELAY: {
      // 请求前额外延迟，确保总线完全稳定
      if ((micros() - state_start_ms) >= PRE_REQUEST_DELAY_US) {
        current_state = STATE_SENDING_REQUEST;
        state_start_ms = now_ms;
      }
      break;
    }

    case STATE_SENDING_REQUEST:
      response_pos = 0;
      sendRequest(IMU_IDS[current_imu_index]);
      current_state = STATE_WAITING_RESPONSE;
      state_start_ms = now_ms;
      break;

    case STATE_WAITING_RESPONSE: {
      if (now_ms - state_start_ms > RESPONSE_TIMEOUT_MS) {
        // 超时处理
        timeout_count++;
        
        if (DEBUG_RETRY) {
          Serial.printf("IMU %d: Timeout (Attempt %d/%d)\n", 
                       IMU_IDS[current_imu_index], 
                       imu_data[current_imu_index].retry_count + 1,
                       MAX_IMMEDIATE_RETRIES + 1);
        }
        
        // 检查是否需要重试
        if (imu_data[current_imu_index].retry_count < MAX_IMMEDIATE_RETRIES) {
          imu_data[current_imu_index].retry_count++;
          current_state = STATE_RETRY_DELAY;
          state_start_ms = now_ms;
        } else {
          // 重试次数用尽，记录失败
          fail_count++;
          recordFailure(current_imu_index);
          imu_data[current_imu_index].retry_count = 0;
          current_state = STATE_IDLE;
        }
        break;
      }

      // 接收数据
      while (Serial2.available() > 0 && response_pos < RESPONSE_LEN) {
        uint8_t byte_in = Serial2.read();
        last_bus_activity_us = micros();

        // 帧同步逻辑
        if (response_pos == 0 && byte_in != IMU_IDS[current_imu_index]) continue;
        if (response_pos == 1 && byte_in != MODBUS_FUNC_READ_HREG) { response_pos = 0; continue; }
        if (response_pos == 2 && byte_in != RESPONSE_BYTE_COUNT) { response_pos = 0; continue; }
        
        response_buf[response_pos++] = byte_in;
      }

      if (response_pos >= RESPONSE_LEN) {
        current_state = STATE_PROCESSING_DATA;
      }
      break;
    }

    case STATE_RETRY_DELAY: {
      // 重试前等待
      if (now_ms - state_start_ms >= RETRY_DELAY_MS) {
        current_state = STATE_SENDING_REQUEST;
      }
      break;
    }

    case STATE_PROCESSING_DATA: {
      bool parse_ok = parseResponse(IMU_IDS[current_imu_index], response_buf, response_pos, imu_data[current_imu_index]);
      
      if (parse_ok) {
        success_count++;
        recordSuccess(current_imu_index);
        cycle_count++;
        outputCycleCsv();
        
        // 轮询到下一个设备
        current_imu_index = (current_imu_index + 1) % NUM_IMUS;
        current_state = STATE_IDLE;
      } else {
        // 解析失败，检查是否需要重试
        if (imu_data[current_imu_index].retry_count < MAX_IMMEDIATE_RETRIES) {
          imu_data[current_imu_index].retry_count++;
          
          if (DEBUG_RETRY) {
            Serial.printf("IMU %d: Parse failed, retrying (%d/%d)\n",
                         IMU_IDS[current_imu_index],
                         imu_data[current_imu_index].retry_count,
                         MAX_IMMEDIATE_RETRIES);
          }
          
          current_state = STATE_RETRY_DELAY;
          state_start_ms = now_ms;
        } else {
          // 重试次数用尽
          fail_count++;
          recordFailure(current_imu_index);
          imu_data[current_imu_index].retry_count = 0;
          
          // 仍然输出数据（全0）
          cycle_count++;
          outputCycleCsv();
          
          current_imu_index = (current_imu_index + 1) % NUM_IMUS;
          current_state = STATE_IDLE;
        }
      }
      break;
    }
  }

  reportStatistics();
}
