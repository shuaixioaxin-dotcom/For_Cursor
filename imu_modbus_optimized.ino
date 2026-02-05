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

// ================= 优化点1: 增加总线时序参数 =================
// 原值可能太短导致RS485收发切换不稳定
static const uint16_t RESPONSE_TIMEOUT_MS = 150;      // 从100增加到150ms
static const uint32_t BUS_SILENCE_US = 1000;          // 从500增加到1000us (约1个字符时间@921600)
static const uint32_t TX_ENABLE_DELAY_US = 50;        // 从30增加到50us
static const uint32_t TX_DISABLE_DELAY_US = 100;      // 从60增加到100us
static const uint32_t POST_TX_SETTLE_US = 200;        // 新增: 发送完成后额外等待时间
static const uint32_t INTER_FRAME_DELAY_MS = 2;       // 新增: 帧间延时(ms)

// ================= 优化点2: 重试机制 =================
static const uint8_t MAX_RETRY_COUNT = 2;             // 单次轮询最大重试次数
static const uint32_t RETRY_DELAY_MS = 5;             // 重试前延时

// ================= Data Conversion =================
static const float ACC_SCALE = 0.0048828f;
static const float QUAT_SCALE = 0.0001f;

// ================= Frequency Monitoring =================
static const uint32_t FREQ_REPORT_INTERVAL_MS = 1000;

// ================= State Machine & Scheduling =================
enum State {
  STATE_IDLE,
  STATE_PRE_SEND_DELAY,       // 新增: 发送前延时状态
  STATE_SENDING_REQUEST,
  STATE_WAITING_RESPONSE,
  STATE_PROCESSING_DATA,
  STATE_RETRY_DELAY           // 新增: 重试延时状态
};

struct ImuData {
  float acc[3];
  float quat[4];
  bool valid;
  uint32_t last_update_ms;
  uint32_t next_poll_ms;
  uint8_t fail_streak;
  
  // 优化点3: 数据保持 - 保存上一次有效数据
  float last_valid_acc[3];
  float last_valid_quat[4];
  uint32_t last_valid_time_ms;
};

ImuData imu_data[NUM_IMUS];
State current_state = STATE_IDLE;
uint8_t current_imu_index = 0;
uint32_t state_start_ms = 0;
uint32_t state_start_us = 0;      // 新增: 微秒级时间戳用于精确延时
uint8_t response_buf[RESPONSE_LEN + 16];  // 增加缓冲区余量
uint16_t response_pos = 0;
uint8_t current_retry_count = 0;  // 当前重试计数

// Statistics
uint32_t cycle_count = 0;
uint32_t success_count = 0;
uint32_t fail_count = 0;
uint32_t timeout_count = 0;
uint32_t crc_error_count = 0;
uint32_t retry_success_count = 0;  // 新增: 重试成功计数
uint32_t last_bus_activity_us = 0;
uint32_t freq_start_ms = 0;

// Debug flags
static const bool DEBUG_SCHEDULING = false;
static const bool DEBUG_RAW_RESPONSES = false;
static const bool DEBUG_PARSING = false;
static const bool DEBUG_RETRY = true;           // 新增: 重试调试

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

// 优化点4: 更彻底的清空接收缓冲区
static void clearRxBuffer() {
  // 等待一小段时间让残余数据到达
  delayMicroseconds(100);
  uint32_t start = micros();
  while (Serial2.available() > 0 || (micros() - start) < 200) {
    if (Serial2.available() > 0) {
      Serial2.read();
      start = micros();  // 重置计时，确保清空所有数据
    }
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

// 优化点5: 使用上一次有效数据填充
static void useLastValidData(ImuData &data) {
  if (data.last_valid_time_ms > 0) {
    // 使用上一次有效数据，标记为插值数据
    data.acc[0] = data.last_valid_acc[0];
    data.acc[1] = data.last_valid_acc[1];
    data.acc[2] = data.last_valid_acc[2];
    data.quat[0] = data.last_valid_quat[0];
    data.quat[1] = data.last_valid_quat[1];
    data.quat[2] = data.last_valid_quat[2];
    data.quat[3] = data.last_valid_quat[3];
    data.valid = true;  // 标记为有效（使用旧数据）
  } else {
    zeroImu(data);
  }
}

// 保存有效数据
static void saveValidData(ImuData &data) {
  data.last_valid_acc[0] = data.acc[0];
  data.last_valid_acc[1] = data.acc[1];
  data.last_valid_acc[2] = data.acc[2];
  data.last_valid_quat[0] = data.quat[0];
  data.last_valid_quat[1] = data.quat[1];
  data.last_valid_quat[2] = data.quat[2];
  data.last_valid_quat[3] = data.quat[3];
  data.last_valid_time_ms = millis();
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

// 优化点6: 改进的发送函数，确保完整发送和正确的收发切换
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

  // 确保接收缓冲区为空
  clearRxBuffer();
  
  // 切换到发送模式
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(TX_ENABLE_DELAY_US);
  
  // 发送数据
  size_t written = Serial2.write(request, sizeof(request));
  
  // 等待硬件FIFO完全发送完毕
  Serial2.flush();
  
  // 额外等待，确保最后一个字节完全发送
  // 在921600波特率下，一个字节约11us，加上余量
  delayMicroseconds(POST_TX_SETTLE_US);
  
  // 切换到接收模式
  digitalWrite(RS485_DE_RE_PIN, LOW);
  delayMicroseconds(TX_DISABLE_DELAY_US);
  
  last_bus_activity_us = micros();
}

bool parseResponse(uint8_t slave_id, const uint8_t *buf, size_t len, ImuData &out) {
  if (len != RESPONSE_LEN) {
    if (DEBUG_PARSING) {
      Serial.printf("Parse fail: len=%d, expected=%d\n", len, RESPONSE_LEN);
    }
    return false;
  }
  if (buf[0] != slave_id || buf[1] != MODBUS_FUNC_READ_HREG) {
    if (DEBUG_PARSING) {
      Serial.printf("Parse fail: addr=%02X func=%02X\n", buf[0], buf[1]);
    }
    return false;
  }
  if (buf[2] != RESPONSE_BYTE_COUNT) {
    if (DEBUG_PARSING) {
      Serial.printf("Parse fail: byte_count=%d, expected=%d\n", buf[2], RESPONSE_BYTE_COUNT);
    }
    return false;
  }

  uint16_t crc_calc = crc16_modbus(buf, len - 2);
  uint16_t crc_recv = static_cast<uint16_t>(buf[len - 2]) |
                      static_cast<uint16_t>(buf[len - 1] << 8);
  if (crc_calc != crc_recv) {
    crc_error_count++;
    if (DEBUG_PARSING) {
      Serial.printf("CRC fail: calc=%04X recv=%04X\n", crc_calc, crc_recv);
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
  
  // 保存有效数据用于后续插值
  saveValidData(out);
  
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
  
  // 优化: 使用上一次有效数据而不是清零
  useLastValidData(imu_data[idx]);
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
  Serial.printf("Success: %lu, Fail: %lu, Timeout: %lu, CRC Error: %lu\n",
                success_count, fail_count, timeout_count, crc_error_count);
  Serial.printf("Retry Success: %lu\n", retry_success_count);
  
  for(uint8_t i=0; i<NUM_IMUS; ++i) {
    Serial.printf("IMU %d: Fail Streak=%d, Next Poll in %lu ms\n", 
                  IMU_IDS[i], imu_data[i].fail_streak, 
                  imu_data[i].next_poll_ms > millis() ? imu_data[i].next_poll_ms - millis() : 0);
  }
  Serial.printf("------------------\n\n");

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
  
  // 优化点7: 增加Serial2的RX缓冲区大小
  Serial2.setRxBufferSize(512);  // 默认是256，增加到512
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    zeroImu(imu_data[i]);
    imu_data[i].next_poll_ms = 0;
    imu_data[i].fail_streak = 0;
    imu_data[i].last_valid_time_ms = 0;
    imu_data[i].last_valid_acc[0] = 0.0f;
    imu_data[i].last_valid_acc[1] = 0.0f;
    imu_data[i].last_valid_acc[2] = 0.0f;
    imu_data[i].last_valid_quat[0] = 0.0f;
    imu_data[i].last_valid_quat[1] = 0.0f;
    imu_data[i].last_valid_quat[2] = 0.0f;
    imu_data[i].last_valid_quat[3] = 0.0f;
  }

  Serial.println("# Optimized Modbus RTU IMU Poller - Stability Enhanced Version");
  Serial.print("# IMU IDs: ");
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    Serial.printf("%d", IMU_IDS[i]);
    if (i < NUM_IMUS - 1) Serial.print(",");
  }
  Serial.println();
  Serial.println("# CSV: id,accx,accy,accz,qw,qx,qy,qz (repeated for each IMU)");
  Serial.printf("# Timeout: %dms, Retry: %d, Inter-frame delay: %dms\n", 
                RESPONSE_TIMEOUT_MS, MAX_RETRY_COUNT, INTER_FRAME_DELAY_MS);
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
        current_retry_count = 0;
        current_state = STATE_PRE_SEND_DELAY;
        state_start_ms = now_ms;
        state_start_us = now_us;
      }
      break;
    }
    
    // 优化点8: 发送前增加短暂延时，确保总线稳定
    case STATE_PRE_SEND_DELAY: {
      if (now_ms - state_start_ms >= INTER_FRAME_DELAY_MS) {
        current_state = STATE_SENDING_REQUEST;
        state_start_ms = now_ms;
      }
      break;
    }

    case STATE_SENDING_REQUEST:
      response_pos = 0;
      memset(response_buf, 0, sizeof(response_buf));
      sendRequest(IMU_IDS[current_imu_index]);
      current_state = STATE_WAITING_RESPONSE;
      state_start_ms = now_ms;
      state_start_us = now_us;
      break;

    case STATE_WAITING_RESPONSE:
      if (now_ms - state_start_ms > RESPONSE_TIMEOUT_MS) {
        timeout_count++;
        
        // 优化点9: 重试机制
        if (current_retry_count < MAX_RETRY_COUNT) {
          current_retry_count++;
          if (DEBUG_RETRY) {
            Serial.printf("Timeout IMU %d, retry %d/%d\n", 
                          IMU_IDS[current_imu_index], current_retry_count, MAX_RETRY_COUNT);
          }
          current_state = STATE_RETRY_DELAY;
          state_start_ms = now_ms;
        } else {
          recordFailure(current_imu_index);
          current_state = STATE_IDLE;
          // 移动到下一个IMU
          current_imu_index = (current_imu_index + 1) % NUM_IMUS;
        }
        break;
      }

      // 优化点10: 改进的接收逻辑 - 批量读取可用数据
      while (Serial2.available() > 0 && response_pos < RESPONSE_LEN) {
        uint8_t byte_in = Serial2.read();
        last_bus_activity_us = micros();

        // 帧头校验
        if (response_pos == 0) {
          if (byte_in != IMU_IDS[current_imu_index]) {
            // 跳过非目标地址的字节
            continue;
          }
        }
        
        if (response_pos == 1) {
          if (byte_in != MODBUS_FUNC_READ_HREG) {
            // 功能码不匹配，重置
            response_pos = 0;
            continue;
          }
        }
        
        if (response_pos == 2) {
          if (byte_in != RESPONSE_BYTE_COUNT) {
            // 字节数不匹配，重置
            response_pos = 0;
            continue;
          }
        }
        
        response_buf[response_pos++] = byte_in;
      }

      if (response_pos >= RESPONSE_LEN) {
        current_state = STATE_PROCESSING_DATA;
      }
      break;
      
    // 优化点11: 重试延时状态
    case STATE_RETRY_DELAY:
      if (now_ms - state_start_ms >= RETRY_DELAY_MS) {
        clearRxBuffer();  // 清空可能的残余数据
        current_state = STATE_SENDING_REQUEST;
        state_start_ms = now_ms;
      }
      break;

    case STATE_PROCESSING_DATA:
      if (parseResponse(IMU_IDS[current_imu_index], response_buf, response_pos, imu_data[current_imu_index])) {
        success_count++;
        if (current_retry_count > 0) {
          retry_success_count++;  // 统计重试成功次数
        }
        recordSuccess(current_imu_index);
      } else {
        // 解析失败，尝试重试
        if (current_retry_count < MAX_RETRY_COUNT) {
          current_retry_count++;
          if (DEBUG_RETRY) {
            Serial.printf("Parse fail IMU %d, retry %d/%d\n", 
                          IMU_IDS[current_imu_index], current_retry_count, MAX_RETRY_COUNT);
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
