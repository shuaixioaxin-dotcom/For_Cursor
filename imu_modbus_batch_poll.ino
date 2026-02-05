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
static const uint16_t RESPONSE_BYTE_COUNT = MODBUS_REG_COUNT * 2;               // 44
static const uint16_t RESPONSE_LEN = 1 + 1 + 1 + RESPONSE_BYTE_COUNT + 2;       // 49

// ================= Timing / Throughput Tuning =================
// 一些从机对“帧间静默”很敏感；3.5char@921600 约 42us，这里给一点余量
static const uint32_t BUS_SILENCE_US = 60;

// DE 拉高后到实际发数的稳定时间（按硬件情况 2~30us）
static const uint32_t TX_ENABLE_DELAY_US = 2;
// flush 后保持 DE 的时间，避免最后 1-2bit 被过早切到接收导致截断（按硬件情况 2~20us）
static const uint32_t TX_DISABLE_HOLD_US = 6;
// 切换到接收后，给收发器一点点稳定时间
static const uint32_t RX_SETTLE_US = 2;

// 两段式读超时：
// - 首字节等待：若从机完全不响应，尽快跳过（提升整体频率）
// - 总时长：从机开始响应后，留足时间读完整帧
static const uint32_t FIRST_BYTE_TIMEOUT_US = 800;
static const uint32_t FRAME_TOTAL_TIMEOUT_US = 3000;

// 如果你仍想用 readBytes，可启用该超时（当前实现已不用 readBytes）
static const uint16_t SERIAL2_READ_TIMEOUT_MS = 3;

// ================= Data Conversion =================
static const float ACC_SCALE = 0.0048828f;
static const float QUAT_SCALE = 0.0001f;

// ================= Output Control =================
// 高频打印会明显拖慢轮询频率：默认关闭，只保留 1Hz 频率报告
#ifndef ENABLE_CSV_OUTPUT
#define ENABLE_CSV_OUTPUT 0
#endif

// ================= Frequency Monitoring =================
static const uint32_t FREQ_REPORT_INTERVAL_MS = 1000;

struct ImuData {
  float acc[3];
  float quat[4];
  bool valid;
  uint32_t last_update_ms;
};

static ImuData imu_data[NUM_IMUS];
static bool imu_status[NUM_IMUS];

static uint8_t request_frames[NUM_IMUS][MODBUS_REQUEST_LEN];

static uint32_t cycle_count = 0;
static uint32_t freq_start_ms = 0;
static uint32_t last_freq_report_ms = 0;
static uint32_t success_count = 0;
static uint32_t fail_count = 0;
static uint32_t ok_per_imu[NUM_IMUS] = {0};
static uint32_t fail_per_imu[NUM_IMUS] = {0};
static uint32_t last_bus_activity_us = 0;

// ================= Utility =================
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
  while (Serial2.available() > 0) {
    Serial2.read();
  }
}

static void waitBusSilence() {
  if (last_bus_activity_us == 0) {
    return;
  }
  uint32_t now = micros();
  uint32_t elapsed = now - last_bus_activity_us;
  if (elapsed < BUS_SILENCE_US) {
    delayMicroseconds(BUS_SILENCE_US - elapsed);
  }
}

static size_t readFrameTwoStage(uint8_t *buf, size_t want_len) {
  uint32_t start = micros();
  size_t pos = 0;

  // 1) 等首字节（快速判断是否有响应）
  while ((micros() - start) < FIRST_BYTE_TIMEOUT_US) {
    if (Serial2.available() > 0) {
      int c = Serial2.read();
      if (c >= 0) {
        buf[pos++] = static_cast<uint8_t>(c);
        last_bus_activity_us = micros();
        break;
      }
    }
  }
  if (pos == 0) {
    return 0;
  }

  // 2) 继续读满整帧（总时长限制）
  while (pos < want_len && (micros() - start) < FRAME_TOTAL_TIMEOUT_US) {
    if (Serial2.available() > 0) {
      int c = Serial2.read();
      if (c >= 0) {
        buf[pos++] = static_cast<uint8_t>(c);
        last_bus_activity_us = micros();
      }
    }
  }
  return pos;
}

static void zeroImu(ImuData &data) {
  data.acc[0] = 0.0f;
  data.acc[1] = 0.0f;
  data.acc[2] = 0.0f;
  data.quat[0] = 0.0f;
  data.quat[1] = 0.0f;
  data.quat[2] = 0.0f;
  data.quat[3] = 0.0f;
  data.valid = false;
  data.last_update_ms = 0;
}

static void buildRequestFrame(uint8_t slave_id, uint8_t out[MODBUS_REQUEST_LEN]) {
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

static bool parseResponse(uint8_t slave_id, const uint8_t *buf, size_t len, ImuData &out) {
  if (len != RESPONSE_LEN) {
    return false;
  }
  if (buf[0] != slave_id || buf[1] != MODBUS_FUNC_READ_HREG) {
    return false;
  }
  if (buf[2] != RESPONSE_BYTE_COUNT) {
    return false;
  }

  uint16_t crc_calc = crc16_modbus(buf, len - 2);
  uint16_t crc_recv = static_cast<uint16_t>(buf[len - 2]) |
                      static_cast<uint16_t>(buf[len - 1] << 8);
  if (crc_calc != crc_recv) {
    return false;
  }

  const size_t data_start = 3;
  int16_t ax = static_cast<int16_t>((buf[data_start] << 8) | buf[data_start + 1]);
  int16_t ay = static_cast<int16_t>((buf[data_start + 2] << 8) | buf[data_start + 3]);
  int16_t az = static_cast<int16_t>((buf[data_start + 4] << 8) | buf[data_start + 5]);

  out.acc[0] = static_cast<float>(ax) * ACC_SCALE;
  out.acc[1] = static_cast<float>(ay) * ACC_SCALE;
  out.acc[2] = static_cast<float>(az) * ACC_SCALE;

  // 取最后 8 字节作为四元数（qw,qx,qy,qz）
  size_t quat_offset = data_start + (RESPONSE_BYTE_COUNT - 8);
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

  out.valid = true;
  out.last_update_ms = millis();
  return true;
}

static void outputCycleCsv() {
#if ENABLE_CSV_OUTPUT
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) {
      Serial.print(",");
    }
    Serial.print(IMU_IDS[i]);
    Serial.print(",");
    if (imu_data[i].valid) {
      Serial.print(imu_data[i].acc[0], 3);
      Serial.print(",");
      Serial.print(imu_data[i].acc[1], 3);
      Serial.print(",");
      Serial.print(imu_data[i].acc[2], 3);
      Serial.print(",");
      Serial.print(imu_data[i].quat[0], 4);
      Serial.print(",");
      Serial.print(imu_data[i].quat[1], 4);
      Serial.print(",");
      Serial.print(imu_data[i].quat[2], 4);
      Serial.print(",");
      Serial.print(imu_data[i].quat[3], 4);
    } else {
      Serial.print("0.000,0.000,0.000,0.0000,0.0000,0.0000,0.0000");
    }
  }
  Serial.println();
#endif
}

static void reportFrequency() {
  if (freq_start_ms == 0 || cycle_count == 0) {
    return;
  }

  uint32_t elapsed = millis() - freq_start_ms;
  if (elapsed == 0) {
    return;
  }

  float frequency = (static_cast<float>(cycle_count) * 1000.0f) / elapsed;
  Serial.print("# Update Frequency: ");
  Serial.print(frequency, 2);
  Serial.print(" Hz (");
  Serial.print(cycle_count);
  Serial.print(" cycles, ok=");
  Serial.print(success_count);
  Serial.print(", fail=");
  Serial.print(fail_count);
  Serial.print(") [per-imu ");
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) {
      Serial.print(" | ");
    }
    Serial.print("id=");
    Serial.print(IMU_IDS[i]);
    Serial.print(" ok=");
    Serial.print(ok_per_imu[i]);
    Serial.print(" fail=");
    Serial.print(fail_per_imu[i]);
  }
  Serial.print("]");
  Serial.println(")");

  cycle_count = 0;
  success_count = 0;
  fail_count = 0;
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    ok_per_imu[i] = 0;
    fail_per_imu[i] = 0;
  }
  freq_start_ms = millis();
}

// ================= Batch Processing (按用户示例逻辑) =================
static void doBatchProcessing() {
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    waitBusSilence();

    // 1) 发送请求：DE=1 -> write+flush -> DE=0（立刻切回接收）
    // 为防粘包/残留字节影响下一帧，发送前先清空 RX
    clearRxBuffer();

    digitalWrite(RS485_DE_RE_PIN, HIGH);
    if (TX_ENABLE_DELAY_US > 0) {
      delayMicroseconds(TX_ENABLE_DELAY_US);
    }
    Serial2.write(request_frames[i], MODBUS_REQUEST_LEN);
    Serial2.flush();
    if (TX_DISABLE_HOLD_US > 0) {
      delayMicroseconds(TX_DISABLE_HOLD_US);
    }
    digitalWrite(RS485_DE_RE_PIN, LOW);
    if (RX_SETTLE_US > 0) {
      delayMicroseconds(RX_SETTLE_US);
    }
    last_bus_activity_us = micros();

    // 2) 等待并读取响应：两段式超时（先等首字节，再读满整帧）
    uint8_t response[RESPONSE_LEN];
    size_t len = readFrameTwoStage(response, RESPONSE_LEN);

    if (len == RESPONSE_LEN && response[0] == IMU_IDS[i] && response[1] == MODBUS_FUNC_READ_HREG) {
      bool ok = parseResponse(IMU_IDS[i], response, len, imu_data[i]);
      imu_status[i] = ok;
      if (ok) {
        success_count++;
        ok_per_imu[i]++;
      } else {
        fail_count++;
        fail_per_imu[i]++;
        zeroImu(imu_data[i]);
      }
    } else {
      imu_status[i] = false;
      fail_count++;
      fail_per_imu[i]++;
      zeroImu(imu_data[i]);

      // 通信失败时清空缓冲区，防止粘包影响下一个 IMU
      clearRxBuffer();
    }
    last_bus_activity_us = micros();
  }

  cycle_count++;
  if (freq_start_ms == 0) {
    freq_start_ms = millis();
  }

  outputCycleCsv();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  Serial2.setTimeout(SERIAL2_READ_TIMEOUT_MS);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    zeroImu(imu_data[i]);
    imu_status[i] = false;
    buildRequestFrame(IMU_IDS[i], request_frames[i]);
  }

  Serial.println("# Modbus RTU batch read enabled");
  Serial.print("# IMU IDs: ");
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) {
      Serial.print(",");
    }
    Serial.print(IMU_IDS[i]);
  }
  Serial.println();
#if ENABLE_CSV_OUTPUT
  Serial.println("# CSV order: id,accx,accy,accz,qw,qx,qy,qz (repeat)");
#else
  Serial.println("# CSV output disabled (ENABLE_CSV_OUTPUT=0) to maximize polling rate");
#endif

  last_freq_report_ms = millis();
  last_bus_activity_us = 0;
}

void loop() {
  doBatchProcessing();

  if (millis() - last_freq_report_ms >= FREQ_REPORT_INTERVAL_MS) {
    last_freq_report_ms = millis();
    reportFrequency();
  }
}

