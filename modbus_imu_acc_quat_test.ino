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
static const uint8_t IMU_IDS[NUM_IMUS] = {1, 2};

// ================= Modbus RTU Configuration =================
static const uint8_t MODBUS_FUNC_READ_HREG = 0x03;

// 只读你需要的两段寄存器：
// ACC: 0x0034, len=3
// QUAT: 0x0046, len=4
static const uint16_t REG_ACC_START = 0x0034;
static const uint16_t REG_ACC_COUNT = 0x0003;
static const uint16_t REG_QUAT_START = 0x0046;
static const uint16_t REG_QUAT_COUNT = 0x0004;

static const uint16_t RESPONSE_TIMEOUT_MS = 30;
static const uint32_t TX_ENABLE_DELAY_US = 30;
static const uint32_t TX_DISABLE_DELAY_US = 60;

// ================= Data Conversion =================
static const float ACC_SCALE = 0.0048828f;
static const float QUAT_SCALE = 0.0001f;

// ================= Frequency Monitoring =================
static const uint32_t FREQ_REPORT_INTERVAL_MS = 1000;

struct ImuData {
  float acc[3];
  float quat[4];  // qw,qx,qy,qz
  bool valid;
  uint32_t last_update_ms;
};

static ImuData imu_data[NUM_IMUS];

static uint32_t cycle_count = 0;
static uint32_t freq_start_ms = 0;
static uint32_t last_freq_report_ms = 0;
static uint32_t success_count = 0;
static uint32_t fail_count = 0;

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
    (void)Serial2.read();
  }
}

static void zeroImu(ImuData &data) {
  data.acc[0] = data.acc[1] = data.acc[2] = 0.0f;
  data.quat[0] = data.quat[1] = data.quat[2] = data.quat[3] = 0.0f;
  data.valid = false;
  data.last_update_ms = 0;
}

static void buildRequest(uint8_t slave_id, uint16_t start_reg, uint16_t count, uint8_t out[8]) {
  out[0] = slave_id;
  out[1] = MODBUS_FUNC_READ_HREG;
  out[2] = (start_reg >> 8) & 0xFF;
  out[3] = start_reg & 0xFF;
  out[4] = (count >> 8) & 0xFF;
  out[5] = count & 0xFF;
  uint16_t crc = crc16_modbus(out, 6);
  out[6] = crc & 0xFF;        // CRC Lo
  out[7] = (crc >> 8) & 0xFF; // CRC Hi
}

static void rs485Send(const uint8_t *data, size_t len) {
  clearRxBuffer();
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(TX_ENABLE_DELAY_US);
  Serial2.write(data, len);
  Serial2.flush();
  delayMicroseconds(TX_DISABLE_DELAY_US);
  digitalWrite(RS485_DE_RE_PIN, LOW);
}

static bool rs485ReadExact(uint8_t *buf, size_t len, uint16_t timeout_ms) {
  const uint32_t start = millis();
  size_t pos = 0;
  while (pos < len && (millis() - start) <= timeout_ms) {
    int b = Serial2.read();
    if (b < 0) {
      // 让出一点 CPU，避免空转
      delayMicroseconds(20);
      continue;
    }
    buf[pos++] = static_cast<uint8_t>(b);
  }
  return pos == len;
}

// 读取 holding registers（功能码 0x03），返回 data 区（不含头/CRC）
// data_out 长度需为 (count*2)
static bool modbusReadHreg(uint8_t slave_id, uint16_t start_reg, uint16_t count, uint8_t *data_out) {
  uint8_t req[8];
  buildRequest(slave_id, start_reg, count, req);
  rs485Send(req, sizeof(req));

  const uint8_t byte_count = static_cast<uint8_t>(count * 2);
  const size_t resp_len = static_cast<size_t>(1 + 1 + 1 + byte_count + 2);
  uint8_t resp[1 + 1 + 1 + 2 * 8 + 2]; // 最多支持 count<=8 的小读；当前只用 3/4
  if (resp_len > sizeof(resp)) {
    return false;
  }

  if (!rs485ReadExact(resp, resp_len, RESPONSE_TIMEOUT_MS)) {
    clearRxBuffer();
    return false;
  }

  if (resp[0] != slave_id || resp[1] != MODBUS_FUNC_READ_HREG || resp[2] != byte_count) {
    return false;
  }

  const uint16_t crc_calc = crc16_modbus(resp, resp_len - 2);
  const uint16_t crc_recv =
      static_cast<uint16_t>(resp[resp_len - 2]) | static_cast<uint16_t>(resp[resp_len - 1] << 8);
  if (crc_calc != crc_recv) {
    return false;
  }

  memcpy(data_out, &resp[3], byte_count);
  return true;
}

static int16_t be16s(const uint8_t hi, const uint8_t lo) {
  return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | static_cast<uint16_t>(lo));
}

static bool readAccQuat(uint8_t slave_id, ImuData &out) {
  uint8_t acc_bytes[REG_ACC_COUNT * 2];
  uint8_t quat_bytes[REG_QUAT_COUNT * 2];

  if (!modbusReadHreg(slave_id, REG_ACC_START, REG_ACC_COUNT, acc_bytes)) {
    return false;
  }
  if (!modbusReadHreg(slave_id, REG_QUAT_START, REG_QUAT_COUNT, quat_bytes)) {
    return false;
  }

  const int16_t ax = be16s(acc_bytes[0], acc_bytes[1]);
  const int16_t ay = be16s(acc_bytes[2], acc_bytes[3]);
  const int16_t az = be16s(acc_bytes[4], acc_bytes[5]);

  out.acc[0] = static_cast<float>(ax) * ACC_SCALE;
  out.acc[1] = static_cast<float>(ay) * ACC_SCALE;
  out.acc[2] = static_cast<float>(az) * ACC_SCALE;

  const int16_t qw = be16s(quat_bytes[0], quat_bytes[1]);
  const int16_t qx = be16s(quat_bytes[2], quat_bytes[3]);
  const int16_t qy = be16s(quat_bytes[4], quat_bytes[5]);
  const int16_t qz = be16s(quat_bytes[6], quat_bytes[7]);

  out.quat[0] = static_cast<float>(qw) * QUAT_SCALE;
  out.quat[1] = static_cast<float>(qx) * QUAT_SCALE;
  out.quat[2] = static_cast<float>(qy) * QUAT_SCALE;
  out.quat[3] = static_cast<float>(qz) * QUAT_SCALE;

  out.valid = true;
  out.last_update_ms = millis();
  return true;
}

static void outputCycleCsv() {
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
}

static void reportFrequency() {
  if (freq_start_ms == 0 || cycle_count == 0) {
    return;
  }
  const uint32_t elapsed = millis() - freq_start_ms;
  if (elapsed == 0) {
    return;
  }

  const float frequency = (static_cast<float>(cycle_count) * 1000.0f) / elapsed;
  Serial.print("# Update Frequency: ");
  Serial.print(frequency, 2);
  Serial.print(" Hz (");
  Serial.print(cycle_count);
  Serial.print(" cycles, ok=");
  Serial.print(success_count);
  Serial.print(", fail=");
  Serial.print(fail_count);
  Serial.println(")");

  cycle_count = 0;
  success_count = 0;
  fail_count = 0;
  freq_start_ms = millis();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    zeroImu(imu_data[i]);
  }

  Serial.println("# Modbus RTU ACC+QUAT read enabled");
  Serial.print("# RS485 baud=");
  Serial.println(RS485_BAUD);
  Serial.print("# IMU IDs: ");
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) {
      Serial.print(",");
    }
    Serial.print(IMU_IDS[i]);
  }
  Serial.println();
  Serial.println("# ACC: start=0x0034, count=3 ; QUAT: start=0x0046, count=4");
  Serial.println("# CSV order: id,accx,accy,accz,qw,qx,qy,qz (repeat)");

  last_freq_report_ms = millis();
}

void loop() {
  // 一次 cycle：把所有 IMU 都更新一遍
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (readAccQuat(IMU_IDS[i], imu_data[i])) {
      success_count++;
    } else {
      fail_count++;
      zeroImu(imu_data[i]);
    }
  }

  cycle_count++;
  if (freq_start_ms == 0) {
    freq_start_ms = millis();
  }
  outputCycleCsv();

  if (millis() - last_freq_report_ms >= FREQ_REPORT_INTERVAL_MS) {
    last_freq_report_ms = millis();
    reportFrequency();
  }
}

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
static const uint16_t REG_ACC_START = 0x0034;
static const uint16_t REG_ACC_COUNT = 0x0003;   // ax, ay, az
static const uint16_t REG_QUAT_START = 0x0046;
static const uint16_t REG_QUAT_COUNT = 0x0004;  // qw, qx, qy, qz

static const uint16_t RESPONSE_TIMEOUT_MS = 30;
static const uint32_t TX_ENABLE_DELAY_US = 30;
static const uint32_t TX_DISABLE_DELAY_US = 60;
static const uint32_t BUS_SILENCE_US = 500;

// ================= Data Conversion =================
static const float ACC_SCALE = 0.0048828f;
static const float QUAT_SCALE = 0.0001f;

// ================= Frequency Monitoring =================
static const uint32_t FREQ_REPORT_INTERVAL_MS = 1000;

struct ImuData {
  float acc[3];
  float quat[4];
  bool valid;
  uint32_t last_update_ms;
};

static ImuData imu_data[NUM_IMUS];
static uint8_t current_imu_index = 0;
static uint32_t cycle_count = 0;
static uint32_t freq_start_ms = 0;
static uint32_t last_freq_report_ms = 0;
static uint32_t success_count = 0;
static uint32_t fail_count = 0;
static uint32_t last_bus_activity_us = 0;

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

static bool busIsIdle() {
  if (last_bus_activity_us == 0) return true;
  return (micros() - last_bus_activity_us) >= BUS_SILENCE_US;
}

static void zeroImu(ImuData &d) {
  d.acc[0] = d.acc[1] = d.acc[2] = 0.0f;
  d.quat[0] = d.quat[1] = d.quat[2] = d.quat[3] = 0.0f;
  d.valid = false;
  d.last_update_ms = 0;
}

static void buildReadRequest(uint8_t slave_id, uint16_t start_reg, uint16_t reg_count,
                             uint8_t out[8]) {
  out[0] = slave_id;
  out[1] = MODBUS_FUNC_READ_HREG;
  out[2] = (start_reg >> 8) & 0xFF;
  out[3] = start_reg & 0xFF;
  out[4] = (reg_count >> 8) & 0xFF;
  out[5] = reg_count & 0xFF;
  uint16_t crc = crc16_modbus(out, 6);
  out[6] = crc & 0xFF;         // CRC Lo
  out[7] = (crc >> 8) & 0xFF;  // CRC Hi
}

static void rs485Send(const uint8_t *data, size_t len) {
  clearRxBuffer();
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(TX_ENABLE_DELAY_US);
  Serial2.write(data, len);
  Serial2.flush();
  delayMicroseconds(TX_DISABLE_DELAY_US);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  last_bus_activity_us = micros();
}

static bool rs485ReadExact(uint8_t *buf, size_t len, uint32_t timeout_ms) {
  size_t pos = 0;
  uint32_t start = millis();
  while (pos < len && (millis() - start) <= timeout_ms) {
    while (Serial2.available() > 0 && pos < len) {
      int v = Serial2.read();
      if (v < 0) break;
      buf[pos++] = static_cast<uint8_t>(v);
      last_bus_activity_us = micros();
    }
    if (pos >= len) break;
    delayMicroseconds(50);
  }
  return pos == len;
}

// Read holding registers and return data bytes into data_out (length = reg_count*2)
static bool modbusReadHolding(uint8_t slave_id, uint16_t start_reg, uint16_t reg_count,
                              uint8_t *data_out) {
  const uint8_t req_len = 8;
  const uint8_t byte_count = static_cast<uint8_t>(reg_count * 2);
  const size_t resp_len = static_cast<size_t>(1 + 1 + 1 + byte_count + 2);

  uint8_t req[req_len];
  buildReadRequest(slave_id, start_reg, reg_count, req);
  rs485Send(req, req_len);

  uint8_t resp[64];
  if (resp_len > sizeof(resp)) return false;
  if (!rs485ReadExact(resp, resp_len, RESPONSE_TIMEOUT_MS)) return false;

  if (resp[0] != slave_id) return false;
  if (resp[1] != MODBUS_FUNC_READ_HREG) return false;
  if (resp[2] != byte_count) return false;

  uint16_t crc_calc = crc16_modbus(resp, resp_len - 2);
  uint16_t crc_recv = static_cast<uint16_t>(resp[resp_len - 2]) |
                      static_cast<uint16_t>(resp[resp_len - 1] << 8);
  if (crc_calc != crc_recv) return false;

  memcpy(data_out, &resp[3], byte_count);
  return true;
}

static int16_t be16s(const uint8_t *p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | static_cast<uint16_t>(p[1]));
}

static bool readImuAccQuat(uint8_t slave_id, ImuData &out) {
  uint8_t acc_bytes[REG_ACC_COUNT * 2];
  uint8_t quat_bytes[REG_QUAT_COUNT * 2];

  if (!modbusReadHolding(slave_id, REG_ACC_START, REG_ACC_COUNT, acc_bytes)) return false;
  if (!modbusReadHolding(slave_id, REG_QUAT_START, REG_QUAT_COUNT, quat_bytes)) return false;

  int16_t ax = be16s(&acc_bytes[0]);
  int16_t ay = be16s(&acc_bytes[2]);
  int16_t az = be16s(&acc_bytes[4]);

  out.acc[0] = static_cast<float>(ax) * ACC_SCALE;
  out.acc[1] = static_cast<float>(ay) * ACC_SCALE;
  out.acc[2] = static_cast<float>(az) * ACC_SCALE;

  int16_t qw = be16s(&quat_bytes[0]);
  int16_t qx = be16s(&quat_bytes[2]);
  int16_t qy = be16s(&quat_bytes[4]);
  int16_t qz = be16s(&quat_bytes[6]);

  out.quat[0] = static_cast<float>(qw) * QUAT_SCALE;
  out.quat[1] = static_cast<float>(qx) * QUAT_SCALE;
  out.quat[2] = static_cast<float>(qy) * QUAT_SCALE;
  out.quat[3] = static_cast<float>(qz) * QUAT_SCALE;

  out.valid = true;
  out.last_update_ms = millis();
  return true;
}

static void outputCycleCsv() {
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) Serial.print(",");
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
}

static void reportFrequency() {
  if (freq_start_ms == 0 || cycle_count == 0) return;
  uint32_t elapsed = millis() - freq_start_ms;
  if (elapsed == 0) return;

  float hz = (static_cast<float>(cycle_count) * 1000.0f) / elapsed;
  Serial.print("# Update Frequency: ");
  Serial.print(hz, 2);
  Serial.print(" Hz (");
  Serial.print(cycle_count);
  Serial.print(" cycles, ok=");
  Serial.print(success_count);
  Serial.print(", fail=");
  Serial.print(fail_count);
  Serial.println(")");

  cycle_count = 0;
  success_count = 0;
  fail_count = 0;
  freq_start_ms = millis();
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  for (uint8_t i = 0; i < NUM_IMUS; ++i) zeroImu(imu_data[i]);

  Serial.println("# Modbus RTU ACC+QUAT read enabled");
  Serial.print("# IMU IDs: ");
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i) Serial.print(",");
    Serial.print(IMU_IDS[i]);
  }
  Serial.println();
  Serial.println("# CSV order: id,accx,accy,accz,qw,qx,qy,qz (repeat)");
  Serial.print("# ACC regs: 0x");
  Serial.print(REG_ACC_START, HEX);
  Serial.print(" len=");
  Serial.println(REG_ACC_COUNT);
  Serial.print("# QUAT regs: 0x");
  Serial.print(REG_QUAT_START, HEX);
  Serial.print(" len=");
  Serial.println(REG_QUAT_COUNT);

  last_freq_report_ms = millis();
}

void loop() {
  if (!busIsIdle()) return;

  uint8_t id = IMU_IDS[current_imu_index];
  bool ok = readImuAccQuat(id, imu_data[current_imu_index]);
  if (ok) {
    success_count++;
  } else {
    fail_count++;
    zeroImu(imu_data[current_imu_index]);
    clearRxBuffer();
  }

  current_imu_index++;
  if (current_imu_index >= NUM_IMUS) {
    current_imu_index = 0;
    cycle_count++;
    if (freq_start_ms == 0) freq_start_ms = millis();
    outputCycleCsv();
  }

  if (millis() - last_freq_report_ms >= FREQ_REPORT_INTERVAL_MS) {
    last_freq_report_ms = millis();
    reportFrequency();
  }
}

