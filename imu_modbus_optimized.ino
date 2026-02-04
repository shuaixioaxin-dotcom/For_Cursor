#include <Arduino.h>

// ================= Pin Configuration =================
constexpr uint8_t RS485_RX_PIN = 32;     // Module RO
constexpr uint8_t RS485_TX_PIN = 33;     // Module DI
constexpr uint8_t RS485_DE_RE_PIN = 25;  // Module DE/RE Control

// ================= Serial Configuration =================
constexpr uint32_t SERIAL_BAUD = 921600;
constexpr uint32_t RS485_BAUD  = 921600;

// ================= IMU Configuration =================
constexpr uint8_t NUM_IMUS = 2;
constexpr uint8_t IMU_IDS[NUM_IMUS] = {1, 2};

// ================= Modbus RTU Configuration =================
constexpr uint8_t  MODBUS_FUNC_READ_HREG = 0x03;
constexpr uint16_t MODBUS_REG_START      = 0x0034;
constexpr uint16_t MODBUS_REG_COUNT      = 0x0016;  // 22 registers
constexpr uint8_t  MODBUS_REQUEST_LEN    = 8;

constexpr uint16_t RESPONSE_BYTE_COUNT   = MODBUS_REG_COUNT * 2;                 // 44
constexpr uint16_t RESPONSE_LEN          = 1 + 1 + 1 + RESPONSE_BYTE_COUNT + 2;  // id+func+bc+data+crc = 49

// ================= Performance Targets =================
constexpr uint32_t OUTPUT_HZ             = 200;
constexpr uint32_t OUTPUT_PERIOD_US      = 1000000UL / OUTPUT_HZ;
constexpr uint32_t IMU_POLL_HZ           = 200;
constexpr uint32_t IMU_POLL_PERIOD_US    = 1000000UL / IMU_POLL_HZ;

// ================= Timing =================
constexpr uint16_t RESPONSE_TIMEOUT_MS   = 4;
constexpr uint32_t BUS_SILENCE_US        = 100;
constexpr uint32_t TX_ENABLE_DELAY_US    = 10;
constexpr uint32_t TX_DISABLE_DELAY_US   = 10;
constexpr uint16_t FAIL_COOLDOWN_MS      = 10;
constexpr uint8_t  MAX_BACKOFF_SHIFT     = 4;

// ================= Data Conversion =================
constexpr float ACC_SCALE  = 0.0048828f;
constexpr float QUAT_SCALE = 0.0001f;

// ================= State =================
struct ImuData {
  float acc[3];
  float quat[4];
  bool valid;
  uint32_t last_update_us;
};

static ImuData imu_data[NUM_IMUS];
static uint32_t imu_next_poll_us[NUM_IMUS];
static uint8_t imu_fail_streak[NUM_IMUS];

static uint8_t current_imu_index = 0;
static bool waiting_response = false;
static uint32_t request_deadline_ms = 0;
static uint32_t last_bus_activity_us = 0;
static uint32_t next_output_us = 0;

// ================= RX Ring Buffer =================
constexpr size_t RX_RING_SZ = 128;
static uint8_t rx_ring[RX_RING_SZ];
static size_t rx_head = 0;
static size_t rx_tail = 0;

static inline size_t rxCount() {
  return (rx_head + RX_RING_SZ - rx_tail) % RX_RING_SZ;
}

static inline void rxDropOne() {
  rx_tail = (rx_tail + 1) % RX_RING_SZ;
}

static inline void rxClear() {
  rx_head = 0;
  rx_tail = 0;
}

static void rxPush(uint8_t b) {
  size_t next = (rx_head + 1) % RX_RING_SZ;
  if (next == rx_tail) {
    rxDropOne();
  }
  rx_ring[rx_head] = b;
  rx_head = next;
}

// ================= Utility =================
static inline bool timeReached(uint32_t now, uint32_t target) {
  return (int32_t)(now - target) >= 0;
}

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

static void clearUartRxBuffer() {
  while (Serial2.available() > 0) {
    (void)Serial2.read();
  }
}

static void clearAllRx() {
  clearUartRxBuffer();
  rxClear();
}

static void drainSerial2() {
  while (Serial2.available() > 0) {
    int v = Serial2.read();
    if (v < 0) break;
    last_bus_activity_us = micros();
    rxPush((uint8_t)v);
  }
}

static bool busIsIdle(uint32_t now_us) {
  if (last_bus_activity_us == 0) return true;
  return timeReached(now_us, last_bus_activity_us + BUS_SILENCE_US);
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
  data.last_update_us = 0;
}

static void scheduleNextPoll(uint8_t idx, bool success) {
  uint32_t now_us = micros();
  if (success) {
    imu_fail_streak[idx] = 0;
    imu_next_poll_us[idx] = now_us + IMU_POLL_PERIOD_US;
  } else {
    if (imu_fail_streak[idx] < MAX_BACKOFF_SHIFT) {
      imu_fail_streak[idx]++;
    }
    uint32_t backoff_ms = (uint32_t)FAIL_COOLDOWN_MS << imu_fail_streak[idx];
    imu_next_poll_us[idx] = now_us + (backoff_ms * 1000UL);
  }
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
}

// ================= Frame extraction =================
static bool tryPopFrame(uint8_t slave_id, uint8_t *frame_out) {
  while (rxCount() >= RESPONSE_LEN) {
    size_t idx0 = rx_tail;
    size_t idx1 = (idx0 + 1) % RX_RING_SZ;
    size_t idx2 = (idx0 + 2) % RX_RING_SZ;

    if (rx_ring[idx0] != slave_id ||
        rx_ring[idx1] != MODBUS_FUNC_READ_HREG ||
        rx_ring[idx2] != RESPONSE_BYTE_COUNT) {
      rxDropOne();
      continue;
    }

    for (size_t i = 0; i < RESPONSE_LEN; ++i) {
      size_t idx = (rx_tail + i) % RX_RING_SZ;
      frame_out[i] = rx_ring[idx];
    }

    uint16_t crc_calc = crc16_modbus(frame_out, RESPONSE_LEN - 2);
    uint16_t crc_recv = (uint16_t)frame_out[RESPONSE_LEN - 2] |
                        ((uint16_t)frame_out[RESPONSE_LEN - 1] << 8);
    if (crc_calc != crc_recv) {
      rxDropOne();
      continue;
    }

    for (size_t i = 0; i < RESPONSE_LEN; ++i) {
      rxDropOne();
    }
    return true;
  }
  return false;
}

static void parseFrame(const uint8_t *buf, ImuData &out) {
  constexpr size_t DATA_START = 3;
  constexpr size_t QUAT_OFFSET = DATA_START + RESPONSE_BYTE_COUNT - 8;

  int16_t ax = (int16_t)((buf[DATA_START] << 8) | buf[DATA_START + 1]);
  int16_t ay = (int16_t)((buf[DATA_START + 2] << 8) | buf[DATA_START + 3]);
  int16_t az = (int16_t)((buf[DATA_START + 4] << 8) | buf[DATA_START + 5]);

  out.acc[0] = (float)ax * ACC_SCALE;
  out.acc[1] = (float)ay * ACC_SCALE;
  out.acc[2] = (float)az * ACC_SCALE;

  int16_t qw = (int16_t)((buf[QUAT_OFFSET] << 8) | buf[QUAT_OFFSET + 1]);
  int16_t qx = (int16_t)((buf[QUAT_OFFSET + 2] << 8) | buf[QUAT_OFFSET + 3]);
  int16_t qy = (int16_t)((buf[QUAT_OFFSET + 4] << 8) | buf[QUAT_OFFSET + 5]);
  int16_t qz = (int16_t)((buf[QUAT_OFFSET + 6] << 8) | buf[QUAT_OFFSET + 7]);

  out.quat[0] = (float)qw * QUAT_SCALE;
  out.quat[1] = (float)qx * QUAT_SCALE;
  out.quat[2] = (float)qy * QUAT_SCALE;
  out.quat[3] = (float)qz * QUAT_SCALE;

  out.valid = true;
  out.last_update_us = micros();
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

// ================= Scheduling =================
static bool pickNextImu(uint32_t now_us) {
  for (uint8_t offset = 0; offset < NUM_IMUS; ++offset) {
    uint8_t idx = (current_imu_index + offset) % NUM_IMUS;
    if (timeReached(now_us, imu_next_poll_us[idx])) {
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
    imu_next_poll_us[i] = micros();
    imu_fail_streak[i] = 0;
  }

  buildAllRequests();
  next_output_us = micros() + OUTPUT_PERIOD_US;
}

void loop() {
  static uint8_t response_buf[RESPONSE_LEN];

  drainSerial2();

  if (waiting_response) {
    uint8_t slave_id = IMU_IDS[current_imu_index];
    if (tryPopFrame(slave_id, response_buf)) {
      parseFrame(response_buf, imu_data[current_imu_index]);
      scheduleNextPoll(current_imu_index, true);
      waiting_response = false;
      current_imu_index = (current_imu_index + 1) % NUM_IMUS;
    } else if (timeReached(millis(), request_deadline_ms)) {
      zeroImu(imu_data[current_imu_index]);
      scheduleNextPoll(current_imu_index, false);
      clearAllRx();
      waiting_response = false;
      current_imu_index = (current_imu_index + 1) % NUM_IMUS;
    }
  }

  if (!waiting_response) {
    uint32_t now_us = micros();
    if (busIsIdle(now_us) && pickNextImu(now_us)) {
      clearAllRx();
      sendRequestCached(current_imu_index);
      waiting_response = true;
      request_deadline_ms = millis() + RESPONSE_TIMEOUT_MS;
    }
  }

  uint32_t now_us = micros();
  if (timeReached(now_us, next_output_us)) {
    outputImuData();
    next_output_us = now_us + OUTPUT_PERIOD_US;
  }
}
