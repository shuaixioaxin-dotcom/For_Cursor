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
static const uint16_t RESPONSE_BYTE_COUNT = MODBUS_REG_COUNT * 2;  // 44 bytes
static const uint16_t RESPONSE_LEN = 1 + 1 + 1 + RESPONSE_BYTE_COUNT + 2;  // 49 bytes

// ================= Timing Configuration (Optimized) =================
// 超时设置：根据波特率计算
// 921600 baud, 10 bits/byte => ~11us/byte
// 49 bytes response => ~539us minimum
// 设置 15ms 超时，留足够余量给IMU处理时间
static const uint16_t RESPONSE_TIMEOUT_MS = 15;

// RS485 收发器切换延迟 (微秒)
static const uint32_t TX_SWITCH_DELAY_US = 50;

// 请求之间的总线静默时间 (微秒)
// 给总线和设备一些恢复时间
static const uint32_t BUS_SILENCE_US = 200;

// ================= Data Conversion =================
static const float ACC_SCALE = 0.0048828f;
static const float QUAT_SCALE = 0.0001f;

// ================= Frequency Monitoring =================
static const uint32_t FREQ_REPORT_INTERVAL_MS = 1000;

// ================= Data Structures =================
struct ImuData {
  float acc[3];
  float quat[4];
  bool valid;
  uint32_t last_update_ms;
};

ImuData imu_data[NUM_IMUS];
bool imu_status[NUM_IMUS];

// ================= Pre-built Request Frames =================
// 预构建请求帧，避免运行时计算
uint8_t request_frames[NUM_IMUS][MODBUS_REQUEST_LEN];

// ================= Statistics =================
static uint32_t cycle_count = 0;
static uint32_t freq_calc_start = 0;
static uint32_t last_freq_report_ms = 0;
static uint32_t success_count = 0;
static uint32_t fail_count = 0;

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

static void buildRequestFrame(uint8_t slave_id, uint8_t *out) {
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

static bool parseResponse(uint8_t slave_id, const uint8_t *buf, size_t len, ImuData &out) {
  // 长度校验
  if (len != RESPONSE_LEN) {
    return false;
  }
  
  // 地址和功能码校验
  if (buf[0] != slave_id || buf[1] != MODBUS_FUNC_READ_HREG) {
    return false;
  }
  
  // 字节数校验
  if (buf[2] != RESPONSE_BYTE_COUNT) {
    return false;
  }

  // CRC校验
  uint16_t crc_calc = crc16_modbus(buf, len - 2);
  uint16_t crc_recv = static_cast<uint16_t>(buf[len - 2]) |
                      static_cast<uint16_t>(buf[len - 1] << 8);
  if (crc_calc != crc_recv) {
    return false;
  }

  // 解析加速度数据 (寄存器偏移 0-2)
  const size_t data_start = 3;
  int16_t ax = static_cast<int16_t>((buf[data_start] << 8) | buf[data_start + 1]);
  int16_t ay = static_cast<int16_t>((buf[data_start + 2] << 8) | buf[data_start + 3]);
  int16_t az = static_cast<int16_t>((buf[data_start + 4] << 8) | buf[data_start + 5]);

  out.acc[0] = static_cast<float>(ax) * ACC_SCALE;
  out.acc[1] = static_cast<float>(ay) * ACC_SCALE;
  out.acc[2] = static_cast<float>(az) * ACC_SCALE;

  // 解析四元数数据 (最后4个寄存器)
  size_t quat_offset = data_start + (RESPONSE_BYTE_COUNT - 8);  // 最后8字节
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

// ================= Output Functions =================
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
  if (freq_calc_start == 0 || cycle_count == 0) {
    return;
  }

  uint32_t elapsed = millis() - freq_calc_start;
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
  Serial.println(")");

  // 重置统计
  cycle_count = 0;
  success_count = 0;
  fail_count = 0;
  freq_calc_start = millis();
}

// ================= Core Batch Processing Function =================
/**
 * 批处理函数 - 按照编码器代码逻辑优化
 * 
 * 关键优化点：
 * 1. 同步阻塞模式，减少状态机开销
 * 2. 使用 Serial2.readBytes() 配合短超时
 * 3. 预构建请求帧，减少运行时计算
 * 4. 通信失败时清空缓冲区，防止粘包
 */
void doBatchProcessing() {
  for (int i = 0; i < NUM_IMUS; i++) {
    // 0. 清空接收缓冲区，确保干净的起始状态
    while (Serial2.available()) {
      Serial2.read();
    }

    // 1. 发送请求
    digitalWrite(RS485_DE_RE_PIN, HIGH);
    delayMicroseconds(TX_SWITCH_DELAY_US);  // 等待收发器切换到发送模式
    Serial2.write(request_frames[i], MODBUS_REQUEST_LEN);
    Serial2.flush();  // 确保数据完全发出
    delayMicroseconds(TX_SWITCH_DELAY_US);  // 等待最后一个字节发送完成
    digitalWrite(RS485_DE_RE_PIN, LOW);  // 切换到接收模式

    // 2. 等待并读取响应
    uint8_t response[RESPONSE_LEN];
    size_t len = Serial2.readBytes(response, RESPONSE_LEN);

    // 3. 解析响应
    if (len == RESPONSE_LEN && 
        response[0] == IMU_IDS[i] && 
        response[1] == MODBUS_FUNC_READ_HREG) {
      
      if (parseResponse(IMU_IDS[i], response, len, imu_data[i])) {
        imu_status[i] = true;
        success_count++;
      } else {
        imu_status[i] = false;
        zeroImu(imu_data[i]);
        fail_count++;
      }
    } else {
      imu_status[i] = false;
      zeroImu(imu_data[i]);
      fail_count++;
      // 通信失败时清空缓冲区，防止粘包影响下一个传感器
      while (Serial2.available()) {
        Serial2.read();
      }
    }

    // 4. 请求之间的总线静默时间
    delayMicroseconds(BUS_SILENCE_US);
  }

  // 4. 更新统计
  cycle_count++;
  if (freq_calc_start == 0) {
    freq_calc_start = millis();
  }

  // 5. 输出数据
  outputCycleCsv();
}

// ================= Setup =================
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  
  // 设置短超时，这是同步批处理的关键
  Serial2.setTimeout(RESPONSE_TIMEOUT_MS);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  // 初始化数据结构
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    zeroImu(imu_data[i]);
    imu_status[i] = false;
    // 预构建请求帧
    buildRequestFrame(IMU_IDS[i], request_frames[i]);
  }

  Serial.println("# Modbus RTU Batch Processing Mode (Optimized)");
  Serial.print("# IMU IDs: ");
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) {
      Serial.print(",");
    }
    Serial.print(IMU_IDS[i]);
  }
  Serial.println();
  Serial.print("# Response timeout: ");
  Serial.print(RESPONSE_TIMEOUT_MS);
  Serial.print(" ms, TX delay: ");
  Serial.print(TX_SWITCH_DELAY_US);
  Serial.print(" us, Bus silence: ");
  Serial.print(BUS_SILENCE_US);
  Serial.println(" us");
  Serial.println("# CSV order: id,accx,accy,accz,qw,qx,qy,qz (repeat)");

  last_freq_report_ms = millis();
}

// ================= Main Loop =================
void loop() {
  // 执行批处理
  doBatchProcessing();

  // 定期报告频率
  if (millis() - last_freq_report_ms >= FREQ_REPORT_INTERVAL_MS) {
    last_freq_report_ms = millis();
    reportFrequency();
  }
}
