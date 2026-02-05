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
static const uint16_t RESPONSE_BYTE_COUNT = MODBUS_REG_COUNT * 2;
static const uint16_t RESPONSE_LEN = 1 + 1 + 1 + RESPONSE_BYTE_COUNT + 2;  // 47 bytes
static const uint16_t RESPONSE_TIMEOUT_MS = 10;  // 缩短超时时间，提高轮询频率
static const uint32_t TX_DISABLE_DELAY_US = 50;  // 发送后切换延迟

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

ImuData imu_data[NUM_IMUS];

static uint32_t cycle_count = 0;
static uint32_t freq_start_ms = 0;
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

static bool parseResponse(uint8_t slave_id, const uint8_t *buf, size_t len, ImuData &out) {
  // 检查基本长度
  if (len != RESPONSE_LEN) {
    return false;
  }
  
  // 检查从机ID和功能码
  if (buf[0] != slave_id || buf[1] != MODBUS_FUNC_READ_HREG) {
    return false;
  }
  
  // 检查字节计数
  if (buf[2] != RESPONSE_BYTE_COUNT) {
    return false;
  }

  // 验证CRC
  uint16_t crc_calc = crc16_modbus(buf, len - 2);
  uint16_t crc_recv = static_cast<uint16_t>(buf[len - 2]) |
                      static_cast<uint16_t>(buf[len - 1] << 8);
  if (crc_calc != crc_recv) {
    return false;
  }

  // 解析加速度数据 (寄存器0x0034-0x0036)
  const size_t data_start = 3;
  int16_t ax = static_cast<int16_t>((buf[data_start] << 8) | buf[data_start + 1]);
  int16_t ay = static_cast<int16_t>((buf[data_start + 2] << 8) | buf[data_start + 3]);
  int16_t az = static_cast<int16_t>((buf[data_start + 4] << 8) | buf[data_start + 5]);

  out.acc[0] = static_cast<float>(ax) * ACC_SCALE;
  out.acc[1] = static_cast<float>(ay) * ACC_SCALE;
  out.acc[2] = static_cast<float>(az) * ACC_SCALE;

  // 解析四元数数据 (最后4个寄存器: 0x0046-0x0049)
  size_t quat_offset = data_start + RESPONSE_BYTE_COUNT - 8;
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

  uint32_t elapsed = millis() - freq_start_ms;
  if (elapsed == 0) {
    return;
  }

  float frequency = (static_cast<float>(cycle_count) * 1000.0f) / elapsed;
  Serial.print("# 更新频率: ");
  Serial.print(frequency, 2);
  Serial.print(" Hz (");
  Serial.print(cycle_count);
  Serial.print(" 周期, 成功=");
  Serial.print(success_count);
  Serial.print(", 失败=");
  Serial.print(fail_count);
  Serial.println(")");

  cycle_count = 0;
  success_count = 0;
  fail_count = 0;
  freq_start_ms = millis();
}

// ================= 批处理核心函数 (参考编码器逻辑) =================
void doBatchProcessing() {
  uint8_t request[MODBUS_REQUEST_LEN];
  uint8_t response[RESPONSE_LEN];
  
  for (int i = 0; i < NUM_IMUS; i++) {
    // 1. 构建并发送请求
    buildRequest(IMU_IDS[i], request);
    
    // 切换到发送模式
    digitalWrite(RS485_DE_RE_PIN, HIGH);
    Serial2.write(request, MODBUS_REQUEST_LEN);
    Serial2.flush();  // 确保数据完全发出
    delayMicroseconds(TX_DISABLE_DELAY_US);
    
    // 立即切换到接收模式
    digitalWrite(RS485_DE_RE_PIN, LOW);

    // 2. 等待并读取响应 (阻塞式读取)
    // Serial2.readBytes 会受 Serial2.setTimeout() 影响
    size_t len = Serial2.readBytes(response, RESPONSE_LEN);

    // 3. 解析响应
    if (len == RESPONSE_LEN && 
        response[0] == IMU_IDS[i] && 
        response[1] == MODBUS_FUNC_READ_HREG &&
        response[2] == RESPONSE_BYTE_COUNT) {
      
      // 尝试解析数据
      if (parseResponse(IMU_IDS[i], response, len, imu_data[i])) {
        success_count++;
      } else {
        // CRC校验失败
        imu_data[i].valid = false;
        fail_count++;
      }
    } else {
      // 通信失败：超时、长度不对或帧头错误
      imu_data[i].valid = false;
      fail_count++;
      
      // 清空缓冲区，防止粘包影响下一个传感器
      while (Serial2.available()) {
        Serial2.read();
      }
    }
  }

  // 4. 更新周期计数
  cycle_count++;
  if (freq_start_ms == 0) {
    freq_start_ms = millis();
  }

  // 5. 输出数据
  outputCycleCsv();
}

// ================= Setup & Loop =================
void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  
  // 设置串口超时（用于 readBytes）
  Serial2.setTimeout(RESPONSE_TIMEOUT_MS);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);  // 默认接收模式

  // 初始化IMU数据
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    imu_data[i].acc[0] = 0.0f;
    imu_data[i].acc[1] = 0.0f;
    imu_data[i].acc[2] = 0.0f;
    imu_data[i].quat[0] = 0.0f;
    imu_data[i].quat[1] = 0.0f;
    imu_data[i].quat[2] = 0.0f;
    imu_data[i].quat[3] = 0.0f;
    imu_data[i].valid = false;
    imu_data[i].last_update_ms = 0;
  }

  Serial.println("# IMU Modbus批处理模式已启用");
  Serial.print("# IMU IDs: ");
  for (uint8_t i = 0; i < NUM_IMUS; ++i) {
    if (i > 0) {
      Serial.print(",");
    }
    Serial.print(IMU_IDS[i]);
  }
  Serial.println();
  Serial.println("# CSV格式: id,accx,accy,accz,qw,qx,qy,qz (重复)");

  last_freq_report_ms = millis();
}

void loop() {
  // 执行批处理：依次轮询所有IMU
  doBatchProcessing();

  // 定期报告频率
  if (millis() - last_freq_report_ms >= FREQ_REPORT_INTERVAL_MS) {
    last_freq_report_ms = millis();
    reportFrequency();
  }
}
