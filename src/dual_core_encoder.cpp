/**
 * @file dual_core_encoder.cpp
 * @brief Multi-encoder communication - Ultra-Fast Dual-Core Batch Processing Mode
 * @note Optimized for maximum frequency using FreeRTOS dual-core processing,
 *       minimized overhead, and aggressive timing optimization.
 */
#include "dual_core_encoder.h"

#include <FastLED.h>
#include <esp_task_wdt.h>
#include <soc/gpio_struct.h>

namespace DualCoreEncoder {
namespace {
constexpr uint8_t kRs485RxPin = 32;
constexpr uint8_t kRs485TxPin = 33;
constexpr uint8_t kRs485DeRePin = 25;

constexpr uint8_t kWs2812Pin = 26;
constexpr uint8_t kNumLeds = 1;
constexpr uint8_t kBuzzerPin = 2;

constexpr size_t kNumEncoders = 6;
const uint8_t kEncoderIds[kNumEncoders] = {1, 2, 3, 4, 5, 6};

uint8_t request_frames[kNumEncoders][8];

struct EncoderData {
  uint16_t values[kNumEncoders];
  bool status[kNumEncoders];
  bool all_ok;
  volatile uint32_t seq_number;
};

EncoderData encoder_data_buffer[2];
volatile uint8_t write_buffer_idx = 0;
volatile uint8_t read_buffer_idx = 1;

volatile unsigned long cycle_count = 0;
volatile unsigned long freq_calc_start = 0;
constexpr uint32_t kFreqReportIntervalMs = 1000;
constexpr uint32_t kWdtYieldEveryCycles = DUAL_CORE_ENCODER_WDT_YIELD_CYCLES;

CRGB leds[kNumLeds];

inline uint16_t calculateCRC(uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= static_cast<uint16_t>(buf[pos]);
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void playStartupSound() {
  int melody[] = {1000, 1500, 2000};
  for (int i = 0; i < 3; i++) {
    tone(kBuzzerPin, melody[i]);
    delay(100);
    noTone(kBuzzerPin);
    delay(50);
  }
}

inline __attribute__((always_inline)) int readBytesFast(
    uint8_t *buffer,
    int length,
    uint32_t response_timeout_us,
    uint32_t inter_byte_timeout_us) {
  int count = 0;
  unsigned long start = micros();

  while (!Serial2.available()) {
    if (micros() - start > response_timeout_us) {
      return 0;
    }
  }

  while (count < length) {
    int available = Serial2.available();
    if (available > 0) {
      int to_read = min(available, length - count);
      for (int i = 0; i < to_read; i++) {
        buffer[count++] = Serial2.read();
      }
      start = micros();
    } else {
      if (micros() - start > inter_byte_timeout_us) {
        break;
      }
    }
  }
  return count;
}

void doBatchProcessing() {
  static uint32_t seq = 0;
  bool current_batch_ok = true;

  EncoderData *data = &encoder_data_buffer[write_buffer_idx];

  for (size_t i = 0; i < kNumEncoders; i++) {
    GPIO.out_w1ts = (1UL << kRs485DeRePin);
    Serial2.write(request_frames[i], 8);
    delayMicroseconds(32);
    GPIO.out_w1tc = (1UL << kRs485DeRePin);

    uint8_t response[7];
    int len = readBytesFast(response, 7, 300, 80);

    if (len == 7 && response[0] == kEncoderIds[i] && response[1] == 0x03) {
      data->values[i] = (response[3] << 8) | response[4];
      data->status[i] = true;
    } else {
      data->status[i] = false;
      current_batch_ok = false;
      for (uint8_t clear_count = 0;
           Serial2.available() && clear_count < 16;
           clear_count++) {
        Serial2.read();
      }
    }
  }

  data->all_ok = current_batch_ok;
  data->seq_number = ++seq;

  uint8_t old_write = write_buffer_idx;
  write_buffer_idx = read_buffer_idx;
  read_buffer_idx = old_write;

  cycle_count++;
}

void taskDataAcquisition(void *parameter) {
  (void)parameter;
  freq_calc_start = millis();

  vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
#if DUAL_CORE_ENCODER_WDT_DISABLE
  esp_task_wdt_delete(NULL);
#endif
  uint32_t yield_counter = 0;

  while (true) {
    doBatchProcessing();
    if (++yield_counter >= kWdtYieldEveryCycles) {
      yield_counter = 0;
      vTaskDelay(1);
    }
  }
}

void taskOutputDisplay(void *parameter) {
  (void)parameter;
  unsigned long last_output_time = 0;
  unsigned long last_freq_report_time = 0;
  unsigned long last_led_update_time = 0;
  uint32_t last_seq = 0;

  constexpr uint32_t kOutputIntervalMs = 10;
  constexpr uint32_t kLedUpdateIntervalMs = 100;

  char output_buffer[192];

  while (true) {
    unsigned long current_time = millis();

    EncoderData *data = &encoder_data_buffer[read_buffer_idx];
    uint32_t current_seq = data->seq_number;

    if (current_seq != last_seq) {
      last_seq = current_seq;

      if (current_time - last_output_time >= kOutputIntervalMs) {
        last_output_time = current_time;

        char *ptr = output_buffer;
        for (size_t i = 0; i < kNumEncoders; i++) {
          uint16_t raw = data->values[i];
          uint32_t angle_x100 = (static_cast<uint32_t>(raw) * 1125) >> 11;

          uint32_t int_part = angle_x100 / 100;
          uint32_t dec_part = angle_x100 % 100;

          if (int_part == 0) {
            *ptr++ = '0';
          } else {
            char tmp[10];
            int j = 0;
            uint32_t n = int_part;
            while (n > 0) {
              tmp[j++] = (n % 10) + '0';
              n /= 10;
            }
            while (j > 0) {
              *ptr++ = tmp[--j];
            }
          }

          *ptr++ = '.';
          *ptr++ = (dec_part / 10) + '0';
          *ptr++ = (dec_part % 10) + '0';

          if (i < kNumEncoders - 1) {
            *ptr++ = ',';
          }
        }
        *ptr++ = '\r';
        *ptr++ = '\n';

        Serial.write(reinterpret_cast<const uint8_t *>(output_buffer),
                     ptr - output_buffer);
      }

      if (current_time - last_led_update_time >= kLedUpdateIntervalMs) {
        last_led_update_time = current_time;
        leds[0] = data->all_ok ? CRGB::Green : CRGB::Red;
        FastLED.show();
      }
    }

    if (current_time - last_freq_report_time >= kFreqReportIntervalMs) {
      last_freq_report_time = current_time;
      unsigned long elapsed_time = current_time - freq_calc_start;
      if (elapsed_time > 0 && cycle_count > 0) {
        float actual_hz =
            static_cast<float>(cycle_count) * 1000.0f / elapsed_time;
        Serial.printf("# Update Frequency: %.2f Hz\n", actual_hz);
        cycle_count = 0;
        freq_calc_start = current_time;
      }
    }

    vTaskDelay(1);
  }
}
}  // namespace

void begin() {
  delay(500);
  Serial.begin(2000000);

  pinMode(kBuzzerPin, OUTPUT);
  pinMode(kRs485DeRePin, OUTPUT);
  digitalWrite(kRs485DeRePin, LOW);

  FastLED.addLeds<WS2812B, kWs2812Pin, GRB>(leds, kNumLeds);
  FastLED.setBrightness(50);
  leds[0] = CRGB::Orange;
  FastLED.show();

  playStartupSound();

  for (size_t i = 0; i < kNumEncoders; i++) {
    request_frames[i][0] = kEncoderIds[i];
    request_frames[i][1] = 0x03;
    request_frames[i][2] = 0x00;
    request_frames[i][3] = 0x01;
    request_frames[i][4] = 0x00;
    request_frames[i][5] = 0x01;
    uint16_t crc = calculateCRC(request_frames[i], 6);
    request_frames[i][6] = crc & 0xFF;
    request_frames[i][7] = (crc >> 8) & 0xFF;
  }

  Serial2.setRxBufferSize(256);
  Serial2.setTxBufferSize(256);
  Serial2.begin(2500000, SERIAL_8N1, kRs485RxPin, kRs485TxPin);

  for (int i = 0; i < 2; i++) {
    encoder_data_buffer[i].all_ok = false;
    encoder_data_buffer[i].seq_number = 0;
    for (size_t j = 0; j < kNumEncoders; j++) {
      encoder_data_buffer[i].values[j] = 0;
      encoder_data_buffer[i].status[j] = false;
    }
  }

  leds[0] = CRGB::Blue;
  FastLED.show();
  Serial.println("# System Ready - Ultra-Fast Dual-Core Mode (2.5Mbps)");
  Serial.println(
      "# Optimizations: No flush() + Aggressive timeouts + Dual-core separation");
#if DUAL_CORE_ENCODER_WDT_DISABLE
  esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(0));
#endif

  xTaskCreatePinnedToCore(taskDataAcquisition,
                          "DataAcq",
                          4096,
                          NULL,
                          configMAX_PRIORITIES - 1,
                          NULL,
                          0);

  xTaskCreatePinnedToCore(
      taskOutputDisplay, "Output", 4096, NULL, 1, NULL, 1);
}

void loop() { vTaskDelay(portMAX_DELAY); }
}  // namespace DualCoreEncoder
