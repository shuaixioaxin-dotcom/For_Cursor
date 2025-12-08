# 14路编码器数据采集与无线传输系统

## 项目概述

本项目实现一个基于 ESP32-S3 的编码器数据采集系统，通过 SPI 总线采集 14 个编码器的数据，简单处理后暂存，并通过按键触发无线传输至 Linux 设备。

## 系统架构

```
┌─────────────┐     SPI      ┌─────────────────┐     WiFi      ┌─────────────┐
│ 14路编码器   │ ──────────→ │   ESP32-S3      │ ──────────→  │  Linux设备   │
└─────────────┘              │   (主控MCU)     │               │  (接收端)   │
                             └─────────────────┘               └─────────────┘
                                    ↑
                               [按键触发]
```

## 核心功能

- ✅ SPI 总线读取 14 路编码器数据
- ✅ 数据滤波与角度转换处理
- ✅ 数据暂存至 RAM/Flash
- ✅ 按键触发数据发送
- ✅ WiFi 无线传输至 Linux 设备
- ✅ JSON 格式数据封装

## 为什么选择 ESP32-S3

| 评估维度 | ESP32-S3 | STM32+ESP01 | RP2040+ESP01 |
|---------|---------|-------------|--------------|
| **价格** | ¥20 | ¥14 | ¥15 |
| **PCB面积** | 小(单模块) | 大(双模块) | 大(双模块) |
| **开发难度** | 简单 | 中等 | 简单 |
| **集成度** | 高(WiFi内置) | 低 | 低 |
| **GPIO数量** | 45个(充足) | 37个 | 30个 |

**详细评估文档**: [docs/design_evaluation.md](docs/design_evaluation.md)

## 项目结构

```
.
├── README.md                      # 本文件
├── platformio.ini                 # PlatformIO 配置
├── src/
│   └── main.cpp                   # ESP32-S3 主程序
├── docs/
│   └── design_evaluation.md       # 详细设计评估文档
├── hardware/
│   └── schematic_notes.md         # 硬件原理图说明
└── linux_receiver/
    ├── receiver.py                # Linux 端接收程序
    └── requirements.txt           # Python 依赖
```

## 快速开始

### 1. 硬件准备

- ESP32-S3 开发板 (如 ESP32-S3-DevKitC-1)
- 编码器模块 (如 AS5048A)
- USB 数据线

### 2. 固件开发

```bash
# 安装 PlatformIO
pip install platformio

# 编译固件
pio run

# 上传固件
pio run --target upload

# 监视串口
pio device monitor
```

### 3. 配置 WiFi 和服务器

编辑 `src/main.cpp`:

```cpp
const char* WIFI_SSID     = "你的WiFi名称";
const char* WIFI_PASSWORD = "你的WiFi密码";
const char* SERVER_IP     = "Linux设备IP";
const uint16_t SERVER_PORT = 8888;
```

### 4. 启动 Linux 接收端

```bash
cd linux_receiver
pip install -r requirements.txt
python receiver.py --port 8888
```

### 5. 测试

1. ESP32 上电后自动连接 WiFi 并开始采集数据
2. 按下 BOOT 按键，数据将发送至 Linux 设备
3. Linux 端显示接收到的数据并保存到文件

## GPIO 分配

| GPIO | 功能 | 说明 |
|------|------|------|
| 11 | SPI_MOSI | 主机输出 |
| 12 | SPI_SCLK | 时钟 |
| 13 | SPI_MISO | 主机输入 |
| 1-10, 14-17 | CS0-CS13 | 14个编码器片选 |
| 0 | BUTTON | 发送触发 |
| 48 | LED | 状态指示 |

## 数据格式

ESP32 发送的 JSON 数据格式:

```json
{
  "device_id": "AA:BB:CC:DD:EE:FF",
  "sample_count": 100,
  "samples": [
    {
      "ts": 1234567,
      "values": [45.5, 90.0, 135.5, ...]
    }
  ]
}
```

## 扩展建议

### 1. 使用 MQTT 通信

```cpp
// 将 TCP Socket 替换为 MQTT
#include <PubSubClient.h>
```

### 2. 增加存储容量

```cpp
// 使用 SPIFFS/LittleFS 替代 NVS
#include <SPIFFS.h>
```

### 3. 低功耗优化

```cpp
// 添加深度睡眠
esp_deep_sleep_start();
```

## 成本估算

| 项目 | 价格 |
|------|------|
| ESP32-S3 模块 | ¥20 |
| 外围元件 | ¥5 |
| PCB | ¥3 |
| **总计** | **约 ¥28** |

(不含编码器)

## 参考资料

- [ESP32-S3 技术规格书](https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_cn.pdf)
- [ESP-IDF 编程指南](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/)
- [AS5048A 数据手册](https://ams.com/as5048a)

## License

MIT License
