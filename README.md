# IMU数据采集系统 - 200Hz高频优化版本

> 针对多IMU设备的高性能数据采集系统，实现200Hz采集频率

## 🚀 主要特性

- ✅ **200Hz采集频率**：通过批量输出和参数优化实现
- ✅ **批量处理机制**：减少75%的串口输出开销
- ✅ **双输出模式**：文本模式（调试）和二进制模式（高性能）
- ✅ **可配置参数**：根据需求灵活调整
- ✅ **Python工具集**：完整的数据接收和解析工具

## 📁 项目结构

```
.
├── src/
│   ├── main.cpp          # 主程序（优化后）
│   └── config.h          # 配置参数说明
├── tools/
│   ├── binary_reader.py  # 二进制数据接收器
│   ├── text_reader.py    # 文本数据接收器
│   └── requirements.txt  # Python依赖
├── platformio.ini        # PlatformIO配置
└── README_CN.md          # 详细文档
```

## 🎯 快速开始

### 1. 硬件连接

```
ESP32 -> RS485模块 -> IMU设备（并联）
```

详见 [README_CN.md](README_CN.md#硬件连接)

### 2. 编译上传

```bash
# 使用PlatformIO
pio run --target upload

# 监视输出
pio device monitor -b 921600
```

### 3. 数据接收

```bash
# Python工具
cd tools
pip install -r requirements.txt

# 文本模式
python text_reader.py /dev/ttyUSB0

# 二进制模式
python binary_reader.py /dev/ttyUSB0
```

## 📊 性能对比

| 版本 | 采集频率 | 输出开销 | CPU占用 |
|------|----------|----------|---------|
| 原版 | ~100Hz | 高 | 中 |
| **优化版** | **~200Hz** | **低** | **低** |

**优化提升**：频率提升100%，资源消耗降低50%

## 🔧 核心优化

1. **批量输出**（⭐最重要）：收集多个样本后统一输出
2. **参数调优**：降低超时和延迟时间
3. **代码简化**：移除不必要的复杂逻辑
4. **二进制模式**：可选的高效输出格式

详见 [README_CN.md](README_CN.md#核心优化策略)

## 📖 文档

- [详细文档（中文）](README_CN.md) - 完整的优化说明和使用指南
- [工具文档](tools/README.md) - Python工具使用说明
- [配置说明](src/config.h) - 参数配置指南

## 🛠️ 配置示例

```cpp
// src/main.cpp 顶部

// 平衡模式（推荐）
#define BATCH_SIZE 4              // 50Hz输出，200Hz采集
#define OUTPUT_BINARY false        // 文本输出

// 高性能模式
#define BATCH_SIZE 8              // 25Hz输出，210Hz采集
#define OUTPUT_BINARY true         // 二进制输出
```

## 📈 实测数据

测试环境：ESP32 @ 240MHz，2个IMU设备

| BATCH_SIZE | 采集频率 | 输出频率 | CPU占用 |
|------------|----------|----------|---------|
| 2          | 190Hz    | 95Hz     | 中 |
| 4          | 200Hz    | 50Hz     | 低 |
| 8          | 210Hz    | 26Hz     | 极低 |

## 🤝 贡献

欢迎提交Issue和Pull Request！

## 📄 许可证

MIT License 
