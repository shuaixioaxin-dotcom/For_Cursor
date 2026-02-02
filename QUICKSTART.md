# 快速开始指南

## 5分钟快速部署

### 前提条件

- [x] ESP32开发板 x 2（发送端 + 接收端）
- [x] RS485模块 x 3
- [x] 多摩川编码器 x 6
- [x] USB数据线 x 2
- [x] 已安装 PlatformIO

### 步骤1：获取接收端MAC地址（2分钟）

```bash
# 克隆项目
git clone <repository_url>
cd workspace

# 准备接收端代码
mv src/main.cpp src/main_transmitter.cpp.bak
mv src/receiver_example.cpp src/main.cpp

# 上传到第一块ESP32（接收端）
pio run -t upload

# 打开串口监视器
pio device monitor
```

记录显示的MAC地址，例如：`24:6F:28:AB:CD:EF`

### 步骤2：配置发送端（1分钟）

编辑 `include/config.h`：

```cpp
// 修改为接收端的MAC地址
#define RECEIVER_MAC {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF}
```

### 步骤3：上传发送端（2分钟）

```bash
# 恢复发送端代码
mv src/main.cpp src/receiver.cpp.bak
mv src/main_transmitter.cpp.bak src/main.cpp

# 连接第二块ESP32（发送端）
pio run -t upload
```

### 步骤4：连接硬件并测试

按照接线图连接：
- 3个RS485模块到ESP32
- 6个编码器到RS485总线（每路2个）
- 供电

查看接收端串口输出，应该能看到数据流！

## 常见首次运行问题

### Q1: 编译错误

```bash
# 清理并重新编译
pio run -t clean
pio run
```

### Q2: 上传失败

```bash
# 检查端口
pio device list

# 指定端口上传
pio run -t upload --upload-port /dev/ttyUSB0
```

### Q3: 无数据输出

1. 检查MAC地址配置是否正确
2. 确认两块ESP32都已上电
3. 检查距离是否过远（建议<5米测试）

### Q4: 编码器无响应

1. 检查RS485接线（A+/B-）
2. 确认编码器供电
3. 检查波特率是否匹配

## 下一步

- 📖 阅读完整 [README.md](README.md)
- 🔧 查看 [配置指南](docs/CONFIGURATION.md)
- 🔌 参考 [接线指南](docs/WIRING.md)
- 🧪 进行 [完整测试](docs/TESTING.md)

## 技术支持

遇到问题？检查：
1. 硬件连接是否正确
2. 配置文件是否修改
3. 固件是否上传成功
4. 串口监视器波特率（115200）

祝使用愉快！
