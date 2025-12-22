## RS-485 16编码器轮询频率测试

本仓库提供 `encoder_benchmark.py`：通过串口连接RS-485(Modbus RTU)总线，轮询 16 个站号（`0x01`~`0x10`）编码器，读取寄存器并统计**有效帧/秒**与**轮询Hz**，用于测试“最大有效获取频率”。

### 安装

```bash
python3 -m pip install -r requirements.txt
```

### 运行示例

```bash
python3 encoder_benchmark.py --port /dev/ttyUSB0 --baudrate 115200 --duration 10
```

### 常用调参（逼近最大频率）

- **减少发送间隔**：`--tx-gap 0.0002`
- **减少发送后等待**：`--pre-rx-wait 0.0008`
- **缩短读窗口**：`--rx-window 0.002`
- **窗口内更频繁轮询**：`--rx-poll 0.0002`

示例：

```bash
python3 encoder_benchmark.py --port /dev/ttyUSB0 --baudrate 115200 --duration 10 \
  --tx-gap 0.0002 --pre-rx-wait 0.0008 --rx-window 0.002 --rx-poll 0.0002
```

### RS485模式（可选）

部分USB-RS485转换器/驱动支持 `pyserial` 的RS485自动收发控制，可尝试：

```bash
python3 encoder_benchmark.py --port /dev/ttyUSB0 --baudrate 115200 --rs485
```
