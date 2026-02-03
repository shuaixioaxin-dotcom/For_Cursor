# For_Cursor
For testing of Cursor

## Modbus RTU 多从站循环读取工具

本仓库包含一个用于Modbus RTU通信的Python工具，支持在多个从站之间循环读取加速度和四元数数据。

### 快速开始

```bash
# 默认从站0x01和0x02，最高频率读取
python modbus_rtu_test.py

# 指定串口
python modbus_rtu_test.py -p COM3 -b 115200

# 自定义从站列表
python modbus_rtu_test.py -s 1,2,3

# 限制读取次数
python modbus_rtu_test.py -c 100
```

详细使用说明请查看 [MODBUS_USAGE.md](MODBUS_USAGE.md) 
