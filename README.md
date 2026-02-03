# For_Cursor
For testing of Cursor

## Modbus RTU 多从站批处理工具 🚀

本仓库包含用于Modbus RTU通信的高性能Python工具集，采用**批处理模式**显著提升多从站通信效率。

### ✨ 核心特性

- 🔥 **批处理模式**：批量发送请求，批量接收响应，性能提升40-85%
- ⚡ **高速通信**：优化的通信时序，支持921600波特率
- 📊 **多从站支持**：同时读取多个从站的ACC（加速度）和QUAT（四元数）
- 📈 **详细统计**：实时统计成功率、频率和耗时

### 🛠️ 工具列表

| 文件 | 说明 |
|------|------|
| `modbus_rtu_test.py` | 主工具 - 批处理模式多从站读取 |
| `quick_test.py` | 快速测试 - 自动运行多个测试场景 |
| `performance_comparison.py` | 性能对比 - 对比传统模式和批处理模式 |
| `MODBUS_USAGE.md` | 详细使用文档 |

### 🚀 快速开始

#### 1. 基本使用

```bash
# 默认从站0x01和0x02，COM66，最高频率
python modbus_rtu_test.py

# 指定串口
python modbus_rtu_test.py -p COM3 -b 115200

# 自定义从站列表（4个从站）
python modbus_rtu_test.py -s 1,2,3,4

# 限制批次数
python modbus_rtu_test.py -c 100
```

#### 2. 快速测试

```bash
# 运行预设测试场景
python quick_test.py

# 指定串口运行测试
python quick_test.py -p COM3
```

#### 3. 性能对比

```bash
# 对比传统模式和批处理模式
python performance_comparison.py

# 测试4个从站性能
python performance_comparison.py -s 1,2,3,4 -c 100
```

### 📊 性能优势

批处理模式相比传统循环模式：

- **2个从站**：提升约 40-60%
- **4个从站**：提升约 60-75%
- **8个从站**：提升约 75-85%

### 📖 详细文档

完整使用说明、参数详解、示例和故障排查请查看：[MODBUS_USAGE.md](MODBUS_USAGE.md)

### 📦 依赖

```bash
pip install pyserial
```

### 💡 使用提示

- 批处理模式通过连续发送请求减少等待时间
- 可通过 `-t` 参数调整从站间延迟（默认0.1ms）
- 支持自定义频率控制（`-f` 参数）
- 按 Ctrl+C 可随时停止并查看统计信息 
