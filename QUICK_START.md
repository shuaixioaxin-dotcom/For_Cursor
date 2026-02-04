# 快速入门指南

## 📦 文件说明

- `optimized_imu_reader.py` - 优化的IMU读取器核心代码
- `test_performance.py` - 性能测试工具
- `IMU_OPTIMIZATION_README.md` - 详细技术文档
- `OPTIMIZATION_COMPARISON.md` - 优化前后对比
- `QUICK_START.md` - 本文档（快速入门）

---

## 🚀 快速开始

### 1. 基本使用

#### 读取单个IMU设备

```bash
python optimized_imu_reader.py -p COM66 -b 921600 -i 2
```

#### 读取多个IMU设备

```bash
python optimized_imu_reader.py -p COM66 -b 921600 -i "2,3,4,5"
```

#### 指定读取频率

```bash
# 以100Hz频率读取
python optimized_imu_reader.py -p COM66 -b 921600 -i 2 -f 100
```

#### 读取指定次数后停止

```bash
# 读取1000次后停止
python optimized_imu_reader.py -p COM66 -b 921600 -i 2 -c 1000
```

---

### 2. 性能测试

#### 测试单个配置

```bash
# 测试双设备性能，运行100次
python test_performance.py -p COM66 -b 921600 -i "2,3" -c 100
```

#### 对比多种配置

```bash
# 自动对比单设备、双设备、三设备、四设备的性能
python test_performance.py -p COM66 -b 921600 --compare -c 100
```

输出示例：
```
==================================================================
性能对比总结
==================================================================

配置          设备数    平均耗时(ms)     最大频率(Hz)     吞吐量         
----------------------------------------------------------------------
单设备        1        5.234           191.06          191.06         
双设备        2        6.123           163.33          326.66         
三设备        3        7.891           126.72          380.16         
四设备        4        9.456           105.75          423.00         

📈 相对单设备的性能提升:
  双设备: 速度比=0.85x, 吞吐量比=1.71x
  三设备: 速度比=0.66x, 吞吐量比=1.99x
  四设备: 速度比=0.55x, 吞吐量比=2.21x
==================================================================
```

---

## 💻 在代码中使用

### 基本示例

```python
from optimized_imu_reader import OptimizedIMUReader

# 创建读取器
reader = OptimizedIMUReader(
    port='COM66',
    baudrate=921600,
    slave_ids=[2, 3, 4]
)

# 连接串口
if reader.connect():
    # 读取数据
    imu_data = reader.read_all_imu_optimized()
    
    # 访问数据
    for slave_id in [2, 3, 4]:
        acc = imu_data[slave_id]['acc']      # (x, y, z) in m/s²
        quat = imu_data[slave_id]['quat']    # (w, x, y, z)
        
        print(f"设备{slave_id}:")
        print(f"  加速度: {acc}")
        print(f"  四元数: {quat}")
    
    # 断开连接
    reader.disconnect()
```

### 持续读取示例

```python
from optimized_imu_reader import OptimizedIMUReader
import time

reader = OptimizedIMUReader('COM66', 921600, [2, 3, 4])
reader.connect()

try:
    while True:
        # 批量读取所有设备
        imu_data = reader.read_all_imu_optimized()
        
        # 处理数据
        for slave_id, data in imu_data.items():
            print(f"设备{slave_id}: ACC={data['acc']}, QUAT={data['quat']}")
        
        time.sleep(0.01)  # 100Hz
        
except KeyboardInterrupt:
    print("停止读取")
finally:
    reader.disconnect()
```

### 带日志的示例

```python
from optimized_imu_reader import OptimizedIMUReader
import logging

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 创建带日志的读取器
reader = OptimizedIMUReader(
    port='COM66',
    baudrate=921600,
    slave_ids=[2, 3, 4],
    logger=logger
)

reader.connect()
imu_data = reader.read_all_imu_optimized()
reader.disconnect()
```

---

## 🔧 常见问题

### Q1: 串口连接失败怎么办？

**A:** 检查以下几点：
1. 串口号是否正确（Windows: COM1, COM2等；Linux: /dev/ttyUSB0等）
2. 波特率是否匹配设备
3. 串口是否被其他程序占用
4. 是否有足够的权限（Linux可能需要sudo）

```bash
# Linux下检查串口
ls -l /dev/ttyUSB*

# 添加用户到dialout组
sudo usermod -a -G dialout $USER
```

---

### Q2: 读取的数据全是0怎么办？

**A:** 可能原因：
1. 从站ID不正确
2. 设备未上电或未连接
3. Modbus地址不匹配
4. 波特率设置错误

解决方法：
```bash
# 先测试单个设备
python optimized_imu_reader.py -p COM66 -b 921600 -i 2 -c 10

# 如果失败，尝试其他从站ID
python optimized_imu_reader.py -p COM66 -b 921600 -i 1 -c 10
```

---

### Q3: 如何调整读取频率？

**A:** 有两种方式：

**方式1：命令行参数**
```bash
python optimized_imu_reader.py -p COM66 -b 921600 -i 2 -f 100  # 100Hz
```

**方式2：代码中控制**
```python
import time

while True:
    imu_data = reader.read_all_imu_optimized()
    # 处理数据...
    time.sleep(0.01)  # 100Hz (1/100 = 0.01秒)
```

---

### Q4: 多设备时如何获得最佳性能？

**A:** 优化建议：

1. **使用高波特率**
```python
# 推荐使用高波特率
reader = OptimizedIMUReader('COM66', 921600, [2,3,4])  # ✅
# 避免低波特率
reader = OptimizedIMUReader('COM66', 9600, [2,3,4])    # ❌
```

2. **批量读取而非单独读取**
```python
# 推荐：一次读取所有设备
imu_data = reader.read_all_imu_optimized()  # ✅

# 避免：循环读取单个设备
for id in [2,3,4]:  # ❌
    reader.read_single_device(id)
```

3. **调整等待时间**（高级用户）

如果需要极致性能，可以修改源码中的等待时间：
```python
# optimized_imu_reader.py 第169-175行
time.sleep(0.001)  # 可以尝试减小这个值

# 第179行
while time.time() - start_time < 0.001:  # 可以尝试调整超时时间
```

---

### Q5: CRC校验失败率高怎么办？

**A:** CRC校验失败通常表示数据传输错误：

1. **检查硬件连接**
   - 串口线是否松动
   - 线缆质量是否良好
   - 线缆长度是否过长（建议<5米）

2. **降低波特率**
```bash
# 尝试降低波特率
python optimized_imu_reader.py -p COM66 -b 115200 -i 2  # 从921600降到115200
```

3. **减少设备数量**
```bash
# 先测试单个设备
python optimized_imu_reader.py -p COM66 -b 921600 -i 2
```

4. **增加等待时间**

修改源码中的等待时间，给设备更多响应时间。

---

### Q6: 如何在多线程环境中使用？

**A:** 读取器已经内置线程安全机制：

```python
import threading
from optimized_imu_reader import OptimizedIMUReader

reader = OptimizedIMUReader('COM66', 921600, [2, 3, 4])
reader.connect()

def read_thread():
    while True:
        imu_data = reader.read_all_imu_optimized()  # 线程安全
        # 处理数据...

# 创建多个读取线程
threads = []
for i in range(3):
    t = threading.Thread(target=read_thread)
    t.start()
    threads.append(t)

# 等待线程结束
for t in threads:
    t.join()
```

**注意：** 虽然代码是线程安全的，但建议使用单个线程读取，多线程处理数据。

---

## 📊 性能指标参考

### 不同配置下的预期性能

| 设备数 | 波特率 | 平均耗时 | 最大频率 | 吞吐量 |
|--------|--------|----------|----------|--------|
| 1      | 921600 | ~5ms     | ~200Hz   | 200次/秒 |
| 2      | 921600 | ~6ms     | ~166Hz   | 332次/秒 |
| 4      | 921600 | ~10ms    | ~100Hz   | 400次/秒 |
| 8      | 921600 | ~18ms    | ~55Hz    | 440次/秒 |

### 波特率对性能的影响

| 波特率 | 单帧传输时间 | 4设备总时间 | 推荐场景 |
|--------|-------------|-------------|---------|
| 9600   | ~50ms       | ~200ms      | 测试用 |
| 115200 | ~4ms        | ~16ms       | 低速采集 |
| 460800 | ~1ms        | ~4ms        | 中速采集 |
| 921600 | ~0.5ms      | ~2ms        | 高速采集 ✅ |

---

## 🎯 最佳实践

### 1. 开发阶段

```bash
# 先测试单设备确认通信正常
python optimized_imu_reader.py -p COM66 -b 921600 -i 2 -c 10

# 然后逐步增加设备数量
python optimized_imu_reader.py -p COM66 -b 921600 -i "2,3" -c 10
python optimized_imu_reader.py -p COM66 -b 921600 -i "2,3,4" -c 10

# 最后进行性能测试
python test_performance.py -p COM66 -b 921600 --compare -c 100
```

### 2. 生产部署

```python
from optimized_imu_reader import OptimizedIMUReader
import logging
import time

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('imu_reader.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# 创建读取器
reader = OptimizedIMUReader(
    port='COM66',
    baudrate=921600,
    slave_ids=[2, 3, 4, 5],
    logger=logger
)

# 连接并验证
if not reader.connect():
    logger.error("无法连接到串口")
    exit(1)

# 主循环
try:
    while True:
        try:
            imu_data = reader.read_all_imu_optimized()
            
            # 数据处理逻辑
            for slave_id, data in imu_data.items():
                # 处理ACC和QUAT数据
                process_imu_data(slave_id, data)
                
        except Exception as e:
            logger.error(f"读取数据失败: {e}")
            time.sleep(0.1)  # 出错后稍作延迟
            
except KeyboardInterrupt:
    logger.info("用户中断")
finally:
    reader.disconnect()
    logger.info("已断开连接")
```

### 3. 错误处理

```python
from optimized_imu_reader import OptimizedIMUReader
import time

reader = OptimizedIMUReader('COM66', 921600, [2, 3, 4])

max_retries = 3
retry_count = 0

while retry_count < max_retries:
    if reader.connect():
        print("连接成功")
        break
    else:
        retry_count += 1
        print(f"连接失败，重试 {retry_count}/{max_retries}")
        time.sleep(1)
else:
    print("无法连接到设备")
    exit(1)

# 主循环带重连机制
error_count = 0
max_errors = 10

while error_count < max_errors:
    try:
        imu_data = reader.read_all_imu_optimized()
        error_count = 0  # 成功后重置错误计数
        
        # 处理数据...
        
    except Exception as e:
        error_count += 1
        print(f"错误 {error_count}/{max_errors}: {e}")
        
        if error_count >= max_errors:
            print("错误次数过多，退出")
            break
            
        # 尝试重连
        reader.disconnect()
        time.sleep(1)
        reader.connect()

reader.disconnect()
```

---

## 🆘 获取帮助

### 查看帮助信息

```bash
# 查看主程序帮助
python optimized_imu_reader.py --help

# 查看测试工具帮助
python test_performance.py --help
```

### 启用详细日志

```python
import logging

# 启用DEBUG级别日志
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

reader = OptimizedIMUReader('COM66', 921600, [2], logger=logger)
```

### 调试模式

```python
# 在代码中添加调试输出
def _parse_responses(self, response_data: bytes, requests: List[Tuple]):
    print(f"收到数据长度: {len(response_data)} 字节")
    print(f"数据内容: {response_data.hex()}")
    # ... 原有代码 ...
```

---

## 📚 更多资源

- **详细技术文档**: 参阅 `IMU_OPTIMIZATION_README.md`
- **性能对比分析**: 参阅 `OPTIMIZATION_COMPARISON.md`
- **源代码**: `optimized_imu_reader.py`
- **性能测试**: `test_performance.py`

---

## ✅ 检查清单

在部署前请确认：

- [ ] 串口连接正常
- [ ] 从站ID配置正确
- [ ] 波特率设置匹配
- [ ] 单设备测试通过
- [ ] 多设备测试通过
- [ ] 性能测试满足需求
- [ ] 错误处理机制完善
- [ ] 日志记录配置完成
- [ ] 数据验证逻辑正确

---

祝使用愉快！🎉
