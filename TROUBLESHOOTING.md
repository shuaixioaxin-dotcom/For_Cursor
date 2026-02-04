# 故障排查指南

## 问题1: 某个设备的数据不更新（一直显示相同值）

### 症状
```
ID: 1 | ACC:( -3.726,  4.341, -6.372)m/s² | QUAT:(0.4097,0.8710,0.0564,-0.2721)  ✅ 正常变化
ID: 2 | ACC:( -9.097, -0.586, -9.985)m/s² | QUAT:(-0.0669,-0.9470,-0.0091,-1.3798)  ❌ 一直不变
ID: 1 | ACC:( -6.694, -2.485, -8.174)m/s² | QUAT:(0.4081,0.8732,0.0507,0.4006)  ✅ 正常变化
ID: 2 | ACC:( -9.097, -0.586, -9.985)m/s² | QUAT:(-0.0669,-0.9470,-0.0091,-1.3798)  ❌ 一直不变
```

### 可能原因

#### 1. 等待时间不足
某些设备响应较慢，批量读取时等待时间不够。

**解决方案：** 使用修复后的版本（已增加等待时间）

```bash
# 重新运行程序
python optimized_imu_reader.py -p COM66 -b 921600 -i "1,2"
```

修复内容：
- 初始等待时间：0.001秒 → **0.005秒**
- 超时时间：0.001秒 → **0.02秒**
- 接收到足够数据时提前退出

---

#### 2. CRC校验失败
设备响应数据有误或传输错误导致CRC校验不通过。

**诊断方法：** 使用调试版本

```bash
python optimized_imu_reader_debug.py -p COM66 -b 921600 -i "1,2" -d -c 20
```

调试输出示例：
```
  === 开始解析响应 ===
  位置 0: 发现从站地址 1
    功能码: 0x03
    CRC计算: 0x1234, CRC接收: 0x1234
    ✅ CRC校验通过
  位置 49: 发现从站地址 2
    功能码: 0x03
    CRC计算: 0x5678, CRC接收: 0x9ABC
    ❌ CRC校验失败  ← 问题在这里
```

**解决方案：**
- 降低波特率：`-b 460800` 或 `-b 115200`
- 检查线缆质量
- 缩短线缆长度
- 减少电磁干扰

---

#### 3. 设备没有响应
设备未上电、地址错误或连接问题。

**诊断方法：**

```bash
# 使用调试版本查看响应情况
python optimized_imu_reader_debug.py -p COM66 -b 921600 -i "1,2" -d -c 10
```

如果看到：
```
  ⚠️ 从站 2 没有响应
```

**检查清单：**
- [ ] 设备是否上电
- [ ] 设备地址是否正确（使用设备配置工具确认）
- [ ] 线缆是否正确连接
- [ ] 设备是否支持该波特率

**测试单个设备：**
```bash
# 先测试 ID 1
python optimized_imu_reader.py -p COM66 -b 921600 -i 1 -c 10

# 再测试 ID 2
python optimized_imu_reader.py -p COM66 -b 921600 -i 2 -c 10
```

---

#### 4. 帧解析错误
响应帧格式不正确或数据错位。

**诊断方法：**

```bash
python optimized_imu_reader_debug.py -p COM66 -b 921600 -i "1,2" -d -c 5
```

查看原始数据：
```
  总接收数据: 98 字节
  数据内容: 01032c[数据]crc 02032c[数据]crc  ← 应该看到两个完整帧
```

**解决方案：** 使用修复后的版本（已改进帧解析逻辑）

---

#### 5. 设备响应顺序问题
如果设备响应顺序不稳定，可能导致解析错误。

**解决方案：** 修复后的版本使用智能帧定位，不依赖响应顺序

---

### 完整诊断流程

#### 步骤1: 使用调试版本运行
```bash
python optimized_imu_reader_debug.py -p COM66 -b 921600 -i "1,2" -d -c 20
```

#### 步骤2: 查看统计信息
```
调试统计信息
==================================================================
总读取次数: 20

各设备统计:

  从站 ID 1:
    成功帧: 20/20 (100.0%)  ✅
    CRC失败: 0
    缺失帧: 0

  从站 ID 2:
    成功帧: 0/20 (0.0%)     ❌ 问题在这里
    CRC失败: 10            ← CRC失败
    缺失帧: 10             ← 或者根本没响应
```

#### 步骤3: 根据统计结果采取措施

**如果CRC失败多：**
```bash
# 降低波特率重试
python optimized_imu_reader_debug.py -p COM66 -b 460800 -i "1,2" -c 20
```

**如果缺失帧多：**
```bash
# 测试单个设备
python optimized_imu_reader.py -p COM66 -b 921600 -i 2 -c 10
```

---

## 问题2: 性能不佳（频率低于预期）

### 症状
```
统计: 平均耗时=25.50ms 实际频率=39.2Hz
```
预期应该是100Hz以上，但实际只有39Hz。

### 解决方案

#### 1. 提高波特率
```bash
python optimized_imu_reader.py -p COM66 -b 921600 -i "1,2"  # 推荐
```

#### 2. 减少设备数量
如果不需要同时读取所有设备，可以分批读取：
```python
# 分批读取
reader1 = OptimizedIMUReader('COM66', 921600, [1, 2])
reader2 = OptimizedIMUReader('COM66', 921600, [3, 4])
```

#### 3. 调整等待时间（高级）
如果所有设备响应都很快，可以适当减少等待时间：

编辑 `optimized_imu_reader.py`：
```python
# 第167行附近
time.sleep(0.003)  # 从0.005减少到0.003
timeout = 0.015    # 从0.02减少到0.015
```

**警告：** 减少等待时间可能导致某些设备响应不及时。

---

## 问题3: 所有设备数据都不更新

### 症状
所有设备的数据都是0或固定值。

### 可能原因

#### 1. 串口未连接
```bash
# 检查串口设备
# Windows
mode  

# Linux
ls -l /dev/ttyUSB*
```

#### 2. 波特率不匹配
```bash
# 尝试常见波特率
python optimized_imu_reader.py -p COM66 -b 9600 -i 1 -c 5
python optimized_imu_reader.py -p COM66 -b 115200 -i 1 -c 5
python optimized_imu_reader.py -p COM66 -b 921600 -i 1 -c 5
```

#### 3. 从站地址错误
```bash
# 尝试不同的从站ID
for i in 1 2 3 4 5; do
    echo "测试 ID $i"
    python optimized_imu_reader.py -p COM66 -b 921600 -i $i -c 3
done
```

#### 4. 设备协议不匹配
确认设备确实使用Modbus RTU协议，寄存器地址是0x0034，长度是0x0016。

---

## 问题4: 偶尔出现数据跳变

### 症状
```
ID: 1 | ACC:( -3.726,  4.341, -6.372)m/s²  ✅
ID: 1 | ACC:( -3.725,  4.340, -6.370)m/s²  ✅
ID: 1 | ACC:(999.999,999.999,999.999)m/s²  ❌ 异常跳变
ID: 1 | ACC:( -3.724,  4.339, -6.368)m/s²  ✅
```

### 解决方案

#### 1. 增加数据验证
```python
from optimized_imu_reader import OptimizedIMUReader

reader = OptimizedIMUReader('COM66', 921600, [1, 2])
reader.connect()

def is_valid_acc(acc):
    """验证加速度数据是否合理"""
    return all(-50 < val < 50 for val in acc)  # m/s²

def is_valid_quat(quat):
    """验证四元数数据是否合理"""
    magnitude = sum(q**2 for q in quat) ** 0.5
    return 0.8 < magnitude < 1.2  # 四元数模应该接近1

while True:
    imu_data = reader.read_all_imu_optimized()
    
    for slave_id, data in imu_data.items():
        if is_valid_acc(data['acc']) and is_valid_quat(data['quat']):
            # 使用数据
            process_data(data)
        else:
            print(f"⚠️ ID {slave_id} 数据异常，跳过")
```

#### 2. 降低波特率减少传输错误
```bash
python optimized_imu_reader.py -p COM66 -b 460800 -i "1,2"
```

---

## 问题5: 程序运行一段时间后崩溃

### 症状
程序运行一段时间后出现异常或停止响应。

### 解决方案

#### 1. 添加异常处理
```python
from optimized_imu_reader import OptimizedIMUReader
import time

reader = OptimizedIMUReader('COM66', 921600, [1, 2])
reader.connect()

error_count = 0
max_errors = 10

while error_count < max_errors:
    try:
        imu_data = reader.read_all_imu_optimized()
        error_count = 0  # 成功后重置错误计数
        
        # 处理数据
        for slave_id, data in imu_data.items():
            print(f"ID {slave_id}: {data}")
        
        time.sleep(0.01)
        
    except Exception as e:
        error_count += 1
        print(f"❌ 错误 {error_count}/{max_errors}: {e}")
        
        # 尝试重连
        reader.disconnect()
        time.sleep(1)
        reader.connect()

reader.disconnect()
```

#### 2. 添加看门狗
```python
import threading
import time

last_update_time = time.time()
watchdog_timeout = 5.0  # 5秒超时

def watchdog():
    global last_update_time
    while True:
        if time.time() - last_update_time > watchdog_timeout:
            print("⚠️ 看门狗超时，重启读取器")
            reader.disconnect()
            time.sleep(1)
            reader.connect()
            last_update_time = time.time()
        time.sleep(1)

# 启动看门狗线程
watchdog_thread = threading.Thread(target=watchdog, daemon=True)
watchdog_thread.start()

# 主循环
while True:
    imu_data = reader.read_all_imu_optimized()
    last_update_time = time.time()  # 更新时间戳
    # 处理数据...
```

---

## 调试工具使用指南

### 基本调试
```bash
# 启用调试输出，运行20次
python optimized_imu_reader_debug.py -p COM66 -b 921600 -i "1,2" -d -c 20
```

### 查看原始数据
调试模式会显示：
- 发送的请求帧
- 接收的数据块大小
- 原始数据十六进制
- 帧解析过程
- CRC校验结果
- 统计信息

### 理解调试输出

```
  发送请求 ID=1: 01030034001645ca
  发送请求 ID=2: 02030034001645f9
  读取数据块: 49 字节
  读取数据块: 49 字节
  总接收数据: 98 字节
  数据内容: 01032c...crc02032c...crc
  
  === 开始解析响应 ===
  位置 0: 发现从站地址 1
    功能码: 0x03
    CRC计算: 0x45ca, CRC接收: 0x45ca
    ✅ CRC校验通过
  位置 49: 发现从站地址 2
    功能码: 0x03
    CRC计算: 0x45f9, CRC接收: 0x45f9
    ✅ CRC校验通过
  找到 2 个有效响应帧
```

---

## 快速参考

### 命令对照表

| 场景 | 命令 |
|------|------|
| 正常使用 | `python optimized_imu_reader.py -p COM66 -b 921600 -i "1,2"` |
| 调试模式 | `python optimized_imu_reader_debug.py -p COM66 -b 921600 -i "1,2" -d -c 20` |
| 性能测试 | `python test_performance.py -p COM66 -b 921600 -i "1,2" -c 100` |
| 单设备测试 | `python optimized_imu_reader.py -p COM66 -b 921600 -i 2 -c 10` |
| 低波特率 | `python optimized_imu_reader.py -p COM66 -b 115200 -i "1,2"` |

### 常见波特率

| 波特率 | 单帧时间 | 适用场景 |
|--------|---------|---------|
| 9600 | ~50ms | 测试/调试 |
| 115200 | ~4ms | 低速采集 |
| 460800 | ~1ms | 中速采集 |
| 921600 | ~0.5ms | 高速采集（推荐） |

---

## 获取帮助

如果以上方法都无法解决问题，请提供以下信息：

1. **调试输出**
```bash
python optimized_imu_reader_debug.py -p COM66 -b 921600 -i "1,2" -d -c 20 > debug.log 2>&1
```

2. **设备信息**
- 设备型号
- 从站地址
- 支持的波特率
- Modbus寄存器地址

3. **系统信息**
- 操作系统
- Python版本
- PySerial版本

4. **问题描述**
- 具体症状
- 出现频率
- 已尝试的解决方案
