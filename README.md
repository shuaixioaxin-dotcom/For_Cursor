# For_Cursor

本仓库包含一个 **ESP32 + RS485(Modbus RTU)** 的最小测试草图，用于**只读取 IMU 的 ACC 与 QUAT 寄存器**，以减少总线数据量并提升刷新率。

## 文件

- `modbus_imu_acc_quat_test.ino`
  - 读取两段寄存器并输出 CSV：
    - **ACC**：起始寄存器 `0x0034`，长度 `3`
    - **QUAT**：起始寄存器 `0x0046`，长度 `4`

## 关键参数（可按需修改）

- **串口**：
  - `Serial`：`2000000`
  - `Serial2(RS485)`：`921600`
- **RS485 方向控制**：`RS485_DE_RE_PIN`
- **IMU IDs**：`IMU_IDS[]`（默认 `{1,2}`）

## 输出格式

- **CSV**：每行输出所有 IMU 的 `id,accx,accy,accz,qw,qx,qy,qz`（重复）
- **频率**：每秒输出一次 `# Update Frequency: xxx Hz`

