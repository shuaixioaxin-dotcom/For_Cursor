## 目标

基于 `STM32F429ZIT6` 设计固件：**接收6路IMU串口自发数据（115200bps）**，统一暂存/解析后，**通过串口1（USART1）以 ≥50Hz** 高频打包发送；支持：

- **LED指示**：运行状态、6路串口活动、串口1发送活动
- **IMU电源控制**：6路GPIO控制IMU供电（规格：GPIO输出低电平掉电）
- **串口命令**：上位机经USART1发送 `reset`（无回车无换行）触发IMU掉电重启

> 推荐实现方式：**每路IMU UART 采用 DMA 循环接收 + IDLE中断切帧**，避免“每字节RXNE中断”导致CPU占用过高与丢包风险。

---

## 外设资源建议（F429）

F429在144pin封装上通常具备足够UART资源：`USART1/2/3/6 + UART4/5/7/8`（具体以CubeMX引脚复用为准）。

### 串口分配（建议）

- **USART1**：聚合后输出 + 接收命令（全双工）
- **USART2**：IMU1 RX（可不接TX）
- **USART3**：IMU2 RX
- **UART4**：IMU3 RX
- **UART5**：IMU4 RX
- **USART6**：IMU5 RX
- **UART7**：IMU6 RX
- （可选）**UART8**：调试log/备用

### 波特率与时钟

- IMU端：`115200 8N1`（无流控）
- USART1输出：推荐同为 `115200` 或更高（如 `460800/921600`），取决于你“整合帧大小”与上位机能力

> 粗略带宽核算：串口8N1实际有效字节吞吐约为 `baud/10` bytes/s。请确保“6路输入总量 + 你的聚合输出帧量”都远离饱和（建议留 ≥30%余量）。

---

## CubeMX（或CubeIDE）关键配置清单

### 1) 6路IMU串口（USART2/3/6, UART4/5/7）

- **Mode**：Asynchronous
- **Baud**：115200
- **NVIC**：
  - 使能对应 `USARTx global interrupt`（用于IDLE中断）
- **DMA**：
  - 为每个串口分配 `RX DMA`，模式选择 **Circular**
- **UART advanced features**：
  - 允许使用 `IDLE line detection interrupt`（代码中开启）

> 发送方向（TX）可以不使用，但CubeMX通常仍会生成TX配置；不接线即可。

### 2) USART1（聚合输出 + 命令输入）

- **TX**：建议用DMA（Normal即可）
- **RX**：建议用中断（或DMA+环形），用于接收 `reset`

### 3) LED与IMU电源控制GPIO

- LED1：运行指示（例如500ms翻转）
- LED2~LED7：对应IMU1~IMU6的串口接收活动（收到数据点亮/闪烁一段时间）
- LED8：USART1发送活动（发送时闪烁）
- IMU_PWR1~IMU_PWR6：GPIO输出（推挽），**低电平=掉电**（按你的备注）

### 4) 定时源（推荐）

用于保证“≥50Hz”固定节拍打包发送：

- 方案A：SysTick 1ms节拍 + 软件计数（最简单）
- 方案B：TIM6/TIM7 20ms中断（更干净）

---

## 软件架构（推荐）

### 数据流

1. 6路UART：DMA写入各自 `rx_dma_buf[i]`（循环）
2. UART中断（IDLE触发）：计算DMA新增字节段，写入软件环形缓冲 `rx_rb[i]`，并记录活动时间戳
3. 主循环/任务：从 `rx_rb[i]` 解析出“完整IMU帧”（或只做“原始字节段搬运”）
4. 每20ms（50Hz）打包一次：取6路“最新帧”组成聚合输出，通过USART1 TX DMA发送

### 聚合帧格式（示例）

> 你后续可以按上位机协议调整。下面给一个“可调试、可扩展”的二进制格式。

- `MAGIC(2)`：0xAA 0x55
- `VER(1)`：0x01
- `SEQ(2)`：递增序号
- `TS_MS(4)`：毫秒时间戳
- `N(1)`：通道数=6
- 循环6次：
  - `CH(1)`：1..6
  - `LEN(2)`：该IMU最新帧长度（0表示本周期无新帧）
  - `DATA(LEN)`：最新帧原始数据（不改动）
- `CRC16(2)`：对前面所有字段做CRC16（如MODBUS/IBM）

---

## reset命令行为（按你的要求）

当USART1收到ASCII字符串 **`reset`**（无回车无换行）：

1. 将 `IMU_PWR1~6` 全部拉低（掉电）
2. 延时例如 `200ms`
3. 将 `IMU_PWR1~6` 全部拉高（上电）
4. 延时例如 `1000ms`（等待IMU启动）
5. 清空6路接收缓冲/最新帧缓存

---

## 代码落地点（本仓库）

本仓库提供可直接拷贝进Cube工程的应用层文件：

- `firmware/app/app_imu_aggregator.c/.h`：6路DMA+IDLE接收、暂存区、聚合发送、LED控制
- `firmware/app/app_console.c/.h`：USART1命令解析（reset）
- `firmware/app/app_board.h`：GPIO映射（需要你按实际引脚改宏）

这些文件**不包含**Cube生成的HAL初始化（如 `MX_USART2_UART_Init()`），你只需要在Cube工程里：

- 生成UART/DMA/GPIO/TIM等初始化
- 在 `main()` 中调用 `APP_Init()` / `APP_Loop()` / `APP_Tick1ms()` 等接口

