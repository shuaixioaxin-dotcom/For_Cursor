## CubeIDE / CubeMX 接入步骤（STM32F429ZIT6）

本仓库的 `firmware/app/` 提供“应用层模块”，你需要把它们拷贝到你的Cube工程（或以git子目录方式引入），并完成以下接线与回调挂接。

---

## 1) CubeMX 外设配置要点

### 串口

- **USART1**：TX(建议DMA) + RX(中断即可)，波特率按你上位机定（推荐 ≥115200）
- **USART2/USART3/UART4/UART5/USART6/UART7**：仅需RX即可（TX可不接），波特率 **115200 8N1**

### DMA（关键）

对 IMU 6路串口的 RX：

- 每路分配一个 **RX DMA**
- DMA Mode 选择 **Circular**
- NVIC 使能各自 `USARTx global interrupt`（因为我们用 **IDLE中断** 作为“切帧/搬运触发点”）

> 提醒：IDLE中断来源于UART外设，不是DMA中断，因此即使DMA是循环模式，也必须开USART IRQ。

### GPIO

- LED1~LED8：Output Push-Pull（按板子改）
- IMU_PWR1~IMU_PWR6：Output Push-Pull（**低电平掉电**）

---

## 2) 拷贝文件

把这些文件加入你的Cube工程（建议放到 `Core/Src` 与 `Core/Inc` 能包含到的位置）：

- `firmware/app/app_board.h`
- `firmware/app/app_ringbuf.h`
- `firmware/app/app_imu_aggregator.c/.h`
- `firmware/app/app_console.c/.h`

然后按你的实际引脚修改：

- `firmware/app/app_board.h`（LED与IMU电源GPIO映射、LED点亮电平）

---

## 3) 在 main() 中初始化与节拍调用

在 `main.c` 里（完成 `MX_GPIO_Init()`、`MX_USARTx_UART_Init()`、DMA初始化后）：

- 调用：
  - `APP_IMU_Init();`
  - `APP_Console_Init();`

在1ms节拍处调用（2选1）：

- 方案A：在 `SysTick_Handler()` 里（不建议写太多逻辑，但本模块很轻量）
- 方案B：用TIM6/TIM7 1ms中断，在回调里调用

调用函数：

- `APP_IMU_Tick1ms();`

主循环可选调用：

- `APP_IMU_Loop();`（当前为空壳，预留将来做协议解析/过滤）

---

## 4) USART2/3/4/5/6/7 的 IRQ 中挂接 IDLE 处理

在 `stm32f4xx_it.c` 的各个 `USARTx_IRQHandler()` 里：

1. 先执行Cube生成的 `HAL_UART_IRQHandler(&huartX);`
2. 再调用：
   - `APP_IMU_OnUartIrq(&huartX);`

示例（伪代码）：

```c
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
  APP_IMU_OnUartIrq(&huart2);
}
```

对 `USART3/UART4/UART5/USART6/UART7` 同理。

---

## 5) USART1 的 RX 命令接收（reset）

推荐做法：**中断逐字节接收**。

在 `main.c` 里启动一次接收（全局变量 `uint8_t u1_rx;`）：

```c
HAL_UART_Receive_IT(&huart1, &u1_rx, 1);
```

在 `stm32f4xx_hal_uart.c` 的回调（用户区）或你自己的 `HAL_UART_RxCpltCallback()` 中：

- 当 `huart==&huart1`：
  - `APP_Console_OnRxByte(u1_rx);`
  - 再次调用 `HAL_UART_Receive_IT(&huart1, &u1_rx, 1);` 续接下一字节

收到 `reset` 后，模块会调用 `APP_IMU_RequestPowerCycle()`，按状态机执行掉电/上电延时。

---

## 6) USART1 发送完成回调（LED8熄灭/释放busy）

在 `HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)` 里加：

```c
APP_IMU_OnTxCplt(huart);
```

---

## 7) 常见问题

- **IMU连续无空闲导致IDLE不触发？**
  - 大多数100Hz帧式输出会有间隔；若你的IMU真的是“无间隙流”，需要改为“定时搬运DMA写指针”方式切分。
- **环形缓冲溢出？**
  - 增大 `APP_IMU_RB_SZ` 或降低输出帧量/提升上位机波特率。
- **聚合输出长度太大导致USART1忙一直发不完？**
  - 提升USART1波特率（如921600），或缩短每路携带的数据（只发解析后的姿态/角速度等）。

