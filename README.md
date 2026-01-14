# For_Cursor
For testing of Cursor

## 多路 IMU 串口接收示例（4 IMU）

本仓库新增了一个面向 STM32F103（StdPeriph）的教育示例：**同时接收 4 路 IMU 串口数据**并用 `USART1` 连贯输出解析结果，核心代码在 `main.c`。

### 关键点

- **中断只收字节**：各路 UART 的 ISR 仅把收到的字节写入各自环形缓冲（不在中断里 `printf`）。
- **主循环统一解析/打印**：在主循环中依次从每路缓冲取数据喂给 `hipnuc_input()`，当解析出完整帧后**一次性打印**，避免输出被打断或不同 IMU 数据交错。

### 串口与引脚（STM32F103 常见默认映射）

- **USART1 (PA9/PA10)**：调试输出到串口助手
- **IMU1 -> USART2 (PA2/PA3)**：接收 IMU 数据
- **IMU2 -> USART3 (PB10/PB11)**：接收 IMU 数据
- **IMU3 -> UART4  (PC10/PC11)**：接收 IMU 数据
- **IMU4 -> UART5  (PC12/PD2)**：接收 IMU 数据

> 如果你的板级引脚复用/芯片型号不同，请按实际硬件修改 `main.c` 的 GPIO/USART 配置。

### 依赖

示例调用 HiPNUC 的解码接口（`hipnuc_dec.h`/`hipnuc_dec.c` 或等价实现）。仓库中默认只给出调用方式，具体库文件请按你的工程集成。
