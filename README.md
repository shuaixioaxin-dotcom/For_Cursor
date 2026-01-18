# IMU 6路串口聚合转发（STM32F429ZIT6）

本仓库提供一套可落地的固件架构与代码骨架：**6路IMU串口自发（115200bps）接收**，通过 **DMA循环接收 + UART IDLE中断切帧** 统一暂存，并以 **≥50Hz** 频率在 **USART1** 上打包发送；支持LED指示与串口1接收 `reset` 命令触发IMU掉电重启。

## 目录

- `docs/stm32f429_imu_aggregator_design.md`：外设分配建议、协议与架构说明
- `docs/cubeide_integration_steps.md`：如何接入CubeIDE/CubeMX工程（回调挂接/初始化步骤）
- `firmware/app/`：应用层源码（可直接拷贝进你的Cube工程）

## 你需要做的事（最少步骤）

- 在CubeMX里配置：USART1 + USART2/3/6 + UART4/5/7，并给6路IMU串口RX分配 **DMA Circular**
- 把 `firmware/app/` 拷贝进工程并按 `app_board.h` 改LED与电源GPIO映射
- 按 `docs/cubeide_integration_steps.md` 把IRQ与回调挂接起来

