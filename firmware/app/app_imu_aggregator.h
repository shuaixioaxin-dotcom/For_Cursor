#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/**
 * IMU聚合器：6路UART(USART2/3/6 + UART4/5/7) DMA循环接收 + IDLE切帧，
 * 每20ms（默认50Hz）打包一次，走USART1发送。
 *
 * 你需要在CubeMX生成的工程里：
 * - 初始化GPIO/USART/DMA（RX DMA必须是Circular）
 * - 在main()里调用 APP_IMU_Init()
 * - 1ms节拍里调用 APP_IMU_Tick1ms()
 * - 在USART2/3/4/5/6/7 IRQ里调用 APP_IMU_OnUartIrq(&huartX)
 * - 在HAL_UART_TxCpltCallback里调用 APP_IMU_OnTxCplt(&huartX)
 */

#ifdef __cplusplus
extern "C" {
#endif

// =================
// 可调参数（按需改）
// =================

#ifndef APP_IMU_CH_COUNT
#define APP_IMU_CH_COUNT 6
#endif

// DMA接收缓冲大小（越大越不易溢出，但更占RAM）
#ifndef APP_IMU_DMA_RX_BUF_SZ
#define APP_IMU_DMA_RX_BUF_SZ 512
#endif

// 每路软件环形缓冲（用于主循环协议解析/调试）
#ifndef APP_IMU_RB_SZ
#define APP_IMU_RB_SZ 2048
#endif

// “最新一帧”最大长度：这里默认将“IDLE到来前收到的字节段”当作一帧
#ifndef APP_IMU_LATEST_FRAME_MAX
#define APP_IMU_LATEST_FRAME_MAX 256
#endif

// 聚合发送频率：默认20ms => 50Hz
#ifndef APP_IMU_AGG_PERIOD_MS
#define APP_IMU_AGG_PERIOD_MS 20
#endif

// reset掉电/上电等待时间
#ifndef APP_IMU_PWR_OFF_MS
#define APP_IMU_PWR_OFF_MS 200
#endif

#ifndef APP_IMU_PWR_ON_WAIT_MS
#define APP_IMU_PWR_ON_WAIT_MS 1000
#endif

typedef struct
{
  uint32_t tick_ms;     // 收到该帧时刻（ms）
  uint32_t seq;         // 每路自增
  uint16_t len;         // data有效长度
  uint8_t  data[APP_IMU_LATEST_FRAME_MAX];
} app_imu_latest_frame_t;

typedef struct
{
  // 每路最新帧（供50Hz打包）
  app_imu_latest_frame_t latest[APP_IMU_CH_COUNT];
} app_imu_snapshot_t;

void APP_IMU_Init(void);

// 在1ms定时（SysTick或TIM中断）中调用
void APP_IMU_Tick1ms(void);

// 可选：主循环里调用（当前实现不强依赖）
void APP_IMU_Loop(void);

// 在对应USARTx_IRQHandler中调用（建议：先HAL_UART_IRQHandler，再调用本函数）
void APP_IMU_OnUartIrq(UART_HandleTypeDef* huart);

// 在HAL_UART_TxCpltCallback里调用（用于USART1 TX DMA完成）
void APP_IMU_OnTxCplt(UART_HandleTypeDef* huart);

// 供console模块调用：请求执行IMU掉电重启（非阻塞状态机）
void APP_IMU_RequestPowerCycle(void);

// 读取当前“最新帧快照”（线程模型：主循环调用最安全）
void APP_IMU_GetSnapshot(app_imu_snapshot_t* out);

// 供调试：从某路的环形缓冲中读出原始字节
uint16_t APP_IMU_ReadRaw(uint8_t imu_index_0based, uint8_t* out, uint16_t maxlen);

#ifdef __cplusplus
}
#endif

