#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * USART1命令解析：
 * - 收到ASCII 'reset'（无换行无回车）触发 APP_IMU_RequestPowerCycle()
 *
 * 使用方式：
 * - 在main里调用 APP_Console_Init()
 * - 在HAL_UART_RxCpltCallback里，当huart==&huart1时调用 APP_Console_OnRxByte(b)
 * - 并持续重新开启下一字节接收（HAL_UART_Receive_IT）
 */

void APP_Console_Init(void);
void APP_Console_OnRxByte(uint8_t b);

#ifdef __cplusplus
}
#endif

