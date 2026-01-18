#pragma once

/**
 * 你需要按实际硬件连接修改本文件中的GPIO映射。
 *
 * 约定：
 * - IMU_PWRx：低电平=掉电；高电平=上电（与你的备注一致）
 * - LEDx：按你电路决定是高亮还是低亮；这里仅提供抽象宏，便于统一调整
 */

#include "stm32f4xx_hal.h"

// =========================
// LED GPIO 映射（示例占位）
// =========================
// 你可以把这些宏改成你实际的 GPIOx / GPIO_PIN_x

#ifndef LED1_GPIO_Port
#define LED1_GPIO_Port GPIOB
#define LED1_Pin       GPIO_PIN_0
#endif

#ifndef LED2_GPIO_Port
#define LED2_GPIO_Port GPIOB
#define LED2_Pin       GPIO_PIN_1
#endif

#ifndef LED3_GPIO_Port
#define LED3_GPIO_Port GPIOB
#define LED3_Pin       GPIO_PIN_2
#endif

#ifndef LED4_GPIO_Port
#define LED4_GPIO_Port GPIOB
#define LED4_Pin       GPIO_PIN_10
#endif

#ifndef LED5_GPIO_Port
#define LED5_GPIO_Port GPIOB
#define LED5_Pin       GPIO_PIN_11
#endif

#ifndef LED6_GPIO_Port
#define LED6_GPIO_Port GPIOB
#define LED6_Pin       GPIO_PIN_12
#endif

#ifndef LED7_GPIO_Port
#define LED7_GPIO_Port GPIOB
#define LED7_Pin       GPIO_PIN_13
#endif

#ifndef LED8_GPIO_Port
#define LED8_GPIO_Port GPIOB
#define LED8_Pin       GPIO_PIN_14
#endif

// 你可以在这里统一配置“点亮电平”
#ifndef LED_ACTIVE_LEVEL
#define LED_ACTIVE_LEVEL GPIO_PIN_SET
#endif

static inline void BOARD_LED_Write(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState on)
{
  // 如需反相，改这里即可
  HAL_GPIO_WritePin(port, pin, on);
}

static inline void BOARD_LED_On(GPIO_TypeDef* port, uint16_t pin)
{
  BOARD_LED_Write(port, pin, (GPIO_PinState)LED_ACTIVE_LEVEL);
}

static inline void BOARD_LED_Off(GPIO_TypeDef* port, uint16_t pin)
{
  BOARD_LED_Write(port, pin, (GPIO_PinState)(LED_ACTIVE_LEVEL == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET));
}

static inline void BOARD_LED_Toggle(GPIO_TypeDef* port, uint16_t pin)
{
  HAL_GPIO_TogglePin(port, pin);
}

// =============================
// IMU 电源控制 GPIO 映射（占位）
// =============================

#ifndef IMU1_PWR_GPIO_Port
#define IMU1_PWR_GPIO_Port GPIOC
#define IMU1_PWR_Pin       GPIO_PIN_0
#endif

#ifndef IMU2_PWR_GPIO_Port
#define IMU2_PWR_GPIO_Port GPIOC
#define IMU2_PWR_Pin       GPIO_PIN_1
#endif

#ifndef IMU3_PWR_GPIO_Port
#define IMU3_PWR_GPIO_Port GPIOC
#define IMU3_PWR_Pin       GPIO_PIN_2
#endif

#ifndef IMU4_PWR_GPIO_Port
#define IMU4_PWR_GPIO_Port GPIOC
#define IMU4_PWR_Pin       GPIO_PIN_3
#endif

#ifndef IMU5_PWR_GPIO_Port
#define IMU5_PWR_GPIO_Port GPIOC
#define IMU5_PWR_Pin       GPIO_PIN_4
#endif

#ifndef IMU6_PWR_GPIO_Port
#define IMU6_PWR_GPIO_Port GPIOC
#define IMU6_PWR_Pin       GPIO_PIN_5
#endif

static inline void BOARD_IMU_PowerOff_All(void)
{
  HAL_GPIO_WritePin(IMU1_PWR_GPIO_Port, IMU1_PWR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IMU2_PWR_GPIO_Port, IMU2_PWR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IMU3_PWR_GPIO_Port, IMU3_PWR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IMU4_PWR_GPIO_Port, IMU4_PWR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IMU5_PWR_GPIO_Port, IMU5_PWR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IMU6_PWR_GPIO_Port, IMU6_PWR_Pin, GPIO_PIN_RESET);
}

static inline void BOARD_IMU_PowerOn_All(void)
{
  HAL_GPIO_WritePin(IMU1_PWR_GPIO_Port, IMU1_PWR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU2_PWR_GPIO_Port, IMU2_PWR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU3_PWR_GPIO_Port, IMU3_PWR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU4_PWR_GPIO_Port, IMU4_PWR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU5_PWR_GPIO_Port, IMU5_PWR_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IMU6_PWR_GPIO_Port, IMU6_PWR_Pin, GPIO_PIN_SET);
}

