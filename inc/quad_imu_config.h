/**
 * @file quad_imu_config.h
 * @brief Configuration header for Quad IMU data handling
 * 
 * This file contains all configuration macros and definitions for the
 * quad IMU data reception and transmission system.
 * 
 * @date 2024-08-07
 * @version 2.0
 */

#ifndef __QUAD_IMU_CONFIG_H
#define __QUAD_IMU_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============ Feature Enable/Disable ============ */

/**
 * @brief Enable/Disable fixed data array decoding example
 * Set to 1 to enable, 0 to disable
 */
#define ENABLE_FIXED_DATA_EXAMPLE     0

/**
 * @brief Enable/Disable DMA for USART reception
 * Set to 1 to enable DMA, 0 to use interrupt-based reception
 * Note: UART5 does not support DMA on STM32F10x
 */
#define ENABLE_USART_DMA              0

/* ============ IMU Configuration ============ */

/**
 * @brief Number of IMU devices connected
 */
#define NUM_IMU                       4

/**
 * @brief IMU channel index definitions
 */
#define IMU1_IDX    0   /* Connected to USART2 */
#define IMU2_IDX    1   /* Connected to USART3 */
#define IMU3_IDX    2   /* Connected to UART4 */
#define IMU4_IDX    3   /* Connected to UART5 */

/* ============ UART Baud Rates ============ */

/**
 * @brief USART1 baud rate (Debug output)
 */
#define USART1_BAUD 115200

/**
 * @brief USART2 baud rate (IMU1)
 */
#define USART2_BAUD 115200

/**
 * @brief USART3 baud rate (IMU2)
 */
#define USART3_BAUD 115200

/**
 * @brief UART4 baud rate (IMU3)
 */
#define UART4_BAUD  115200

/**
 * @brief UART5 baud rate (IMU4)
 */
#define UART5_BAUD  115200

/* ============ Buffer Sizes ============ */

/**
 * @brief Size of UART receive buffer for each channel
 */
#define UART_RX_BUF_SIZE        (512)

/**
 * @brief Size of log string buffer for individual IMU data
 */
#define LOG_STRING_SIZE         (2048)

/**
 * @brief Size of combined log buffer for all IMU data output
 */
#define COMBINED_LOG_SIZE       (4096)

/* ============ Hardware Pin Mapping ============ */

/*
 * USART1 (Debug Output):
 *   TX: PA9
 *   RX: PA10
 *
 * USART2 (IMU1):
 *   TX: PA2
 *   RX: PA3
 *
 * USART3 (IMU2):
 *   TX: PB10
 *   RX: PB11
 *
 * UART4 (IMU3):
 *   TX: PC10
 *   RX: PC11
 *
 * UART5 (IMU4):
 *   TX: PC12
 *   RX: PD2
 */

/* ============ DMA Channel Mapping ============ */

/*
 * DMA1 Channel6: USART2 RX
 * DMA1 Channel3: USART3 RX
 * DMA2 Channel3: UART4 RX
 * UART5: No DMA support on STM32F10x (uses interrupt mode)
 */

#ifdef __cplusplus
}
#endif

#endif /* __QUAD_IMU_CONFIG_H */
