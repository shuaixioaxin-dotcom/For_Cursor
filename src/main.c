/**
 * @file main.c
 * @brief Quad IMU Serial Port Data Reception Example
 * 
 * This example demonstrates how to receive data from 4 Inertial Measurement Units (IMUs) 
 * using USART2, USART3, UART4, UART5 and print the combined decoded results via USART1.
 * 
 * @hardware_connections:
 * - USART1 (PA09/PA10): Prints combined IMU data to the console
 * - USART2 (PA02/PA03): Receives data from IMU1
 * - USART3 (PB10/PB11): Receives data from IMU2
 * - UART4  (PC10/PC11): Receives data from IMU3
 * - UART5  (PC12/PD02): Receives data from IMU4
 * 
 * @software_configuration:
 * - ENABLE_FIXED_DATA_EXAMPLE: Set to 1 to enable fixed data array decoding example
 * - ENABLE_USART_DMA: Set to 1 to enable DMA for USART reception
 * - NUM_IMU: Number of IMU devices (4)
 * 
 * @program_flow:
 * 1. System Initialization:
 *    - Initialize delay, NVIC priority, and USART1 for debug output
 *    - Configure USART2/3/4/5 for IMU data reception
 *    - Optionally configure DMA for USART reception
 * 2. Print Welcome Information:
 *    - Print system clock frequencies and reception mode
 * 3. Main Loop:
 *    - Continuously process received IMU data from all 4 channels
 *    - Decode and combine the data
 *    - Send combined data via USART1
 * 
 * @date 2024-08-07
 * @version 2.0
 */
 
#include <string.h>
#include <stdio.h>
#include "delay.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "hipnuc_dec.h"

/* Enable/Disable fixed data array decoding example */
#define ENABLE_FIXED_DATA_EXAMPLE     0

/* Enable/Disable DMA for USART reception */
#define ENABLE_USART_DMA              0

/* Number of IMU devices */
#define NUM_IMU                       4

/* Baud rates */
#define USART1_BAUD 115200
#define USART2_BAUD 115200
#define USART3_BAUD 115200
#define UART4_BAUD  115200
#define UART5_BAUD  115200

#define UART_RX_BUF_SIZE        (512)
#define LOG_STRING_SIZE         (2048)
#define COMBINED_LOG_SIZE       (4096)

/* IMU index definitions */
#define IMU1_IDX    0
#define IMU2_IDX    1
#define IMU3_IDX    2
#define IMU4_IDX    3

/* IMU data structure for each channel */
typedef struct {
    hipnuc_raw_t raw;           /* IMU stream read/control struct */
    uint8_t rx_buf[UART_RX_BUF_SIZE];   /* Reception buffer */
    uint8_t dma_buf[UART_RX_BUF_SIZE];  /* DMA buffer (if DMA enabled) */
    uint16_t rx_index;          /* Current index in rx buffer */
    uint8_t new_data_flag;      /* 0: no new data, 1: new data arrived */
    uint8_t data_valid;         /* 0: data not valid, 1: data parsed successfully */
} imu_channel_t;

/* IMU channels array */
static imu_channel_t imu_channels[NUM_IMU];

/* The char buffer used to show individual results */
static char log_buf[LOG_STRING_SIZE];

/* The char buffer used for combined output */
static char combined_log_buf[COMBINED_LOG_SIZE];

/* Function prototypes */
static void USART_Configuration(void);
static void DMA_Configuration(void);
static void app_init(void);
static void printf_welcome_information(void);
static void process_all_imu_data(void);
static void process_imu_channel(uint8_t imu_idx);
static void send_combined_data(void);
static void handle_usart2_rx_idle(void);
static void handle_usart3_rx_idle(void);
static void handle_uart4_rx_idle(void);
static void handle_uart5_rx_idle(void);

/**
 * @brief Main program
 * 
 * Initializes the system and enters an infinite loop to process IMU data.
 * 
 * @return int 
 */
int main(void)
{
    app_init();
    printf_welcome_information();

    /* Use macro to control whether to show the fixed data array decoding example */
    #if ENABLE_FIXED_DATA_EXAMPLE
    #include "example_data.h"
    process_example_data();
    #endif
    
    while (1)
    {
        process_all_imu_data();
    }    
}

/**
 * @brief System initialization
 * 
 * Initializes delay, NVIC priority, all USARTs, and optionally DMA.
 * Also initializes all IMU instances.
 */
static void app_init(void)
{
    delay_init();
    delay_ms(1);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    /* Initialize all USARTs */
    USART_Configuration();
    
    #if ENABLE_USART_DMA
    /* Initialize DMA for all receiving USARTs */
    DMA_Configuration();
    #endif
    
    /* Initialize all IMU channel instances */
    for (uint8_t i = 0; i < NUM_IMU; i++)
    {
        memset(&imu_channels[i], 0, sizeof(imu_channel_t));
    }
}

/**
 * @brief Print system clock frequencies and reception mode
 */
static void printf_welcome_information(void)
{
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);

    printf("\r\n========================================\r\n");
    printf("HiPNUC Quad IMU Data Decode Example\r\n");
    printf("========================================\r\n");
    printf("Number of IMU channels: %d\r\n", NUM_IMU);
    printf("IMU1: USART2 (PA2/PA3)\r\n");
    printf("IMU2: USART3 (PB10/PB11)\r\n");
    printf("IMU3: UART4  (PC10/PC11)\r\n");
    printf("IMU4: UART5  (PC12/PD2)\r\n");
    printf("Output: USART1 (PA9/PA10)\r\n");
    
    #if ENABLE_USART_DMA
    printf("USART reception: DMA\r\n");
    #else
    printf("USART reception: UART Interrupt\r\n");
    #endif
    
    printf("System Clock Frequencies:\r\n");
    printf("  SYSCLK: %lu Hz\r\n", RCC_Clocks.SYSCLK_Frequency);
    printf("  HCLK: %lu Hz\r\n", RCC_Clocks.HCLK_Frequency);
    printf("========================================\r\n\r\n");
}

/**
 * @brief Process all IMU data from all channels
 * 
 * Checks each IMU channel for new data, processes it, and sends combined output.
 */
static void process_all_imu_data(void)
{
    uint8_t any_new_data = 0;
    
    /* Process each IMU channel */
    for (uint8_t i = 0; i < NUM_IMU; i++)
    {
        if (imu_channels[i].new_data_flag)
        {
            process_imu_channel(i);
            any_new_data = 1;
        }
    }
    
    /* If any channel has new valid data, send combined output */
    if (any_new_data)
    {
        send_combined_data();
    }
}

/**
 * @brief Process IMU data for a specific channel
 * 
 * @param imu_idx Index of the IMU channel (0-3)
 */
static void process_imu_channel(uint8_t imu_idx)
{
    imu_channel_t *ch = &imu_channels[imu_idx];
    
    if (!ch->new_data_flag)
        return;
    
    ch->new_data_flag = 0;
    ch->data_valid = 0;
    
    for (uint16_t i = 0; i < ch->rx_index; i++)
    {
        if (hipnuc_input(&ch->raw, ch->rx_buf[i]))
        {
            ch->data_valid = 1;
        }
    }
    
    ch->rx_index = 0; /* Reset buffer index after processing */
}

/**
 * @brief Send combined data from all valid IMU channels via USART1
 */
static void send_combined_data(void)
{
    int offset = 0;
    uint8_t valid_count = 0;
    
    /* Build header */
    offset += snprintf(combined_log_buf + offset, COMBINED_LOG_SIZE - offset,
                       "\r\n====== Combined IMU Data ======\r\n");
    
    /* Add data from each IMU channel */
    for (uint8_t i = 0; i < NUM_IMU; i++)
    {
        imu_channel_t *ch = &imu_channels[i];
        
        offset += snprintf(combined_log_buf + offset, COMBINED_LOG_SIZE - offset,
                          "--- IMU%d ", i + 1);
        
        if (ch->data_valid)
        {
            valid_count++;
            
            /* Get timestamp and basic info */
            offset += snprintf(combined_log_buf + offset, COMBINED_LOG_SIZE - offset,
                              "(Valid, Frame Len:%d) ---\r\n", ch->raw.len);
            
            /* Add accelerometer data */
            offset += snprintf(combined_log_buf + offset, COMBINED_LOG_SIZE - offset,
                              "  ACC: %.3f, %.3f, %.3f (m/s^2)\r\n",
                              ch->raw.hi91.acc[0], ch->raw.hi91.acc[1], ch->raw.hi91.acc[2]);
            
            /* Add gyroscope data */
            offset += snprintf(combined_log_buf + offset, COMBINED_LOG_SIZE - offset,
                              "  GYR: %.3f, %.3f, %.3f (deg/s)\r\n",
                              ch->raw.hi91.gyr[0], ch->raw.hi91.gyr[1], ch->raw.hi91.gyr[2]);
            
            /* Add magnetometer data */
            offset += snprintf(combined_log_buf + offset, COMBINED_LOG_SIZE - offset,
                              "  MAG: %.3f, %.3f, %.3f (uT)\r\n",
                              ch->raw.hi91.mag[0], ch->raw.hi91.mag[1], ch->raw.hi91.mag[2]);
            
            /* Add euler angles */
            offset += snprintf(combined_log_buf + offset, COMBINED_LOG_SIZE - offset,
                              "  EULER: Roll=%.2f, Pitch=%.2f, Yaw=%.2f (deg)\r\n",
                              ch->raw.hi91.euler[0], ch->raw.hi91.euler[1], ch->raw.hi91.euler[2]);
            
            /* Add quaternion */
            offset += snprintf(combined_log_buf + offset, COMBINED_LOG_SIZE - offset,
                              "  QUAT: %.4f, %.4f, %.4f, %.4f\r\n",
                              ch->raw.hi91.quat[0], ch->raw.hi91.quat[1], 
                              ch->raw.hi91.quat[2], ch->raw.hi91.quat[3]);
            
            /* Add pressure and timestamp if available */
            offset += snprintf(combined_log_buf + offset, COMBINED_LOG_SIZE - offset,
                              "  PRES: %.2f Pa, TS: %u ms\r\n",
                              ch->raw.hi91.prs, ch->raw.hi91.ts);
        }
        else
        {
            offset += snprintf(combined_log_buf + offset, COMBINED_LOG_SIZE - offset,
                              "(No Valid Data) ---\r\n");
        }
    }
    
    /* Add summary */
    offset += snprintf(combined_log_buf + offset, COMBINED_LOG_SIZE - offset,
                      "==============================\r\n");
    offset += snprintf(combined_log_buf + offset, COMBINED_LOG_SIZE - offset,
                      "Valid IMUs: %d/%d\r\n", valid_count, NUM_IMU);
    offset += snprintf(combined_log_buf + offset, COMBINED_LOG_SIZE - offset,
                      "==============================\r\n");
    
    /* Send via USART1 */
    printf("%s", combined_log_buf);
}

/**
 * @brief Configure all USARTs
 * 
 * - USART1: Debug output (TX only needed)
 * - USART2: IMU1 data reception (PA2-TX, PA3-RX)
 * - USART3: IMU2 data reception (PB10-TX, PB11-RX)
 * - UART4:  IMU3 data reception (PC10-TX, PC11-RX)
 * - UART5:  IMU4 data reception (PC12-TX, PD2-RX)
 */
static void USART_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable all required clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_USART3 | 
                          RCC_APB1Periph_UART4 | RCC_APB1Periph_UART5, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                          RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

    /* ============ USART1 Configuration (Debug Output) ============ */
    /* USART1 TX: PA9 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* USART1 RX: PA10 (optional, but configured) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ============ USART2 Configuration (IMU1) ============ */
    /* USART2 TX: PA2 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* USART2 RX: PA3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ============ USART3 Configuration (IMU2) ============ */
    /* USART3 TX: PB10 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* USART3 RX: PB11 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ============ UART4 Configuration (IMU3) ============ */
    /* UART4 TX: PC10 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    /* UART4 RX: PC11 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* ============ UART5 Configuration (IMU4) ============ */
    /* UART5 TX: PC12 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    /* UART5 RX: PD2 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* ============ Common USART Settings ============ */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    /* ============ Initialize USART1 (Debug Output) ============ */
    USART_InitStructure.USART_BaudRate = USART1_BAUD;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);

    /* ============ Initialize USART2 (IMU1) ============ */
    USART_InitStructure.USART_BaudRate = USART2_BAUD;
    USART_Init(USART2, &USART_InitStructure);
    USART_Cmd(USART2, ENABLE);
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
    #if !ENABLE_USART_DMA
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    #endif

    /* ============ Initialize USART3 (IMU2) ============ */
    USART_InitStructure.USART_BaudRate = USART3_BAUD;
    USART_Init(USART3, &USART_InitStructure);
    USART_Cmd(USART3, ENABLE);
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    #if !ENABLE_USART_DMA
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    #endif

    /* ============ Initialize UART4 (IMU3) ============ */
    USART_InitStructure.USART_BaudRate = UART4_BAUD;
    USART_Init(UART4, &USART_InitStructure);
    USART_Cmd(UART4, ENABLE);
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);
    #if !ENABLE_USART_DMA
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    #endif

    /* ============ Initialize UART5 (IMU4) ============ */
    USART_InitStructure.USART_BaudRate = UART5_BAUD;
    USART_Init(UART5, &USART_InitStructure);
    USART_Cmd(UART5, ENABLE);
    USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);
    #if !ENABLE_USART_DMA
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    #endif

    /* ============ NVIC Configuration for all receiving USARTs ============ */
    /* USART2 NVIC */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* USART3 NVIC */
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    /* UART4 NVIC */
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);

    /* UART5 NVIC */
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief Configure DMA for all receiving USARTs
 * 
 * - DMA1 Channel6: USART2 RX
 * - DMA1 Channel3: USART3 RX
 * - DMA2 Channel3: UART4 RX (Note: UART4/5 use DMA2)
 * - (UART5 does not support DMA on STM32F10x)
 */
static void DMA_Configuration(void)
{
    #if ENABLE_USART_DMA
    DMA_InitTypeDef DMA_InitStructure;

    /* Enable DMA1 and DMA2 clocks */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_DMA2, ENABLE);

    /* ============ DMA1 Channel6 for USART2 RX ============ */
    DMA_DeInit(DMA1_Channel6);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)imu_channels[IMU1_IDX].dma_buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = UART_RX_BUF_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel6, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel6, ENABLE);
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);

    /* ============ DMA1 Channel3 for USART3 RX ============ */
    DMA_DeInit(DMA1_Channel3);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)imu_channels[IMU2_IDX].dma_buf;
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel3, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);

    /* ============ DMA2 Channel3 for UART4 RX ============ */
    DMA_DeInit(DMA2_Channel3);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)imu_channels[IMU3_IDX].dma_buf;
    DMA_Init(DMA2_Channel3, &DMA_InitStructure);
    DMA_Cmd(DMA2_Channel3, ENABLE);
    USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);

    /* Note: UART5 does not have DMA capability on STM32F10x */
    /* UART5 will use interrupt-based reception even when DMA is enabled */
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    #endif
}

/* ============ Interrupt Handlers ============ */

/**
 * @brief USART2 interrupt handler (IMU1)
 */
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        /* Clear IDLE line detected bit */
        USART_ReceiveData(USART2);
        handle_usart2_rx_idle();
    }

    #if !ENABLE_USART_DMA
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        uint8_t ch = USART_ReceiveData(USART2);
        if (imu_channels[IMU1_IDX].rx_index < UART_RX_BUF_SIZE)
        {
            imu_channels[IMU1_IDX].rx_buf[imu_channels[IMU1_IDX].rx_index++] = ch;
        }
    }
    #endif
}

/**
 * @brief USART3 interrupt handler (IMU2)
 */
void USART3_IRQHandler(void)
{
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        /* Clear IDLE line detected bit */
        USART_ReceiveData(USART3);
        handle_usart3_rx_idle();
    }

    #if !ENABLE_USART_DMA
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        uint8_t ch = USART_ReceiveData(USART3);
        if (imu_channels[IMU2_IDX].rx_index < UART_RX_BUF_SIZE)
        {
            imu_channels[IMU2_IDX].rx_buf[imu_channels[IMU2_IDX].rx_index++] = ch;
        }
    }
    #endif
}

/**
 * @brief UART4 interrupt handler (IMU3)
 */
void UART4_IRQHandler(void)
{
    if (USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
    {
        /* Clear IDLE line detected bit */
        USART_ReceiveData(UART4);
        handle_uart4_rx_idle();
    }

    #if !ENABLE_USART_DMA
    if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
    {
        uint8_t ch = USART_ReceiveData(UART4);
        if (imu_channels[IMU3_IDX].rx_index < UART_RX_BUF_SIZE)
        {
            imu_channels[IMU3_IDX].rx_buf[imu_channels[IMU3_IDX].rx_index++] = ch;
        }
    }
    #endif
}

/**
 * @brief UART5 interrupt handler (IMU4)
 */
void UART5_IRQHandler(void)
{
    if (USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)
    {
        /* Clear IDLE line detected bit */
        USART_ReceiveData(UART5);
        handle_uart5_rx_idle();
    }

    /* UART5 always uses interrupt-based reception (no DMA support) */
    if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
    {
        uint8_t ch = USART_ReceiveData(UART5);
        if (imu_channels[IMU4_IDX].rx_index < UART_RX_BUF_SIZE)
        {
            imu_channels[IMU4_IDX].rx_buf[imu_channels[IMU4_IDX].rx_index++] = ch;
        }
    }
}

/* ============ IDLE Line Handlers ============ */

/**
 * @brief Handle USART2 RX IDLE (IMU1)
 */
static void handle_usart2_rx_idle(void)
{
    #if ENABLE_USART_DMA
    DMA_Cmd(DMA1_Channel6, DISABLE);
    uint16_t rx_size = UART_RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Channel6);
    
    for (uint16_t i = 0; i < rx_size; i++)
    {
        if (imu_channels[IMU1_IDX].rx_index < UART_RX_BUF_SIZE)
        {
            imu_channels[IMU1_IDX].rx_buf[imu_channels[IMU1_IDX].rx_index++] = 
                imu_channels[IMU1_IDX].dma_buf[i];
        }
    }
    
    DMA_SetCurrDataCounter(DMA1_Channel6, UART_RX_BUF_SIZE);
    DMA_Cmd(DMA1_Channel6, ENABLE);
    #endif
    
    imu_channels[IMU1_IDX].new_data_flag = 1;
}

/**
 * @brief Handle USART3 RX IDLE (IMU2)
 */
static void handle_usart3_rx_idle(void)
{
    #if ENABLE_USART_DMA
    DMA_Cmd(DMA1_Channel3, DISABLE);
    uint16_t rx_size = UART_RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Channel3);
    
    for (uint16_t i = 0; i < rx_size; i++)
    {
        if (imu_channels[IMU2_IDX].rx_index < UART_RX_BUF_SIZE)
        {
            imu_channels[IMU2_IDX].rx_buf[imu_channels[IMU2_IDX].rx_index++] = 
                imu_channels[IMU2_IDX].dma_buf[i];
        }
    }
    
    DMA_SetCurrDataCounter(DMA1_Channel3, UART_RX_BUF_SIZE);
    DMA_Cmd(DMA1_Channel3, ENABLE);
    #endif
    
    imu_channels[IMU2_IDX].new_data_flag = 1;
}

/**
 * @brief Handle UART4 RX IDLE (IMU3)
 */
static void handle_uart4_rx_idle(void)
{
    #if ENABLE_USART_DMA
    DMA_Cmd(DMA2_Channel3, DISABLE);
    uint16_t rx_size = UART_RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA2_Channel3);
    
    for (uint16_t i = 0; i < rx_size; i++)
    {
        if (imu_channels[IMU3_IDX].rx_index < UART_RX_BUF_SIZE)
        {
            imu_channels[IMU3_IDX].rx_buf[imu_channels[IMU3_IDX].rx_index++] = 
                imu_channels[IMU3_IDX].dma_buf[i];
        }
    }
    
    DMA_SetCurrDataCounter(DMA2_Channel3, UART_RX_BUF_SIZE);
    DMA_Cmd(DMA2_Channel3, ENABLE);
    #endif
    
    imu_channels[IMU3_IDX].new_data_flag = 1;
}

/**
 * @brief Handle UART5 RX IDLE (IMU4)
 * Note: UART5 does not support DMA on STM32F10x
 */
static void handle_uart5_rx_idle(void)
{
    imu_channels[IMU4_IDX].new_data_flag = 1;
}

/* ============ Printf Redirect ============ */

/**
 * @brief Redirect printf to USART1
 */
int fputc(int ch, FILE *f)
{
    (void)f;
    USART_SendData(USART1, (uint8_t)ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    return ch;
}
