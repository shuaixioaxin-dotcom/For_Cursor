/**
 * @file main.c
 * @brief Serial Port Data Reception Example for 4 IMUs with Trigger Control
 * 
 * This example demonstrates how to receive data from 4 Inertial Measurement Units (IMUs) 
 * using USART2, USART3, UART4, and UART5, and print the decoded results via USART1.
 * It also implements a trigger mechanism via USART1 commands ("imu1", "imu2", etc.)
 * to pulse GPIO pins connected to the IMUs.
 * 
 * @hardware_connections:
 * - USART1 (PA09/PA10): Prints result to the console (Debug) & Receives commands
 * - USART2 (PA02/PA03): IMU 1
 * - USART3 (PB10/PB11): IMU 2
 * - UART4  (PC10/PC11): IMU 3
 * - UART5  (PC12/PD02): IMU 4
 * - Trigger 1: PA4
 * - Trigger 2: PA5
 * - Trigger 3: PA6
 * - Trigger 4: PA7
 * 
 * @software_configuration:
 * - ENABLE_USART_DMA: Set to 0 (DMA not implemented for all channels in this multi-IMU example to save resources/complexity)
 * 
 * @date 2024-08-07
 * @version 1.3
 */
 
#include "delay.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "misc.h"
#include "hipnuc_dec.h"
#include <stdio.h>
#include <string.h>

/* Enable/Disable DMA for USART reception - Disabled for multi-channel simplicity */
#define ENABLE_USART_DMA              0

#define USART1_BAUD 115200
#define IMU_BAUD    115200

#define IMU_COUNT               4
#define UART_RX_BUF_SIZE        (1024)
#define LOG_STRING_SIZE         (1024)
#define CMD_BUF_SIZE            32

/* Trigger Pin Definitions */
#define TRIG_PIN_1 GPIO_Pin_4
#define TRIG_PORT_1 GPIOA
#define TRIG_PIN_2 GPIO_Pin_5
#define TRIG_PORT_2 GPIOA
#define TRIG_PIN_3 GPIO_Pin_6
#define TRIG_PORT_3 GPIOA
#define TRIG_PIN_4 GPIO_Pin_7
#define TRIG_PORT_4 GPIOA

/* IMU stream read/control structs for 4 IMUs */
static hipnuc_raw_t hipnuc_raw[IMU_COUNT];

/* Data arrived flags: 0: no new data, 1: new data */
static volatile uint8_t new_data_flag[IMU_COUNT] = {0};

/* Processed packet ready flags */
static uint8_t packet_ready[IMU_COUNT] = {0};

/* Trigger request flags */
static volatile uint8_t trigger_req[IMU_COUNT] = {0};

/* The char buffer used to show result */
static char log_buf[LOG_STRING_SIZE];

/* Rx buffers for 4 IMUs */
static uint8_t uart_rx_buf[IMU_COUNT][UART_RX_BUF_SIZE];
static volatile uint16_t uart_rx_index[IMU_COUNT] = {0};

/* Command buffer for USART1 */
static char cmd_buf[CMD_BUF_SIZE];
static uint8_t cmd_idx = 0;

/* Function prototypes */
static void System_Configuration(void);
static void USART1_Configuration(void);
static void IMU_USART_Configuration(void);
static void IMU_Trigger_Configuration(void);
static void app_init(void);
//static void printf_welcome_information(void);
static void process_data(void);
static void process_triggers(void);

/**
 * @brief Main program
 */
int main(void)
{
    app_init();
//    printf_welcome_information();
    
    while (1)
    {
        process_triggers();
        process_data();
    }    
}

/**
 * @brief System initialization
 */
static void app_init(void)
{
    delay_init();
    delay_ms(1);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    /* Initialize Trigger GPIOs */
    IMU_Trigger_Configuration();
    
    /* Initialize USART1 for debug output */
    USART1_Configuration();
    
    /* Initialize USARTs for IMU data reception */
    IMU_USART_Configuration();
    
    /* Initialize instances */
    for(int i=0; i<IMU_COUNT; i++) {
        memset(&hipnuc_raw[i], 0, sizeof(hipnuc_raw_t));
        new_data_flag[i] = 0;
        uart_rx_index[i] = 0;
        packet_ready[i] = 0;
        trigger_req[i] = 0;
    }
    
    memset(cmd_buf, 0, CMD_BUF_SIZE);
}

/**
 * @brief Process IMU Triggers
 */
static void process_triggers(void)
{
    for(int i=0; i<IMU_COUNT; i++) {
        if(trigger_req[i]) {
            trigger_req[i] = 0;
            
            GPIO_TypeDef* port;
            uint16_t pin;
            
            switch(i) {
                case 0: port = TRIG_PORT_1; pin = TRIG_PIN_1; break;
                case 1: port = TRIG_PORT_2; pin = TRIG_PIN_2; break;
                case 2: port = TRIG_PORT_3; pin = TRIG_PIN_3; break;
                case 3: port = TRIG_PORT_4; pin = TRIG_PIN_4; break;
                default: continue;
            }
            
            // Pulse the pin (Active High pulse assumed, or Low->High->Low)
            // Assuming default Low, pulse High.
            GPIO_SetBits(port, pin);
            delay_ms(10); // 10ms pulse width
            GPIO_ResetBits(port, pin);
            
            //printf("Triggered IMU %d\r\n", i+1);
        }
    }
}

/**
 * @brief Process IMU data from all channels
 */
static void process_data(void)
{
    static uint16_t wait_timer = 0;
    uint8_t any_new_decoded = 0;

    // 1. Process received data from each channel
    for (int i = 0; i < IMU_COUNT; i++)
    {
        if (new_data_flag[i])
        {
            uint16_t current_len = uart_rx_index[i];
            
            // Mark processed for this interrupt batch
            new_data_flag[i] = 0; 
            
            // Processing loop
            for (uint16_t k = 0; k < current_len; k++)
            {
                if (hipnuc_input(&hipnuc_raw[i], uart_rx_buf[i][k]))
                {
                    /* Packet decoded successfully */
                    packet_ready[i] = 1;
                    any_new_decoded = 1;
                }
            }
            
            // Reset buffer index safely
            IRQn_Type irq_n;
            switch(i) {
                case 0: irq_n = USART2_IRQn; break;
                case 1: irq_n = USART3_IRQn; break;
                case 2: irq_n = UART4_IRQn; break;
                case 3: irq_n = UART5_IRQn; break;
                default: irq_n = USART2_IRQn; break;
            }
            
            NVIC_DisableIRQ(irq_n);
            uart_rx_index[i] = 0;
            NVIC_EnableIRQ(irq_n);
        }
    }

    // 2. Logic for dynamic batch printing
    // If any new packet arrived and we are not already waiting, start the timer.
    if (any_new_decoded && wait_timer == 0)
    {
        wait_timer = 10; // Start 10ms window (assuming ~1ms loop with delay_ms(1))
    }

    // If timer is active, count down
    if (wait_timer > 0)
    {
        wait_timer--;
        
        // Check states
        uint8_t all_ready = 1;
        uint8_t any_ready = 0;
        
        for (int i = 0; i < IMU_COUNT; i++)
        {
            if (!packet_ready[i]) {
                all_ready = 0;
            } else {
                any_ready = 1;
            }
        }

        // Trigger print if:
        // A) All 4 IMUs have data (Optimization to print immediately)
        // B) Timeout expired (Print whatever we have, e.g. 2 or 3 IMUs)
        if (all_ready || wait_timer == 0)
        {
            if (any_ready)
            {
                // Print combined data block
                for (int i = 0; i < IMU_COUNT; i++)
                {
                    if (packet_ready[i])
                    {
                        // Format packet for this IMU
                        hipnuc_dump_packet(&hipnuc_raw[i], log_buf, sizeof(log_buf));
												printf("[IMU %d]:%s\r\n", i + 1,log_buf);
                        
                        // Clear ready flag
                        packet_ready[i] = 0;
                    }
                }
                printf("\r\n"); // Extra newline for separation
            }
            
            // Stop/Reset timer
            wait_timer = 0;
        }
    }
}

/**
 * @brief Configure Trigger GPIOs
 */
static void IMU_Trigger_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // Configure PA4, PA5, PA6, PA7 as Push-Pull Output
    GPIO_InitStructure.GPIO_Pin = TRIG_PIN_1 | TRIG_PIN_2 | TRIG_PIN_3 | TRIG_PIN_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // Set default Low
    GPIO_ResetBits(TRIG_PORT_1, TRIG_PIN_1);
    GPIO_ResetBits(TRIG_PORT_2, TRIG_PIN_2);
    GPIO_ResetBits(TRIG_PORT_3, TRIG_PIN_3);
    GPIO_ResetBits(TRIG_PORT_4, TRIG_PIN_4);
}

/**
 * @brief Configure USART1 for Debug
 */
static void USART1_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    // PA10 RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA9 TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = USART1_BAUD;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
    USART_Init(USART1, &USART_InitStructure); 
    USART_Cmd(USART1, ENABLE);
    
    // Enable Interrupts for USART1
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // Lower priority than IMUs? Or same.
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief Configure USART2, USART3, UART4, UART5
 */
static void IMU_USART_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Enable Clocks
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_USART3 | 
                           RCC_APB1Periph_UART4 | RCC_APB1Periph_UART5, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

    // Common USART Config
    USART_InitStructure.USART_BaudRate = IMU_BAUD;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    // --- USART2 (PA2 TX, PA3 RX) ---
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    USART_Init(USART2, &USART_InitStructure);
    USART_Cmd(USART2, ENABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // --- USART3 (PB10 TX, PB11 RX) ---
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    USART_Init(USART3, &USART_InitStructure);
    USART_Cmd(USART3, ENABLE);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    // --- UART4 (PC10 TX, PC11 RX) ---
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    USART_Init(UART4, &USART_InitStructure);
    USART_Cmd(UART4, ENABLE);
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    // --- UART5 (PC12 TX, PD2 RX) ---
    // RX is PD2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    // TX is PC12
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    USART_Init(UART5, &USART_InitStructure);
    USART_Cmd(UART5, ENABLE);
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}

// --- Interrupt Handlers ---

void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t ch = USART_ReceiveData(USART1);
        
        // Simple command parser
        // Append char to buffer
        if (cmd_idx < CMD_BUF_SIZE - 1) {
             cmd_buf[cmd_idx++] = ch;
             cmd_buf[cmd_idx] = '\0';
        }
        
        // Check for commands
        // We check if the buffer *ends* with the command string to be more robust against partials or noise
        // or just substring search.
        
        if (strstr(cmd_buf, "imu1")) {
            trigger_req[0] = 1;
            cmd_idx = 0; memset(cmd_buf, 0, CMD_BUF_SIZE);
        } else if (strstr(cmd_buf, "imu2")) {
            trigger_req[1] = 1;
            cmd_idx = 0; memset(cmd_buf, 0, CMD_BUF_SIZE);
        } else if (strstr(cmd_buf, "imu3")) {
            trigger_req[2] = 1;
            cmd_idx = 0; memset(cmd_buf, 0, CMD_BUF_SIZE);
        } else if (strstr(cmd_buf, "imu4")) {
            trigger_req[3] = 1;
            cmd_idx = 0; memset(cmd_buf, 0, CMD_BUF_SIZE);
        }
        
        // Reset on newline or full
        if (ch == '\r' || ch == '\n' || cmd_idx >= CMD_BUF_SIZE - 1) {
            cmd_idx = 0;
            memset(cmd_buf, 0, CMD_BUF_SIZE);
        }
    }
}

void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint8_t ch = USART_ReceiveData(USART2);
        if(uart_rx_index[0] < UART_RX_BUF_SIZE) {
            uart_rx_buf[0][uart_rx_index[0]++] = ch;
        }
    }
    if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) {
        USART_ReceiveData(USART2); // Clear IDLE
        new_data_flag[0] = 1;
    }
}

void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        uint8_t ch = USART_ReceiveData(USART3);
        if(uart_rx_index[1] < UART_RX_BUF_SIZE) {
            uart_rx_buf[1][uart_rx_index[1]++] = ch;
        }
    }
    if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET) {
        USART_ReceiveData(USART3); 
        new_data_flag[1] = 1;
    }
}

void UART4_IRQHandler(void)
{
    if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) {
        uint8_t ch = USART_ReceiveData(UART4);
        if(uart_rx_index[2] < UART_RX_BUF_SIZE) {
            uart_rx_buf[2][uart_rx_index[2]++] = ch;
        }
    }
    if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET) {
        USART_ReceiveData(UART4); 
        new_data_flag[2] = 1;
    }
}

void UART5_IRQHandler(void)
{
    if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) {
        uint8_t ch = USART_ReceiveData(UART5);
        if(uart_rx_index[3] < UART_RX_BUF_SIZE) {
            uart_rx_buf[3][uart_rx_index[3]++] = ch;
        }
    }
    if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET) {
        USART_ReceiveData(UART5); 
        new_data_flag[3] = 1;
    }
}
