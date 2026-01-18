/**
 * @file main.c
 * @brief Serial Port Data Reception Example for 6 IMUs (4 HW + 2 SW) and IO Control
 * 
 * This example demonstrates how to receive data from 6 Inertial Measurement Units (IMUs).
 * Channels 1-4 use Hardware USARTs.
 * Channels 5-6 use Software Serial (GPIO + Timer).
 * 
 * @hardware_connections:
 * - USART1 (PA09/PA10): Debug & Commands
 * - USART2 (PA02/PA03): IMU 1
 * - USART3 (PB10/PB11): IMU 2
 * - UART4  (PC10/PC11): IMU 3
 * - UART5  (PC12/PD02): IMU 4
 * - SoftSerial1 (PB00): IMU 5 (RX only)
 * - SoftSerial2 (PB01): IMU 6 (RX only)
 * - IO Control (PA01): Outputs 3.3V ("reset") 
 * 
 * @software_configuration:
 * - ENABLE_USART_DMA: Set to 0
 * 
 * @date 2026-01-18
 * @version 1.2
 */
 
#include "delay.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "hipnuc_dec.h"
#include <stdio.h>
#include <string.h>

/* Enable/Disable DMA for USART reception - Disabled for multi-channel simplicity */
#define ENABLE_USART_DMA              0

#define USART1_BAUD 115200
#define IMU_BAUD    115200

#define IMU_COUNT               6
#define UART_RX_BUF_SIZE        (1024)
#define LOG_STRING_SIZE         (1024)

/* Control IO Macros */
#define CTRL_GPIO_PORT          GPIOA
#define CTRL_GPIO_PIN           GPIO_Pin_1
#define CTRL_GPIO_CLK           RCC_APB2Periph_GPIOA

/* Soft Serial Macros */
// IMU 5
#define SS1_PORT                GPIOB
#define SS1_PIN                 GPIO_Pin_0
#define SS1_CLK                 RCC_APB2Periph_GPIOB
#define SS1_EXTI_LINE           EXTI_Line0
#define SS1_EXTI_PORT           GPIO_PortSourceGPIOB
#define SS1_EXTI_PIN            GPIO_PinSource0
#define SS1_EXTI_IRQ            EXTI0_IRQn
#define SS1_TIM                 TIM6
#define SS1_TIM_CLK             RCC_APB1Periph_TIM6
#define SS1_TIM_IRQ             TIM6_IRQn

// IMU 6
#define SS2_PORT                GPIOB
#define SS2_PIN                 GPIO_Pin_1
#define SS2_CLK                 RCC_APB2Periph_GPIOB
#define SS2_EXTI_LINE           EXTI_Line1
#define SS2_EXTI_PORT           GPIO_PortSourceGPIOB
#define SS2_EXTI_PIN            GPIO_PinSource1
#define SS2_EXTI_IRQ            EXTI1_IRQn
#define SS2_TIM                 TIM7
#define SS2_TIM_CLK             RCC_APB1Periph_TIM7
#define SS2_TIM_IRQ             TIM7_IRQn

/* IMU stream read/control structs for 6 IMUs */
static hipnuc_raw_t hipnuc_raw[IMU_COUNT];

/* Data arrived flags: 0: no new data, 1: new data */
static volatile uint8_t new_data_flag[IMU_COUNT] = {0};

/* Processed packet ready flags */
static uint8_t packet_ready[IMU_COUNT] = {0};

/* The char buffer used to show result */
static char log_buf[LOG_STRING_SIZE];

/* Rx buffers for 6 IMUs */
static uint8_t uart_rx_buf[IMU_COUNT][UART_RX_BUF_SIZE];
static volatile uint16_t uart_rx_index[IMU_COUNT] = {0};

/* Command buffer for USART1 */
#define CMD_BUF_SIZE 20
static char usart1_rx_buf[CMD_BUF_SIZE];
static volatile uint8_t usart1_rx_idx = 0;

/* Soft Serial State Machine */
typedef struct {
    uint8_t state;       // 0: IDLE, 1: START, 2: DATA, 3: STOP
    uint8_t bit_cnt;     // 0-7
    uint8_t data_byte;   // Buffer for current byte
} SoftSerial_State_t;

volatile SoftSerial_State_t ss1_state = {0};
volatile SoftSerial_State_t ss2_state = {0};

/* Function prototypes */
static void USART1_Configuration(void);
static void Control_GPIO_Configuration(void);
static void IMU_USART_Configuration(void);
static void SoftSerial_Configuration(void);
static void app_init(void);
static void printf_welcome_information(void);
static void process_data(void);

/**
 * @brief Main program
 */
int main(void)
{
    app_init();
    printf_welcome_information();
    while (1)
    {
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
    
    /* Initialize USART1 for debug output and command input */
    USART1_Configuration();
    
    /* Initialize Control GPIO */
    Control_GPIO_Configuration();

    /* Initialize USARTs for IMU data reception */
    IMU_USART_Configuration();
    
    /* Initialize Soft Serial */
    SoftSerial_Configuration();
    
    /* Initialize instances */
    for(int i=0; i<IMU_COUNT; i++) {
        memset(&hipnuc_raw[i], 0, sizeof(hipnuc_raw_t));
        new_data_flag[i] = 0;
        uart_rx_index[i] = 0;
        packet_ready[i] = 0;
    }
}

/**
 * @brief Print system clock frequencies and reception mode
 */
static void printf_welcome_information(void)
{
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);
    printf("HiPNUC 6-Channel IMU Data Integration \r\n");
    printf("Channels 1-4: Hardware USART\r\n");
    printf("Channels 5-6: Software Serial (PB0, PB1)\r\n");
    printf("SYSCLK: %d Hz\r\n", RCC_Clocks.SYSCLK_Frequency);
    printf("HCLK: %d Hz\r\n", RCC_Clocks.HCLK_Frequency);
}

/**
 * @brief Process IMU data from all channels
 */
static void process_data(void)
{
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
                }
            }
            
            // Reset buffer index safely
            IRQn_Type irq_n;
            switch(i) {
                case 0: irq_n = USART2_IRQn; break;
                case 1: irq_n = USART3_IRQn; break;
                case 2: irq_n = UART4_IRQn; break;
                case 3: irq_n = UART5_IRQn; break;
                // Soft serial use different mechanism, but we need to protect access
                case 4: irq_n = SS1_TIM_IRQ; break; 
                case 5: irq_n = SS2_TIM_IRQ; break;
                default: irq_n = USART2_IRQn; break;
            }
            
            NVIC_DisableIRQ(irq_n);
            uart_rx_index[i] = 0;
            NVIC_EnableIRQ(irq_n);
        }
    }

    // 2. Check if all 6 IMUs have valid data ready
    if (packet_ready[0] && packet_ready[1] && packet_ready[2] && 
        packet_ready[3] && packet_ready[4] && packet_ready[5])
    {
        for (int i = 0; i < IMU_COUNT; i++)
        {
            // Format packet for this IMU
            hipnuc_dump_packet(&hipnuc_raw[i], log_buf, sizeof(log_buf));
            printf("[IMU%d]:%s\r\n", i + 1, log_buf);
            
            // Clear ready flag
            packet_ready[i] = 0;
        }
        printf("\r\n"); 
    }
}

/**
 * @brief Configure USART1 for Debug and Commands
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
    
    // Enable Receive Interrupt
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    // Configure NVIC for USART1
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

/**
 * @brief Configure Control GPIO (PA1)
 */
static void Control_GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(CTRL_GPIO_CLK, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = CTRL_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CTRL_GPIO_PORT, &GPIO_InitStructure);
    
    GPIO_SetBits(CTRL_GPIO_PORT, CTRL_GPIO_PIN); // Default 3.3V
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
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
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

/**
 * @brief Configure Software Serial (GPIO + EXTI + Timer)
 */
static void SoftSerial_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    // 1. Enable Clocks
    RCC_APB2PeriphClockCmd(SS1_CLK | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(SS1_TIM_CLK | SS2_TIM_CLK, ENABLE);

    // 2. Configure GPIOs (Inputs with PullUp)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    // SS1
    GPIO_InitStructure.GPIO_Pin = SS1_PIN;
    GPIO_Init(SS1_PORT, &GPIO_InitStructure);
    
    // SS2
    GPIO_InitStructure.GPIO_Pin = SS2_PIN;
    GPIO_Init(SS2_PORT, &GPIO_InitStructure);

    // 3. Configure EXTI
    // SS1 (PB0)
    GPIO_EXTILineConfig(SS1_EXTI_PORT, SS1_EXTI_PIN);
    EXTI_InitStructure.EXTI_Line = SS1_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // Start bit
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // SS2 (PB1)
    GPIO_EXTILineConfig(SS2_EXTI_PORT, SS2_EXTI_PIN);
    EXTI_InitStructure.EXTI_Line = SS2_EXTI_LINE;
    EXTI_Init(&EXTI_InitStructure);

    // 4. Configure Timers (Basic Timers TIM6/TIM7)
    // Clock is usually PCLK1 x 2 = 72MHz or 36MHz. Assuming 72MHz for calculation.
    // Baud 115200 -> 8.68us bit time.
    // We want a tick resolution. Let's say 1us. Prescaler = 72-1 -> 1MHz counter.
    // Bit time = 8.68 ticks. That's a bit low res.
    // Let's use Prescaler = 0 (72MHz). Bit time = 625 ticks.
    
    TIM_TimeBaseStructure.TIM_Period = 625; // Initial - will be changed
    TIM_TimeBaseStructure.TIM_Prescaler = 0; // 72MHz counter
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
    TIM_TimeBaseInit(SS1_TIM, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(SS2_TIM, &TIM_TimeBaseStructure);
    
    TIM_ITConfig(SS1_TIM, TIM_IT_Update, ENABLE);
    TIM_ITConfig(SS2_TIM, TIM_IT_Update, ENABLE);

    // Don't enable Timers yet, EXTI will do it.

    // 5. Configure NVIC
    // EXTI
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    
    NVIC_InitStructure.NVIC_IRQChannel = SS1_EXTI_IRQ;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = SS2_EXTI_IRQ;
    NVIC_Init(&NVIC_InitStructure);
    
    // Timers - Higher priority than EXTI to ensure timing? 
    // Actually similar priority is fine.
    NVIC_InitStructure.NVIC_IRQChannel = SS1_TIM_IRQ;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = SS2_TIM_IRQ;
    NVIC_Init(&NVIC_InitStructure);
}

// --- Interrupt Handlers ---

static int check_suffix(const char* buf, int len, const char* suffix)
{
    int suffix_len = strlen(suffix);
    if (len < suffix_len) return 0;
    return (strncmp(buf + len - suffix_len, suffix, suffix_len) == 0);
}

void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t ch = USART_ReceiveData(USART1);
        
        if (usart1_rx_idx < CMD_BUF_SIZE - 1)
        {
            usart1_rx_buf[usart1_rx_idx++] = (char)ch;
        }
        else
        {
            memmove(usart1_rx_buf, usart1_rx_buf + 1, CMD_BUF_SIZE - 2);
            usart1_rx_buf[CMD_BUF_SIZE - 2] = (char)ch;
            usart1_rx_idx = CMD_BUF_SIZE - 1;
        }
        
        usart1_rx_buf[usart1_rx_idx] = '\0';
        
        if (check_suffix(usart1_rx_buf, usart1_rx_idx, "reset"))
        {
            GPIO_ResetBits(CTRL_GPIO_PORT, CTRL_GPIO_PIN);
            delay_ms(1500);
            GPIO_SetBits(CTRL_GPIO_PORT, CTRL_GPIO_PIN);
            usart1_rx_idx = 0;
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

// --- Soft Serial Interrupts ---

// Constants for 72MHz clock and 115200 baud
// Bit time = 72000000 / 115200 = 625 cycles
#define BIT_TIME        625
#define HALF_BIT_TIME   312

// Handler for SS1 (PB0) Start Bit
void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(SS1_EXTI_LINE) != RESET)
    {
        // 1. Disable EXTI
        EXTI->IMR &= ~SS1_EXTI_LINE; 
        
        // 2. Setup State
        ss1_state.state = 1; // Start bit detected, wait for middle of bit 0? 
        // Actually, logic:
        // Falling edge is start of Start Bit.
        // We need to sample middle of Start Bit? No, check valid start bit.
        // Or jump straight to middle of D0?
        // Standard: Wait 1.5 bit times to sample D0.
        // Or wait 0.5 bit time to verify Start Bit is still low?
        // Let's do: Wait 1.5 bit times, read D0.
        
        ss1_state.bit_cnt = 0;
        ss1_state.data_byte = 0;
        
        // 3. Start Timer with period = 1.5 bit time
        SS1_TIM->CNT = 0;
        SS1_TIM->ARR = BIT_TIME + HALF_BIT_TIME; 
        SS1_TIM->SR = (uint16_t)~TIM_IT_Update; // Clear pending
        SS1_TIM->CR1 |= TIM_CR1_CEN;
        
        EXTI_ClearITPendingBit(SS1_EXTI_LINE);
    }
}

// Handler for SS1 Timer
void TIM6_IRQHandler(void)
{
    if (TIM_GetITStatus(SS1_TIM, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(SS1_TIM, TIM_IT_Update);
        
        // If this is the first interrupt (1.5 bit time passed), we are at middle of D0
        // Change ARR to 1 bit time for subsequent bits
        if (SS1_TIM->ARR != BIT_TIME)
        {
            SS1_TIM->ARR = BIT_TIME;
        }
        
        if (ss1_state.bit_cnt < 8)
        {
            // Read data bit
            uint8_t bit = GPIO_ReadInputDataBit(SS1_PORT, SS1_PIN);
            if (bit)
            {
                ss1_state.data_byte |= (1 << ss1_state.bit_cnt);
            }
            ss1_state.bit_cnt++;
        }
        else
        {
            // All 8 bits read. We are now at middle of Stop bit (theoretically).
            // Stop timer.
            SS1_TIM->CR1 &= ~TIM_CR1_CEN;
            
            // Store byte
            if (uart_rx_index[4] < UART_RX_BUF_SIZE) {
                uart_rx_buf[4][uart_rx_index[4]++] = ss1_state.data_byte;
            }
            
            // Notify new data (Soft serial sets flag on every byte, processed efficiently)
            new_data_flag[4] = 1;
            
            // Re-enable EXTI for next start bit
            EXTI_ClearITPendingBit(SS1_EXTI_LINE); // Clear potential glitches
            EXTI->IMR |= SS1_EXTI_LINE;
        }
    }
}

// Handler for SS2 (PB1) Start Bit
void EXTI1_IRQHandler(void)
{
    if(EXTI_GetITStatus(SS2_EXTI_LINE) != RESET)
    {
        EXTI->IMR &= ~SS2_EXTI_LINE; 
        
        ss2_state.state = 1; 
        ss2_state.bit_cnt = 0;
        ss2_state.data_byte = 0;
        
        SS2_TIM->CNT = 0;
        SS2_TIM->ARR = BIT_TIME + HALF_BIT_TIME; 
        SS2_TIM->SR = (uint16_t)~TIM_IT_Update;
        SS2_TIM->CR1 |= TIM_CR1_CEN;
        
        EXTI_ClearITPendingBit(SS2_EXTI_LINE);
    }
}

// Handler for SS2 Timer
void TIM7_IRQHandler(void)
{
    if (TIM_GetITStatus(SS2_TIM, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(SS2_TIM, TIM_IT_Update);
        
        if (SS2_TIM->ARR != BIT_TIME)
        {
            SS2_TIM->ARR = BIT_TIME;
        }
        
        if (ss2_state.bit_cnt < 8)
        {
            uint8_t bit = GPIO_ReadInputDataBit(SS2_PORT, SS2_PIN);
            if (bit)
            {
                ss2_state.data_byte |= (1 << ss2_state.bit_cnt);
            }
            ss2_state.bit_cnt++;
        }
        else
        {
            SS2_TIM->CR1 &= ~TIM_CR1_CEN;
            
            if (uart_rx_index[5] < UART_RX_BUF_SIZE) {
                uart_rx_buf[5][uart_rx_index[5]++] = ss2_state.data_byte;
            }
            
            new_data_flag[5] = 1;
            
            EXTI_ClearITPendingBit(SS2_EXTI_LINE);
            EXTI->IMR |= SS2_EXTI_LINE;
        }
    }
}
