/**
 * @file main.c
 * @brief 多路串口(4 IMU)数据接收与解码示例
 *
 * 本示例演示如何同时接收 4 个 IMU 的串口数据，并通过 USART1 连续、连贯地打印解码结果。
 * 设计要点：
 * - **ISR(中断)只负责收字节进环形缓冲**，不在中断里 printf，避免输出被打断/交错。
 * - **主循环统一取出各路缓冲数据并解码、一次性打印**，保证 USART1 输出连贯。
 *
 * @hardware_connections (STM32F103 系列常用引脚)：
 * - USART1 (PA09/PA10): 调试输出(打印到串口助手)
 * - IMU1 -> USART2 (PA02/PA03): 接收 IMU 串口数据
 * - IMU2 -> USART3 (PB10/PB11): 接收 IMU 串口数据
 * - IMU3 -> UART4  (PC10/PC11): 接收 IMU 串口数据
 * - IMU4 -> UART5  (PC12/PD02): 接收 IMU 串口数据
 *
 * @note
 * - 若你的板级资源/引脚复用不同，请按实际硬件修改 GPIO 配置。
 * - 本示例依赖 HiPNUC 提供的解码库：hipnuc_dec.h/hipnuc_dec.c（或等价实现）。
 *
 * @website http://www.hipnuc.com
 * @date 2024-08-07
 * @version 1.1 (4 路 IMU 接收版本)
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "delay.h"
#include "misc.h"
#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"

#include "hipnuc_dec.h"

/* Enable/Disable fixed data array decoding example */
#define ENABLE_FIXED_DATA_EXAMPLE     0

/* Enable/Disable DMA for USART reception (本示例默认使用中断+环形缓冲) */
#define ENABLE_USART_DMA              0

#define USART1_BAUD 115200
#define IMU_BAUD    115200

#define IMU_PORT_COUNT        4
#define UART_RX_BUF_SIZE      2048
#define LOG_STRING_SIZE       1024
/* 4 路合并打印的最大长度（按需可调大） */
#define COMBINED_LOG_SIZE     8192
/* 单次主循环每路最多处理的字节数，避免单路刷屏导致其他路饥饿 */
#define MAX_BYTES_PER_LOOP    256

typedef struct
{
    volatile uint16_t head;
    volatile uint16_t tail;
    uint8_t buf[UART_RX_BUF_SIZE];
    volatile uint32_t overflow_cnt;
} ring_buf_t;

typedef struct
{
    USART_TypeDef *uart;
    ring_buf_t rb;
    hipnuc_raw_t raw;
} imu_port_t;

/* IMU ports: USART2/USART3/UART4/UART5 */
static imu_port_t g_imus[IMU_PORT_COUNT];

/* 打印用缓冲（主循环里串行使用即可） */
static char log_buf[LOG_STRING_SIZE];
static char combined_log[COMBINED_LOG_SIZE];

/* 每路最新一帧的字符串与长度缓存，用于“4 组合并输出” */
static char imu_log[IMU_PORT_COUNT][LOG_STRING_SIZE];
static uint16_t imu_frame_len[IMU_PORT_COUNT];
static uint8_t imu_updated[IMU_PORT_COUNT];

/* Function prototypes */
static void USART_Configuration(uint32_t usart1_baud, uint32_t imu_baud);
static void DMA_Configuration(void);
static void app_init(void);
static void printf_welcome_information(void);
static void process_data(void);

static inline uint16_t rb_next(uint16_t v)
{
    return (uint16_t)((v + 1u) % UART_RX_BUF_SIZE);
}

static inline void rb_push_isr(ring_buf_t *rb, uint8_t ch)
{
    uint16_t next = rb_next(rb->head);
    if (next == rb->tail)
    {
        /* 缓冲满：丢弃新字节并计数（尽量不破坏已在缓冲中的帧） */
        rb->overflow_cnt++;
        return;
    }
    rb->buf[rb->head] = ch;
    rb->head = next;
}

static inline int rb_pop(ring_buf_t *rb, uint8_t *out)
{
    if (rb->tail == rb->head)
        return 0;
    *out = rb->buf[rb->tail];
    rb->tail = rb_next(rb->tail);
    return 1;
}

static inline void imu_uart_irq_handler(USART_TypeDef *uart, ring_buf_t *rb)
{
    /* RXNE: 收到新字节 */
    if (USART_GetITStatus(uart, USART_IT_RXNE) != RESET)
    {
        uint8_t ch = (uint8_t)USART_ReceiveData(uart);
        rb_push_isr(rb, ch);
    }

    /* IDLE: 可用于“帧间隙”提示，本示例不依赖它，但可清除以免反复触发 */
    if (USART_GetITStatus(uart, USART_IT_IDLE) != RESET)
    {
        /*
         * 参考手册：清 IDLE 需要先读 SR 再读 DR
         * (读 DR 也可用 USART_ReceiveData)
         */
        volatile uint32_t tmp;
        tmp = uart->SR;
        tmp = uart->DR;
        (void)tmp;
    }
}

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
        process_data();
    }
}

static void app_init(void)
{
    delay_init();
    delay_ms(1);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /*
     * 重要：4 个 IMU 是自发发送的，可能在上电后立刻产生串口中断。
     * 因此必须先完成 g_imus[] 的指针/缓冲初始化，再开启各串口中断，避免 ISR 访问未初始化对象导致 HardFault。
     */
    memset(g_imus, 0, sizeof(g_imus));
    g_imus[0].uart = USART2;
    g_imus[1].uart = USART3;
    g_imus[2].uart = UART4;
    g_imus[3].uart = UART5;

    for (uint8_t i = 0; i < IMU_PORT_COUNT; i++)
    {
        memset(&g_imus[i].raw, 0, sizeof(hipnuc_raw_t));
        g_imus[i].rb.head = 0;
        g_imus[i].rb.tail = 0;
        g_imus[i].rb.overflow_cnt = 0;
    }

    memset(imu_log, 0, sizeof(imu_log));
    memset(imu_frame_len, 0, sizeof(imu_frame_len));
    memset(imu_updated, 0, sizeof(imu_updated));

    /* USART1: debug output; USART2/3/4/5: IMU reception */
    USART_Configuration(USART1_BAUD, IMU_BAUD);

#if ENABLE_USART_DMA
    DMA_Configuration();
#endif
}

static void printf_welcome_information(void)
{
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);

    printf("HiPNUC IMU data decode example (4 IMUs)\r\n");
    printf("Output: USART1\r\n");
    printf("IMU Rx: USART2/USART3/UART4/UART5\r\n");
#if ENABLE_USART_DMA
    printf("USART reception: DMA\r\n");
#else
    printf("USART reception: UART Interrupt + RingBuffer\r\n");
#endif

    printf("System Clock Frequencies:\r\n");
    printf("SYSCLK: %lu Hz\r\n", (unsigned long)RCC_Clocks.SYSCLK_Frequency);
    printf("HCLK: %lu Hz\r\n", (unsigned long)RCC_Clocks.HCLK_Frequency);
}

static int all_imus_updated(void)
{
    for (uint8_t i = 0; i < IMU_PORT_COUNT; i++)
    {
        if (!imu_updated[i])
            return 0;
    }
    return 1;
}

static void clear_imu_updated(void)
{
    for (uint8_t i = 0; i < IMU_PORT_COUNT; i++)
        imu_updated[i] = 0;
}

static void print_combined_group(void)
{
    size_t off = 0;

    off += (size_t)snprintf(combined_log + off, sizeof(combined_log) - off,
                            "=== IMU GROUP BEGIN ===\r\n");

    for (uint8_t imu = 0; imu < IMU_PORT_COUNT; imu++)
    {
        off += (size_t)snprintf(combined_log + off, sizeof(combined_log) - off,
                                "[IMU%u] frame_len:%u\r\n%s\r\n",
                                (unsigned int)(imu + 1u),
                                (unsigned int)imu_frame_len[imu],
                                imu_log[imu]);
        if (off >= sizeof(combined_log))
        {
            off = sizeof(combined_log) - 1u;
            combined_log[off] = '\0';
            break;
        }
    }

    off += (size_t)snprintf(combined_log + off, sizeof(combined_log) - off,
                            "=== IMU GROUP END ===\r\n\r\n");

    printf("%s", combined_log);
}

static void process_data(void)
{
    for (uint8_t imu = 0; imu < IMU_PORT_COUNT; imu++)
    {
        uint8_t ch;
        uint16_t processed = 0;
        while (processed < MAX_BYTES_PER_LOOP && rb_pop(&g_imus[imu].rb, &ch))
        {
            processed++;
            if (hipnuc_input(&g_imus[imu].raw, ch))
            {
                /* Convert result to string */
                hipnuc_dump_packet(&g_imus[imu].raw, log_buf, sizeof(log_buf));

                /* 缓存“最新一帧”到对应 IMU 槽位 */
                imu_frame_len[imu] = (uint16_t)g_imus[imu].raw.len;
                (void)snprintf(imu_log[imu], sizeof(imu_log[imu]), "%s", log_buf);
                imu_updated[imu] = 1;

                /* 当 4 路都更新后，再合并输出一组 */
                if (all_imus_updated())
                {
                    print_combined_group();
                    clear_imu_updated();
                }
            }
        }
    }
}

static void USART_Configuration(uint32_t usart1_baud, uint32_t imu_baud)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Clocks: USART + GPIO */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_USART3 | RCC_APB1Periph_UART4 | RCC_APB1Periph_UART5, ENABLE);

    /* USART1 (PA9 TX, PA10 RX) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART2 (PA2 TX, PA3 RX) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART3 (PB10 TX, PB11 RX) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* UART4 (PC10 TX, PC11 RX) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* UART5 (PC12 TX, PD2 RX) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* USART common configuration */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    /* USART1 */
    USART_InitStructure.USART_BaudRate = usart1_baud;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);

    /* IMU UARTs: USART2/USART3/UART4/UART5 */
    USART_InitStructure.USART_BaudRate = imu_baud;
    USART_Init(USART2, &USART_InitStructure);
    USART_Init(USART3, &USART_InitStructure);
    USART_Init(UART4, &USART_InitStructure);
    USART_Init(UART5, &USART_InitStructure);

    USART_Cmd(USART2, ENABLE);
    USART_Cmd(USART3, ENABLE);
    USART_Cmd(UART4, ENABLE);
    USART_Cmd(UART5, ENABLE);

    /* Enable RXNE + IDLE interrupt for all IMU UARTs (打印不在中断里做) */
#if !ENABLE_USART_DMA
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
#endif
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);
    USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);

    /* NVIC configuration */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_Init(&NVIC_InitStructure);
}

static void DMA_Configuration(void)
{
    /*
     * 预留：若需要 DMA 环形接收，可为 USART2/3/UART4/UART5 分别配置 DMA 通道。
     * 由于不同芯片/通道映射差异较大，本示例默认使用中断+环形缓冲，保证通用与易读。
     */
}

void USART2_IRQHandler(void)
{
    imu_uart_irq_handler(USART2, &g_imus[0].rb);
}

void USART3_IRQHandler(void)
{
    imu_uart_irq_handler(USART3, &g_imus[1].rb);
}

void UART4_IRQHandler(void)
{
    imu_uart_irq_handler(UART4, &g_imus[2].rb);
}

void UART5_IRQHandler(void)
{
    imu_uart_irq_handler(UART5, &g_imus[3].rb);
}

