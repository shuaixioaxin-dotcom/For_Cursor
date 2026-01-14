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
#include "stm32f10x_tim.h"
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
/* 严格同步窗口：4 路时间戳 max-min 必须 <= 该窗口才允许输出 */
#define SYNC_WINDOW_US        3000u
/* 若一直凑不齐 4 路，超过该超时则丢弃并重新对齐 */
#define GROUP_TIMEOUT_US      50000u
/* 没有成组输出时，周期性打印状态，避免“看起来没数据” */
#define STATUS_PERIOD_US      1000000u
/* 每路缓存帧队列深度：提升匹配成功率（RAM 允许可加大） */
#define FRAME_QUEUE_DEPTH     4u
/* 允许丢包：成组输出时，允许缺失的 IMU 数量（例如 2 表示至少 2 路有效即可输出） */
#define MIN_IMU_PER_GROUP     2u

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

typedef struct
{
    hipnuc_raw_t raw;
    uint16_t len;
    uint32_t ts_us;
} imu_frame_t;

typedef struct
{
    imu_frame_t q[FRAME_QUEUE_DEPTH];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} frame_queue_t;

/* 每路帧队列：用于严格窗口内对齐匹配 */
static frame_queue_t g_fq[IMU_PORT_COUNT];

static uint32_t g_group_seq = 0;
static uint32_t g_last_status_us = 0;
static uint32_t g_frame_ok_cnt[IMU_PORT_COUNT];
static uint32_t g_sync_drop_cnt = 0;
static uint32_t g_sync_timeout_cnt = 0;
static uint32_t g_last_timeout_log_us = 0;
static uint32_t g_soft_time = 0;
static uint32_t g_loop_cnt = 0;
static uint8_t g_timebase_ok = 0;
static uint32_t g_group_ref_us = 0;
static volatile uint32_t g_rx_byte_cnt[IMU_PORT_COUNT];

/* -------- 时间戳：使用 TIM2 1MHz 自由运行计数（不使用 DWT） -------- */
static void tim2_timebase_init_1mhz(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);

    /*
     * STM32F1 定时器时钟规则：
     * - TIM2 在 APB1 上
     * - 若 PCLK1 分频 = 1，则 TIMxCLK = PCLK1
     * - 否则 TIMxCLK = 2 * PCLK1
     */
    uint32_t pclk1 = clocks.PCLK1_Frequency;
    uint32_t tim2clk = pclk1;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
        tim2clk = pclk1 * 2u;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef tb;
    TIM_TimeBaseStructInit(&tb);
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    tb.TIM_Period = 0xFFFFFFFFu;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_Prescaler = (uint16_t)((tim2clk / 1000000u) - 1u); /* 1MHz -> 1us/计数 */
    TIM_TimeBaseInit(TIM2, &tb);

    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);

    /* 检测计数器是否在走，避免 timebase 失效导致逻辑“沉默” */
    {
        uint32_t a = TIM_GetCounter(TIM2);
        for (volatile uint32_t i = 0; i < 50000u; i++)
        {
            __NOP();
        }
        uint32_t b = TIM_GetCounter(TIM2);
        g_timebase_ok = (a != b) ? 1u : 0u;
    }
}

static uint32_t micros_now(void)
{
    if (g_timebase_ok)
        return (uint32_t)TIM_GetCounter(TIM2);
    /*
     * 退化：无可靠 us 时间源时，用软计数占位，仅用于限频/状态输出不“卡死”。
     * 注意：这不是精确时间。
     */
    return ++g_soft_time;
}

/* Function prototypes */
static void USART_Configuration(uint32_t usart1_baud, uint32_t imu_baud);
static void DMA_Configuration(void);
static void app_init(void);
static void printf_welcome_information(void);
static void process_data(void);
static void try_emit_groups(void);

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
        /*
         * 统计收到的原始字节数：用于判断是否“物理层收到了数据但解码失败”
         * 这里用 uart 指针映射到 index（与 app_init 的分配一致）。
         */
        if (uart == USART2) g_rx_byte_cnt[0]++;
        else if (uart == USART3) g_rx_byte_cnt[1]++;
        else if (uart == UART4) g_rx_byte_cnt[2]++;
        else if (uart == UART5) g_rx_byte_cnt[3]++;
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

    SystemCoreClockUpdate();
    tim2_timebase_init_1mhz();

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

    memset(g_fq, 0, sizeof(g_fq));
    g_group_seq = 0;
    g_last_status_us = 0;
    memset(g_frame_ok_cnt, 0, sizeof(g_frame_ok_cnt));
    g_sync_drop_cnt = 0;
    g_sync_timeout_cnt = 0;
    g_last_timeout_log_us = 0;
    g_soft_time = 0;
    g_loop_cnt = 0;
    g_group_ref_us = 0;
    for (uint8_t i = 0; i < IMU_PORT_COUNT; i++)
        g_rx_byte_cnt[i] = 0;

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
    printf("Sync window: %lu us, timeout: %lu us\r\n",
           (unsigned long)SYNC_WINDOW_US,
           (unsigned long)GROUP_TIMEOUT_US);
    printf("Frame queue depth: %lu\r\n", (unsigned long)FRAME_QUEUE_DEPTH);
    printf("Timebase: TIM2 %s\r\n", g_timebase_ok ? "OK" : "FAIL (fallback)");
}

static inline uint8_t fq_next(uint8_t v)
{
    return (uint8_t)((v + 1u) % FRAME_QUEUE_DEPTH);
}

static void fq_push(uint8_t imu, const hipnuc_raw_t *raw, uint16_t len, uint32_t ts_us)
{
    frame_queue_t *fq = &g_fq[imu];

    /* 满了则丢弃最旧的，保证能继续前进 */
    if (fq->count >= FRAME_QUEUE_DEPTH)
    {
        fq->head = fq_next(fq->head);
        fq->count--;
        g_sync_drop_cnt++;
    }

    imu_frame_t *slot = &fq->q[fq->tail];
    memcpy(&slot->raw, raw, sizeof(hipnuc_raw_t));
    slot->len = len;
    slot->ts_us = ts_us;

    fq->tail = fq_next(fq->tail);
    fq->count++;
}

static int fq_peek(uint8_t imu, imu_frame_t *out)
{
    frame_queue_t *fq = &g_fq[imu];
    if (fq->count == 0u)
        return 0;
    *out = fq->q[fq->head];
    return 1;
}

static int fq_pop(uint8_t imu, imu_frame_t *out)
{
    frame_queue_t *fq = &g_fq[imu];
    if (fq->count == 0u)
        return 0;
    *out = fq->q[fq->head];
    fq->head = fq_next(fq->head);
    fq->count--;
    return 1;
}

static int all_queues_nonempty(void)
{
    for (uint8_t i = 0; i < IMU_PORT_COUNT; i++)
    {
        if (g_fq[i].count == 0u)
            return 0;
    }
    return 1;
}

static void print_combined_group(const imu_frame_t frames[IMU_PORT_COUNT], const uint8_t present[IMU_PORT_COUNT])
{
    size_t off = 0;

    uint32_t min_ts = 0xFFFFFFFFu, max_ts = 0;
    for (uint8_t i = 0; i < IMU_PORT_COUNT; i++)
    {
        if (!present[i])
            continue;
        if (frames[i].ts_us < min_ts) min_ts = frames[i].ts_us;
        if (frames[i].ts_us > max_ts) max_ts = frames[i].ts_us;
    }
    if (min_ts == 0xFFFFFFFFu)
    {
        /* 全缺失不应发生 */
        min_ts = 0;
        max_ts = 0;
    }

    off += (size_t)snprintf(combined_log + off, sizeof(combined_log) - off,
                            "=== IMU GROUP #%lu dt=%luus ===\r\n",
                            (unsigned long)g_group_seq,
                            (unsigned long)(max_ts - min_ts));

    for (uint8_t imu = 0; imu < IMU_PORT_COUNT; imu++)
    {
        if (present[imu])
        {
            /* 按组打印时再生成字符串 */
            hipnuc_dump_packet((hipnuc_raw_t *)&frames[imu].raw, log_buf, sizeof(log_buf));
            off += (size_t)snprintf(combined_log + off, sizeof(combined_log) - off,
                                    "[IMU%u] t=%luus frame_len:%u\r\n%s\r\n",
                                    (unsigned int)(imu + 1u),
                                    (unsigned long)frames[imu].ts_us,
                                    (unsigned int)frames[imu].len,
                                    log_buf);
        }
        else
        {
            off += (size_t)snprintf(combined_log + off, sizeof(combined_log) - off,
                                    "[IMU%u] MISSING\r\n",
                                    (unsigned int)(imu + 1u));
        }
        if (off >= sizeof(combined_log))
        {
            off = sizeof(combined_log) - 1u;
            combined_log[off] = '\0';
            break;
        }
    }

    off += (size_t)snprintf(combined_log + off, sizeof(combined_log) - off, "\r\n");

    printf("%s", combined_log);
}

static void drop_oldest_head_frame(void)
{
    uint8_t oldest = 0;
    uint32_t min_ts = 0xFFFFFFFFu;
    for (uint8_t i = 0; i < IMU_PORT_COUNT; i++)
    {
        if (g_fq[i].count == 0u)
            continue;
        uint32_t ts = g_fq[i].q[g_fq[i].head].ts_us;
        if (ts < min_ts)
        {
            min_ts = ts;
            oldest = i;
        }
    }

    imu_frame_t dummy;
    (void)fq_pop(oldest, &dummy);
    g_sync_drop_cnt++;
}

static void try_emit_groups(void)
{
    /*
     * 严格同步策略：
     * - 每路用小 FIFO 缓存多帧，提升可匹配概率（避免某一路连发覆盖掉可匹配帧）。
     * - 当 4 路队列头部帧时间戳 max-min <= SYNC_WINDOW_US，则输出一组并各 pop 1 帧。
     * - 否则 pop 掉最旧的队列头帧继续追齐。
     * - 若最旧帧等待超过 GROUP_TIMEOUT_US，则丢弃最旧帧并记录 timeout（防止长期卡住）。
     */

    uint32_t now = micros_now();

    /*
     * 允许丢包的严格窗口分组：
     * - 以“当前最旧可用帧”的时间戳作为 ref；
     * - 在每路队列中找一个落在 [ref, ref+SYNC_WINDOW_US] 的帧作为本组成员（找不到则 MISSING）；
     * - 若本组有效 IMU 数 >= MIN_IMU_PER_GROUP，则输出一组；
     * - 若 ref 等待超过 GROUP_TIMEOUT_US 仍凑不出最低数量，则丢弃 ref（最旧帧）推进；
     */

    /* 计算当前 ref（全局最旧队首） */
    uint32_t ref = 0xFFFFFFFFu;
    uint8_t any = 0;
    for (uint8_t i = 0; i < IMU_PORT_COUNT; i++)
    {
        if (g_fq[i].count == 0u)
            continue;
        any = 1;
        uint32_t ts = g_fq[i].q[g_fq[i].head].ts_us;
        if (ts < ref)
            ref = ts;
    }
    if (!any)
        return;

    if (g_group_ref_us == 0u)
        g_group_ref_us = ref;

    /* ref 等太久则推进（并限频提示） */
    if ((uint32_t)(now - g_group_ref_us) > GROUP_TIMEOUT_US)
    {
        drop_oldest_head_frame();
        g_group_ref_us = 0u;
        g_sync_timeout_cnt++;
        if ((uint32_t)(now - g_last_timeout_log_us) >= STATUS_PERIOD_US)
        {
            g_last_timeout_log_us = now;
            printf("[SYNC] timeout drop oldest frame (allow loss)\r\n");
        }
        return;
    }

    /* 尝试围绕 ref 组一帧（允许缺失） */
    imu_frame_t frames[IMU_PORT_COUNT];
    uint8_t present[IMU_PORT_COUNT] = {0};
    uint8_t present_cnt = 0;

    for (uint8_t imu = 0; imu < IMU_PORT_COUNT; imu++)
    {
        frame_queue_t *fq = &g_fq[imu];
        if (fq->count == 0u)
            continue;

        /* 在队列里找第一个落在窗口内的帧 */
        uint8_t idx = fq->head;
        for (uint8_t k = 0; k < fq->count; k++)
        {
            imu_frame_t *cand = &fq->q[idx];
            if (cand->ts_us >= ref && (uint32_t)(cand->ts_us - ref) <= SYNC_WINDOW_US)
            {
                /* pop 掉窗口前的旧帧（包括选中的） */
                for (uint8_t popn = 0; popn <= k; popn++)
                {
                    imu_frame_t tmp;
                    (void)fq_pop(imu, &tmp);
                    if (popn == k)
                        frames[imu] = tmp;
                }
                present[imu] = 1;
                present_cnt++;
                break;
            }
            idx = fq_next(idx);
        }
    }

    if (present_cnt >= MIN_IMU_PER_GROUP)
    {
        print_combined_group(frames, present);
        g_group_seq++;
        g_group_ref_us = 0u;
    }
}

static void print_status_if_needed(void)
{
    uint32_t now = micros_now();
    g_loop_cnt++;
    if (g_timebase_ok)
    {
        if ((uint32_t)(now - g_last_status_us) < STATUS_PERIOD_US)
            return;
        g_last_status_us = now;
    }
    else
    {
        /* 无可靠时间基准时，每 N 次循环打印一次 */
        if ((g_loop_cnt % 2000000u) != 0u)
            return;
    }

    uint8_t miss_mask = 0;
    for (uint8_t i = 0; i < IMU_PORT_COUNT; i++)
    {
        if (g_fq[i].count == 0u)
            miss_mask |= (uint8_t)(1u << i);
    }

    printf("[STAT] rx_bytes:%lu %lu %lu %lu | ok_cnt:%lu %lu %lu %lu | q_cnt:%u %u %u %u | miss_mask:0x%02X | ovf:%lu %lu %lu %lu | drop:%lu timeout:%lu\r\n",
           (unsigned long)g_rx_byte_cnt[0],
           (unsigned long)g_rx_byte_cnt[1],
           (unsigned long)g_rx_byte_cnt[2],
           (unsigned long)g_rx_byte_cnt[3],
           (unsigned long)g_frame_ok_cnt[0],
           (unsigned long)g_frame_ok_cnt[1],
           (unsigned long)g_frame_ok_cnt[2],
           (unsigned long)g_frame_ok_cnt[3],
           (unsigned int)g_fq[0].count,
           (unsigned int)g_fq[1].count,
           (unsigned int)g_fq[2].count,
           (unsigned int)g_fq[3].count,
           (unsigned int)miss_mask,
           (unsigned long)g_imus[0].rb.overflow_cnt,
           (unsigned long)g_imus[1].rb.overflow_cnt,
           (unsigned long)g_imus[2].rb.overflow_cnt,
           (unsigned long)g_imus[3].rb.overflow_cnt,
           (unsigned long)g_sync_drop_cnt,
           (unsigned long)g_sync_timeout_cnt);
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
                g_frame_ok_cnt[imu]++;

                /* 入队：保存原始解析结果（严格同步用队列对齐） */
                fq_push(imu, &g_imus[imu].raw, (uint16_t)g_imus[imu].raw.len, micros_now());

                /* 有新帧就尝试匹配输出 */
                try_emit_groups();
            }
        }
    }

    /* 即使没有成组输出，也会周期性打印状态便于定位 */
    print_status_if_needed();
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

