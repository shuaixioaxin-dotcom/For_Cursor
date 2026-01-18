#include "app_imu_aggregator.h"

#include <string.h>
#include "app_board.h"
#include "app_ringbuf.h"

// CubeMX工程里应提供这些句柄（按你实际启用的UART命名）
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart7;

typedef enum
{
  PWR_IDLE = 0,
  PWR_OFF,
  PWR_WAIT_OFF,
  PWR_ON,
  PWR_WAIT_ON,
} pwr_state_t;

typedef struct
{
  UART_HandleTypeDef* huart; // 对应UART句柄
  uint8_t* dma_buf;
  uint16_t dma_buf_sz;
  volatile uint16_t dma_last_pos;

  app_ringbuf_t rb;

  app_imu_latest_frame_t latest;

  // LED活动指示（ms倒计时）
  volatile uint16_t led_activity_ms;
} imu_ch_t;

static uint32_t g_tick_ms = 0;
static uint16_t g_agg_elapsed_ms = 0;
static uint16_t g_seq = 0;
static volatile bool g_u1_tx_busy = false;

static volatile bool g_req_power_cycle = false;
static pwr_state_t g_pwr_state = PWR_IDLE;
static uint16_t g_pwr_wait_ms = 0;

// DMA缓冲与环形缓冲背板
static uint8_t g_dma_rx_buf[APP_IMU_CH_COUNT][APP_IMU_DMA_RX_BUF_SZ];
static uint8_t g_rb_backing[APP_IMU_CH_COUNT][APP_IMU_RB_SZ];

static imu_ch_t g_ch[APP_IMU_CH_COUNT];

static uint8_t g_u1_tx_buf[4096];

static inline void led_run_init(void)
{
  BOARD_LED_On(LED1_GPIO_Port, LED1_Pin);
}

static inline void led_imu_on(uint8_t i)
{
  switch (i)
  {
    case 0: BOARD_LED_On(LED2_GPIO_Port, LED2_Pin); break;
    case 1: BOARD_LED_On(LED3_GPIO_Port, LED3_Pin); break;
    case 2: BOARD_LED_On(LED4_GPIO_Port, LED4_Pin); break;
    case 3: BOARD_LED_On(LED5_GPIO_Port, LED5_Pin); break;
    case 4: BOARD_LED_On(LED6_GPIO_Port, LED6_Pin); break;
    case 5: BOARD_LED_On(LED7_GPIO_Port, LED7_Pin); break;
    default: break;
  }
}

static inline void led_imu_off(uint8_t i)
{
  switch (i)
  {
    case 0: BOARD_LED_Off(LED2_GPIO_Port, LED2_Pin); break;
    case 1: BOARD_LED_Off(LED3_GPIO_Port, LED3_Pin); break;
    case 2: BOARD_LED_Off(LED4_GPIO_Port, LED4_Pin); break;
    case 3: BOARD_LED_Off(LED5_GPIO_Port, LED5_Pin); break;
    case 4: BOARD_LED_Off(LED6_GPIO_Port, LED6_Pin); break;
    case 5: BOARD_LED_Off(LED7_GPIO_Port, LED7_Pin); break;
    default: break;
  }
}

static inline void led_tx_on(void)
{
  BOARD_LED_On(LED8_GPIO_Port, LED8_Pin);
}

static inline void led_tx_off(void)
{
  BOARD_LED_Off(LED8_GPIO_Port, LED8_Pin);
}

static uint16_t crc16_ibm(const uint8_t* data, uint16_t len)
{
  // CRC-16/IBM (poly 0xA001, init 0xFFFF)
  uint16_t crc = 0xFFFFu;
  for (uint16_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u) : (uint16_t)(crc >> 1);
  }
  return crc;
}

static inline uint16_t dma_rx_pos(UART_HandleTypeDef* huart, uint16_t dma_buf_sz)
{
  if (huart->hdmarx == NULL)
    return 0;
  uint16_t remain = (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);
  return (uint16_t)(dma_buf_sz - remain);
}

static void start_rx_dma(imu_ch_t* ch)
{
  // 启动DMA循环接收；要求：DMA配置为Circular
  (void)HAL_UART_Receive_DMA(ch->huart, ch->dma_buf, ch->dma_buf_sz);

  // 可选：关闭半传输中断，减少无意义中断
  if (ch->huart->hdmarx)
    __HAL_DMA_DISABLE_IT(ch->huart->hdmarx, DMA_IT_HT);

  // 开启IDLE中断（切帧）
  __HAL_UART_ENABLE_IT(ch->huart, UART_IT_IDLE);

  ch->dma_last_pos = dma_rx_pos(ch->huart, ch->dma_buf_sz);
}

static void ch_init(void)
{
  memset(g_ch, 0, sizeof(g_ch));

  // 固定映射：IMU1..6 -> USART2, USART3, UART4, UART5, USART6, UART7
  g_ch[0].huart = &huart2;
  g_ch[1].huart = &huart3;
  g_ch[2].huart = &huart4;
  g_ch[3].huart = &huart5;
  g_ch[4].huart = &huart6;
  g_ch[5].huart = &huart7;

  for (uint8_t i = 0; i < APP_IMU_CH_COUNT; i++)
  {
    g_ch[i].dma_buf = &g_dma_rx_buf[i][0];
    g_ch[i].dma_buf_sz = APP_IMU_DMA_RX_BUF_SZ;
    app_rb_init(&g_ch[i].rb, &g_rb_backing[i][0], APP_IMU_RB_SZ);
    g_ch[i].dma_last_pos = 0;
    g_ch[i].latest.tick_ms = 0;
    g_ch[i].latest.seq = 0;
    g_ch[i].latest.len = 0;
    g_ch[i].led_activity_ms = 0;
  }
}

static void clear_all_buffers(void)
{
  for (uint8_t i = 0; i < APP_IMU_CH_COUNT; i++)
  {
    app_rb_clear(&g_ch[i].rb);
    g_ch[i].dma_last_pos = dma_rx_pos(g_ch[i].huart, g_ch[i].dma_buf_sz);
    g_ch[i].latest.len = 0;
    g_ch[i].latest.seq = 0;
    g_ch[i].latest.tick_ms = g_tick_ms;
  }
}

static void on_imu_activity(uint8_t i)
{
  g_ch[i].led_activity_ms = 50; // 闪烁50ms
  led_imu_on(i);
}

static void on_u1_tx_activity(void)
{
  led_tx_on();
}

static void build_and_send_agg(void)
{
  if (g_u1_tx_busy)
    return;

  uint16_t p = 0;
  const uint8_t magic[2] = {0xAA, 0x55};
  const uint8_t ver = 0x01;
  const uint8_t n = APP_IMU_CH_COUNT;
  const uint32_t ts = g_tick_ms;
  const uint16_t seq = g_seq++;

  // header
  memcpy(&g_u1_tx_buf[p], magic, 2); p += 2;
  g_u1_tx_buf[p++] = ver;
  g_u1_tx_buf[p++] = (uint8_t)(seq & 0xFF);
  g_u1_tx_buf[p++] = (uint8_t)((seq >> 8) & 0xFF);
  g_u1_tx_buf[p++] = (uint8_t)(ts & 0xFF);
  g_u1_tx_buf[p++] = (uint8_t)((ts >> 8) & 0xFF);
  g_u1_tx_buf[p++] = (uint8_t)((ts >> 16) & 0xFF);
  g_u1_tx_buf[p++] = (uint8_t)((ts >> 24) & 0xFF);
  g_u1_tx_buf[p++] = n;

  // body
  for (uint8_t i = 0; i < APP_IMU_CH_COUNT; i++)
  {
    app_imu_latest_frame_t lf;
    // 这里简单拷贝结构体（主循环与ISR并发时有概率读到中间态；可接受则保持简单）
    // 若你希望更严格一致性，可在调用处临界区保护或改为双缓冲。
    memcpy(&lf, &g_ch[i].latest, sizeof(lf));

    g_u1_tx_buf[p++] = (uint8_t)(i + 1); // CH

    uint16_t len = lf.len;
    if (len > APP_IMU_LATEST_FRAME_MAX)
      len = APP_IMU_LATEST_FRAME_MAX;

    // 防止超出TX缓冲
    if ((uint32_t)p + 2u + (uint32_t)len + 2u > sizeof(g_u1_tx_buf))
      len = 0;

    g_u1_tx_buf[p++] = (uint8_t)(len & 0xFF);
    g_u1_tx_buf[p++] = (uint8_t)((len >> 8) & 0xFF);
    if (len)
    {
      memcpy(&g_u1_tx_buf[p], lf.data, len);
      p += len;
    }
  }

  // CRC
  uint16_t crc = crc16_ibm(g_u1_tx_buf, p);
  g_u1_tx_buf[p++] = (uint8_t)(crc & 0xFF);
  g_u1_tx_buf[p++] = (uint8_t)((crc >> 8) & 0xFF);

  if (HAL_UART_Transmit_DMA(&huart1, g_u1_tx_buf, p) == HAL_OK)
  {
    g_u1_tx_busy = true;
    on_u1_tx_activity();
  }
}

static void handle_idle_for_channel(uint8_t i)
{
  imu_ch_t* ch = &g_ch[i];

  uint16_t pos = dma_rx_pos(ch->huart, ch->dma_buf_sz);
  uint16_t last = ch->dma_last_pos;
  if (pos == last)
    return;

  // 计算新增字节段并推入环形缓冲，同时把“本次段”缓存为latest（截断到LATEST_FRAME_MAX）
  uint8_t latest_tmp[APP_IMU_LATEST_FRAME_MAX];
  uint16_t latest_len = 0;

  if (pos > last)
  {
    uint16_t len = (uint16_t)(pos - last);
    (void)app_rb_push(&ch->rb, &ch->dma_buf[last], len);

    uint16_t take = len;
    if (take > APP_IMU_LATEST_FRAME_MAX)
      take = APP_IMU_LATEST_FRAME_MAX;
    memcpy(latest_tmp, &ch->dma_buf[last], take);
    latest_len = take;
  }
  else
  {
    uint16_t len1 = (uint16_t)(ch->dma_buf_sz - last);
    uint16_t len2 = pos;

    (void)app_rb_push(&ch->rb, &ch->dma_buf[last], len1);
    (void)app_rb_push(&ch->rb, &ch->dma_buf[0], len2);

    // latest：优先拿“wrap后”的连续段拼接（最多截断）
    uint16_t take1 = len1;
    if (take1 > APP_IMU_LATEST_FRAME_MAX)
      take1 = APP_IMU_LATEST_FRAME_MAX;
    memcpy(latest_tmp, &ch->dma_buf[last], take1);
    latest_len = take1;

    if (latest_len < APP_IMU_LATEST_FRAME_MAX)
    {
      uint16_t remain = (uint16_t)(APP_IMU_LATEST_FRAME_MAX - latest_len);
      uint16_t take2 = len2;
      if (take2 > remain)
        take2 = remain;
      memcpy(&latest_tmp[latest_len], &ch->dma_buf[0], take2);
      latest_len = (uint16_t)(latest_len + take2);
    }
  }

  ch->dma_last_pos = pos;

  // 更新最新帧
  ch->latest.tick_ms = g_tick_ms;
  ch->latest.seq++;
  ch->latest.len = latest_len;
  if (latest_len)
    memcpy(ch->latest.data, latest_tmp, latest_len);

  on_imu_activity(i);
}

void APP_IMU_Init(void)
{
  led_run_init();
  led_tx_off();
  for (uint8_t i = 0; i < APP_IMU_CH_COUNT; i++)
    led_imu_off(i);

  BOARD_IMU_PowerOn_All();

  ch_init();

  for (uint8_t i = 0; i < APP_IMU_CH_COUNT; i++)
    start_rx_dma(&g_ch[i]);
}

void APP_IMU_Tick1ms(void)
{
  g_tick_ms++;

  // LED1：运行状态（500ms翻转）
  if ((g_tick_ms % 500u) == 0u)
    BOARD_LED_Toggle(LED1_GPIO_Port, LED1_Pin);

  // IMU活动LED：倒计时归零熄灭
  for (uint8_t i = 0; i < APP_IMU_CH_COUNT; i++)
  {
    uint16_t t = g_ch[i].led_activity_ms;
    if (t)
    {
      t--;
      g_ch[i].led_activity_ms = t;
      if (t == 0)
        led_imu_off(i);
    }
  }

  // TX活动LED：如果一直busy则保持亮；这里在TxCplt里灭
  // reset电源状态机（非阻塞）
  if (g_req_power_cycle && g_pwr_state == PWR_IDLE)
  {
    g_req_power_cycle = false;
    g_pwr_state = PWR_OFF;
  }

  switch (g_pwr_state)
  {
    case PWR_IDLE:
      break;
    case PWR_OFF:
      BOARD_IMU_PowerOff_All();
      g_pwr_wait_ms = APP_IMU_PWR_OFF_MS;
      g_pwr_state = PWR_WAIT_OFF;
      break;
    case PWR_WAIT_OFF:
      if (g_pwr_wait_ms)
        g_pwr_wait_ms--;
      else
        g_pwr_state = PWR_ON;
      break;
    case PWR_ON:
      BOARD_IMU_PowerOn_All();
      g_pwr_wait_ms = APP_IMU_PWR_ON_WAIT_MS;
      g_pwr_state = PWR_WAIT_ON;
      break;
    case PWR_WAIT_ON:
      if (g_pwr_wait_ms)
        g_pwr_wait_ms--;
      else
      {
        clear_all_buffers();
        for (uint8_t i = 0; i < APP_IMU_CH_COUNT; i++)
          start_rx_dma(&g_ch[i]);
        g_pwr_state = PWR_IDLE;
      }
      break;
    default:
      g_pwr_state = PWR_IDLE;
      break;
  }

  // 20ms聚合发送节拍（≥50Hz）
  g_agg_elapsed_ms++;
  if (g_agg_elapsed_ms >= APP_IMU_AGG_PERIOD_MS)
  {
    g_agg_elapsed_ms = 0;
    build_and_send_agg();
  }
}

void APP_IMU_Loop(void)
{
  // 预留：如你后续要解析IMU协议，可在这里从环形缓冲读出并做帧同步/CRC等
}

void APP_IMU_OnUartIrq(UART_HandleTypeDef* huart)
{
  // IDLE切帧：注意HAL_UART_IRQHandler内部不一定会清IDLE，需要我们自己处理
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
  {
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    // 找到对应通道
    for (uint8_t i = 0; i < APP_IMU_CH_COUNT; i++)
    {
      if (g_ch[i].huart == huart)
      {
        handle_idle_for_channel(i);
        break;
      }
    }
  }

  // ORE等错误：尽量清掉，避免卡死
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
    __HAL_UART_CLEAR_OREFLAG(huart);
}

void APP_IMU_OnTxCplt(UART_HandleTypeDef* huart)
{
  if (huart == &huart1)
  {
    g_u1_tx_busy = false;
    led_tx_off();
  }
}

void APP_IMU_RequestPowerCycle(void)
{
  g_req_power_cycle = true;
}

void APP_IMU_GetSnapshot(app_imu_snapshot_t* out)
{
  if (!out)
    return;
  for (uint8_t i = 0; i < APP_IMU_CH_COUNT; i++)
    memcpy(&out->latest[i], &g_ch[i].latest, sizeof(app_imu_latest_frame_t));
}

uint16_t APP_IMU_ReadRaw(uint8_t imu_index_0based, uint8_t* out, uint16_t maxlen)
{
  if (imu_index_0based >= APP_IMU_CH_COUNT)
    return 0;
  return app_rb_pop(&g_ch[imu_index_0based].rb, out, maxlen);
}

