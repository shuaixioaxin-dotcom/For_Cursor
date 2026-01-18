#include "app_console.h"

#include <string.h>
#include "app_imu_aggregator.h"

// 简单滑动窗口匹配 "reset"
static uint8_t g_win[5];
static uint8_t g_filled = 0;

void APP_Console_Init(void)
{
  memset(g_win, 0, sizeof(g_win));
  g_filled = 0;
}

void APP_Console_OnRxByte(uint8_t b)
{
  // 过滤常见的回车换行（即使你说不带，也避免误触发）
  if (b == '\r' || b == '\n')
    return;

  if (g_filled < sizeof(g_win))
  {
    g_win[g_filled++] = b;
  }
  else
  {
    memmove(&g_win[0], &g_win[1], sizeof(g_win) - 1u);
    g_win[sizeof(g_win) - 1u] = b;
  }

  if (g_filled == sizeof(g_win))
  {
    if (g_win[0] == 'r' && g_win[1] == 'e' && g_win[2] == 's' && g_win[3] == 'e' && g_win[4] == 't')
    {
      APP_IMU_RequestPowerCycle();
      // 清空窗口，避免连续触发
      memset(g_win, 0, sizeof(g_win));
      g_filled = 0;
    }
  }
}

