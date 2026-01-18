#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/**
 * 单生产者/单消费者环形缓冲（ISR写入，主循环读取的典型用法）。
 * - 写入端：IDLE中断中把DMA新增字节段 push 进来
 * - 读取端：主循环/任务中 pop 进行协议解析
 *
 * 注意：这里的实现尽量轻量；并发安全依赖“写指针只在ISR更新、读指针只在主循环更新”。
 */

typedef struct
{
  uint8_t* buf;
  uint16_t size;   // 必须 <= 65535
  volatile uint16_t w; // 写指针
  volatile uint16_t r; // 读指针
} app_ringbuf_t;

static inline void app_rb_init(app_ringbuf_t* rb, uint8_t* backing, uint16_t size)
{
  rb->buf = backing;
  rb->size = size;
  rb->w = 0;
  rb->r = 0;
}

static inline void app_rb_clear(app_ringbuf_t* rb)
{
  rb->w = 0;
  rb->r = 0;
}

static inline uint16_t app_rb_used(const app_ringbuf_t* rb)
{
  uint16_t w = rb->w;
  uint16_t r = rb->r;
  return (w >= r) ? (uint16_t)(w - r) : (uint16_t)(rb->size - (r - w));
}

static inline uint16_t app_rb_free(const app_ringbuf_t* rb)
{
  // 留一个字节用于区分满/空
  return (uint16_t)(rb->size - 1u - app_rb_used(rb));
}

static inline uint16_t app_rb_push(app_ringbuf_t* rb, const uint8_t* data, uint16_t len)
{
  uint16_t free = app_rb_free(rb);
  if (len > free)
    len = free;

  uint16_t w = rb->w;
  uint16_t first = (uint16_t)(rb->size - w);
  if (first > len)
    first = len;

  if (first)
    memcpy(&rb->buf[w], data, first);
  if (len > first)
    memcpy(&rb->buf[0], &data[first], (size_t)(len - first));

  rb->w = (uint16_t)((w + len) % rb->size);
  return len;
}

static inline uint16_t app_rb_pop(app_ringbuf_t* rb, uint8_t* out, uint16_t maxlen)
{
  uint16_t used = app_rb_used(rb);
  if (maxlen > used)
    maxlen = used;

  uint16_t r = rb->r;
  uint16_t first = (uint16_t)(rb->size - r);
  if (first > maxlen)
    first = maxlen;

  if (first)
    memcpy(out, &rb->buf[r], first);
  if (maxlen > first)
    memcpy(&out[0], &rb->buf[0], (size_t)(maxlen - first));

  rb->r = (uint16_t)((r + maxlen) % rb->size);
  return maxlen;
}

