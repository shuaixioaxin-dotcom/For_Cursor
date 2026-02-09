/**
 * @file dual_core_encoder.h
 * @brief Multi-encoder communication - Ultra-Fast Dual-Core Batch Processing Mode
 * @note Optimized for maximum frequency using FreeRTOS dual-core processing,
 *       minimized overhead, and aggressive timing optimization.
 */
#pragma once

#include <Arduino.h>

#ifndef DUAL_CORE_ENCODER_WDT_DISABLE
#define DUAL_CORE_ENCODER_WDT_DISABLE 0
#endif

#ifndef DUAL_CORE_ENCODER_WDT_YIELD_CYCLES
#define DUAL_CORE_ENCODER_WDT_YIELD_CYCLES 500
#endif

namespace DualCoreEncoder {
void begin();
void loop();
}  // namespace DualCoreEncoder
