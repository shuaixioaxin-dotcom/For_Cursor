/**
 * @file dual_core_encoder.h
 * @brief Multi-encoder communication - Ultra-Fast Dual-Core Batch Processing Mode
 * @note Optimized for maximum frequency using FreeRTOS dual-core processing,
 *       minimized overhead, and aggressive timing optimization.
 */
#pragma once

#include <Arduino.h>

namespace DualCoreEncoder {
void begin();
void loop();
}  // namespace DualCoreEncoder
