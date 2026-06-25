#pragma once
#include <stdint.h>

// 1 kHz millisecond timebase for V3F. Starts a free-running 1 ms periodic
// interrupt (TIM3) and exposes a monotonic millisecond counter via millis().
//
// core_hz: the V3F core/HB1 bus clock that feeds TIM3 (pass SystemCoreClock).
void timebase_init(uint32_t core_hz);

// Monotonic milliseconds since timebase_init(). Wraps at 2^32 ms (~49.7 days).
// The protocol parsers (hurra.c / ferrum.c) consume this via `extern millis`.
uint32_t millis(void);
