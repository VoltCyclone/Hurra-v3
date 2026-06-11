#pragma once
#include <stdint.h>

// 1 kHz millisecond timebase for the V5F core. Starts a free-running 1 ms
// periodic interrupt (TIM4) and exposes a monotonic millisecond counter via
// millis().
//
// core_hz: API-symmetric with timebase_init(); pass SystemCoreClock. Used only
// as a fallback — the implementation queries RCC_GetClocksFreq().HCLK_Frequency
// for the actual HB1/TIM4 clock so the 1 ms period is exact.
void timebase_v5f_init(uint32_t core_hz);

// Monotonic milliseconds since timebase_v5f_init(). Wraps at 2^32 ms (~49.7
// days). Consumed by the merge module (usb_merge.c) via `extern millis`.
uint32_t millis(void);

// Free-running microsecond counter (TIM9, 32-bit, 1 MHz). Started by
// timebase_v5f_init(). Monotonic, wraps at 2^32 µs (~71.6 min). No interrupt —
// a plain CNT read. Fed to humanize_record_arrival() on each real mouse report
// so the humanization filter can estimate the adaptive feed interval.
uint32_t timebase_v5f_us(void);
