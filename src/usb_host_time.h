#pragma once
#include <stdint.h>

// Saturating wall-clock budget arithmetic for USBHS control transfers.
//
// Computes the microseconds left in a transfer budget given the current and
// start timestamps. The subtraction is unsigned so it is correct across a
// single TIM9 counter wrap; the result saturates at zero once the budget is
// spent (callers treat a zero budget as "no NAK retry — return immediately").
//
// Pulled into a header so the arithmetic is unit-testable host-side without the
// USBHS register file (usb_host.c itself is pure MMIO and cannot be compiled
// off-target).
static inline uint32_t usbhs_remaining_us(uint32_t now_us, uint32_t start_us,
                                          uint32_t budget_us)
{
	uint32_t elapsed = now_us - start_us;        /* wrap-safe unsigned subtract */
	return (elapsed >= budget_us) ? 0u : (budget_us - elapsed);
}
