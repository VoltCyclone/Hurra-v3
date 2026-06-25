#pragma once
#include <stdint.h>
#include "usb_host.h"   // USB_SPEED_HIGH / USB_SPEED_FULL / USB_SPEED_LOW

// Decode a USB interrupt-endpoint bInterval byte into a poll period in
// microseconds, honouring the speed-dependent encoding (USB 2.0 §9.6.6):
//
//   * High-Speed: bInterval is an EXPONENT. period = 2^(bInterval-1) * 125us,
//     valid 1..16 (125us .. 4.096s).
//   * Full/Low-Speed: bInterval is a period directly in milliseconds, 1..255.
//
// A raw bInterval is captured verbatim from the endpoint descriptor, so the same
// byte must be decoded against the device's enumerated speed. 0 is treated as 1
// (defensive); HS is clamped to <=16 before the shift so a malformed value can't
// overflow it, FS/LS clamped to <=255.
//
// Pure (no MMIO) so it is unit-testable host-side; the poll-map builders that use
// it (two_board.c) are register-bearing and cannot compile off-target.
static inline uint32_t bInterval_to_us(uint32_t bInterval, uint8_t speed)
{
	if (bInterval == 0) bInterval = 1;
	if (speed == USB_SPEED_HIGH) {
		if (bInterval > 16) bInterval = 16;
		return (1u << (bInterval - 1)) * 125u;
	}
	if (bInterval > 255) bInterval = 255;
	return bInterval * 1000u;
}
