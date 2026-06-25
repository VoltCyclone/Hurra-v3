// test/usb_desc_interval_test.c — host-native tests for the speed-aware USB
// bInterval -> microseconds decode. The firmware poll-map builders are MMIO and
// cannot run off-target; only this decode is extracted (usb_desc_interval.h) and
// verified here. Real-device HS pacing is gated by the bench test in the plan.

#include <stdio.h>
#include <stdint.h>
#include "usb_host.h"          // USB_SPEED_FULL/LOW/HIGH
#include "usb_desc_interval.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    printf("FAIL: %s\n", msg); failures++; } else { \
    printf("ok: %s\n", msg); } } while (0)

int main(void) {
	// --- Full-Speed: bInterval is a period in milliseconds. ---
	CHECK(bInterval_to_us(1,  USB_SPEED_FULL) == 1000u,   "FS bInterval=1 -> 1000us");
	CHECK(bInterval_to_us(10, USB_SPEED_FULL) == 10000u,  "FS bInterval=10 -> 10000us");
	CHECK(bInterval_to_us(255,USB_SPEED_FULL) == 255000u, "FS bInterval=255 -> 255000us");

	// --- Low-Speed: same ms framing as FS. ---
	CHECK(bInterval_to_us(10, USB_SPEED_LOW)  == 10000u,  "LS bInterval=10 -> 10000us");

	// --- High-Speed: bInterval is an exponent; period = 2^(b-1) * 125us. ---
	CHECK(bInterval_to_us(1, USB_SPEED_HIGH)  == 125u,    "HS bInterval=1 -> 125us (8kHz)");
	CHECK(bInterval_to_us(2, USB_SPEED_HIGH)  == 250u,    "HS bInterval=2 -> 250us (4kHz)");
	CHECK(bInterval_to_us(4, USB_SPEED_HIGH)  == 1000u,   "HS bInterval=4 -> 1000us (1kHz)");
	CHECK(bInterval_to_us(8, USB_SPEED_HIGH)  == 16000u,  "HS bInterval=8 -> 16000us");
	CHECK(bInterval_to_us(16,USB_SPEED_HIGH)  == (1u<<15)*125u, "HS bInterval=16 -> 4.096s");

	// --- Defensive: 0 treated as 1 at both speeds. ---
	CHECK(bInterval_to_us(0, USB_SPEED_FULL)  == 1000u,   "FS bInterval=0 -> clamped to 1ms");
	CHECK(bInterval_to_us(0, USB_SPEED_HIGH)  == 125u,    "HS bInterval=0 -> clamped to 125us");

	// --- Defensive: out-of-range clamps (no shift overflow on HS). ---
	CHECK(bInterval_to_us(17, USB_SPEED_HIGH) == (1u<<15)*125u, "HS bInterval=17 -> clamped to 16");
	CHECK(bInterval_to_us(200,USB_SPEED_HIGH) == (1u<<15)*125u, "HS bInterval=200 -> clamped to 16 (no overflow)");
	CHECK(bInterval_to_us(300,USB_SPEED_FULL) == 255000u, "FS bInterval=300 -> clamped to 255ms");

	if (failures == 0) printf("ALL PASS\n");
	else printf("%d FAILURE(S)\n", failures);
	return failures ? 1 : 0;
}
