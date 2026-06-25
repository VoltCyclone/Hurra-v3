// test/usb_host_time_test.c — host-native tests for the USBHS control-transfer
// budget arithmetic (wrap-safe saturating microsecond deadline).
//
// usb_host.c is pure USBHS MMIO and cannot run off-target; only the deadline
// math is extracted (usb_host_time.h) and verified here. The register-level
// integration is gated by the bench test in the plan (real-device enumeration
// + SET_REPORT no-freeze).

#include <stdio.h>
#include <stdint.h>
#include "usb_host_time.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    printf("FAIL: %s\n", msg); failures++; } else { \
    printf("ok: %s\n", msg); } } while (0)

int main(void) {
	// Fresh budget: full amount remains.
	CHECK(usbhs_remaining_us(1000, 1000, 2000000u) == 2000000u,
	      "no time elapsed -> full budget remains");

	// Part-spent budget.
	CHECK(usbhs_remaining_us(1000 + 500000u, 1000, 2000000u) == 1500000u,
	      "half elapsed -> half remains");

	// Exactly spent -> zero (boundary).
	CHECK(usbhs_remaining_us(1000 + 2000000u, 1000, 2000000u) == 0,
	      "budget exactly spent -> 0");

	// Over-spent -> saturates at zero, never wraps to a huge value.
	CHECK(usbhs_remaining_us(1000 + 2000001u, 1000, 2000000u) == 0,
	      "over budget -> saturates at 0 (no underflow)");

	// Counter wrap: start 1000us before wrap (0xFFFFFC18 == 2^32-1000), now is
	// 500us past zero. True elapsed across the wrap is 1000+500 = 1500us; the
	// unsigned subtract must yield that, not a huge negative-as-unsigned value.
	CHECK(usbhs_remaining_us(500u, 0xFFFFFC18u, 2000000u) == 2000000u - 1500u,
	      "TIM9 wrap -> elapsed computed correctly across wrap");

	// Wrap with budget already spent.
	CHECK(usbhs_remaining_us(0xFFFFFC18u + 2000000u, 0xFFFFFC18u, 2000000u) == 0,
	      "TIM9 wrap, budget spent -> 0");

	if (failures == 0) printf("ALL PASS\n");
	else printf("%d FAILURE(S)\n", failures);
	return failures ? 1 : 0;
}
