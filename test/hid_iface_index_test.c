// test/hid_iface_index_test.c — host-native tests for HID per-interface wIndex
// validation (SET_/GET_IDLE/PROTOCOL interface bounds check).
//
// The USB device stack is pure USBFS/USBHSD MMIO and cannot run off-target;
// only the interface-index bounds decision is extracted (hid_iface_index.h).

#include <stdio.h>
#include <stdint.h>
#include "hid_iface_index.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    printf("FAIL: %s\n", msg); failures++; } else { \
    printf("ok: %s\n", msg); } } while (0)

int main(void) {
	uint8_t idx = 0xAA;

	// In-range interface 0.
	idx = 0xAA;
	CHECK(hid_iface_index(0x0000, 8, &idx) && idx == 0,
	      "interface 0 in range -> idx 0");

	// In-range top interface (max-1).
	idx = 0xAA;
	CHECK(hid_iface_index(0x0007, 8, &idx) && idx == 7,
	      "interface 7 in range (max 8) -> idx 7");

	// Out-of-range: equal to max -> reject, leave idx untouched.
	idx = 0xAA;
	CHECK(!hid_iface_index(0x0008, 8, &idx) && idx == 0xAA,
	      "interface == max -> rejected, idx untouched");

	// Far out of range.
	idx = 0xAA;
	CHECK(!hid_iface_index(0x00FF, 8, &idx),
	      "interface 0xFF -> rejected");

	// Only the low byte (interface) matters; high byte of wIndex ignored.
	idx = 0xAA;
	CHECK(hid_iface_index(0xAB03, 8, &idx) && idx == 3,
	      "high byte of wIndex ignored -> idx 3");

	if (failures == 0) printf("ALL PASS\n");
	else printf("%d FAILURE(S)\n", failures);
	return failures ? 1 : 0;
}
