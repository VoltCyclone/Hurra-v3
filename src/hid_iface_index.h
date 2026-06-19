#pragma once
#include <stdint.h>
#include <stdbool.h>

// Validate a HID SET_/GET_IDLE/PROTOCOL wIndex (interface number, low byte of
// wIndex) against the per-interface idle/protocol arrays.
//
// Composite HID devices have one idle and one protocol value PER interface; the
// request's wIndex selects which. An out-of-range interface must be STALLed
// rather than indexing past the array. Pulled into a header so the bounds check
// is unit-testable without the USB device stack.
//
// Returns true and writes *out_idx (the array index) when the interface is in
// range; returns false (caller STALLs) otherwise.
static inline bool hid_iface_index(uint16_t w_index, uint8_t max_ifaces,
                                   uint8_t *out_idx)
{
	uint8_t ifc = (uint8_t)(w_index & 0xFF);
	if (ifc >= max_ifaces)
		return false;
	*out_idx = ifc;
	return true;
}
