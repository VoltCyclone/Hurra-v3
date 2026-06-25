#pragma once
#include <stdint.h>
#include <stdbool.h>

// src/hid_layout.h — shared HID report-descriptor layout decode.
//
// Factored out of usb_merge.c so both the device-side merge (Board A) and the
// host-side physical-report feed for catch_xy (Board B, two_board.c) parse the
// mouse report layout identically. Pure functions over caller-owned state — no
// globals — so the move is behavior-preserving (usb_merge_test is the gate).

typedef struct {
	uint16_t x_bit;
	uint16_t y_bit;
	uint16_t wheel_bit;     // 0xFFFF = none
	uint8_t  x_size;
	uint8_t  y_size;
	uint8_t  wheel_size;
	uint8_t  report_id;
	uint8_t  y_report_id;
	uint8_t  wheel_report_id;
	uint8_t  data_off;
	bool     valid;
	int16_t  x_max;
	int16_t  y_max;
	int16_t  w_max;
	bool     fast_path;
	uint8_t  x_byte;
	uint8_t  y_byte;
	uint8_t  w_byte;        // 0xFF = none
	bool     x_is16;
	bool     y_is16;
	bool     w_is16;
} mouse_layout_t;

// Parse a HID report descriptor into *out (zeroes it first). Extracts the X/Y/
// wheel field positions, sizes, report IDs, per-axis clamp maxima, and a
// byte-aligned fast-path flag. Safe on malformed/short descriptors (bounds-checked).
void hid_layout_parse_mouse(mouse_layout_t *out, const uint8_t *rd, uint16_t rdlen);

// Read a signed bit-field from a HID report buffer. bit_off is relative to the
// data area; data_off (report-ID prefix bytes) is added internally. Returns 0 on
// any out-of-bounds access. Sign-extends from bit_size.
int32_t hid_layout_read_field(const uint8_t *buf, uint8_t buf_len,
                              uint16_t bit_off, uint8_t bit_size, uint8_t data_off);
