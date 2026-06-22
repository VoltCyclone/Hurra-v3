// src/hid_layout.c — shared HID report-descriptor layout decode.
//
// Factored verbatim out of usb_merge.c (parse_mouse_layout / read_report_field)
// so the host-side catch_xy physical-report feed (two_board.c) and the device-side
// merge (usb_merge.c) share one parser. Behavior-preserving move: usb_merge_test
// is the regression gate.
#include "hid_layout.h"
#include <string.h>

void hid_layout_parse_mouse(mouse_layout_t *ml, const uint8_t *rd, uint16_t rdlen)
{
	memset(ml, 0, sizeof(*ml));
	ml->wheel_bit = 0xFFFF;

	uint16_t usage_page = 0;
	uint8_t  usages[16];
	uint8_t  num_usages = 0;
	uint16_t usage_min = 0, usage_max = 0;
	uint8_t  report_size = 0;
	uint8_t  report_count = 0;
	uint8_t  current_rid = 0;
	uint16_t bit_pos = 0;

	uint16_t i = 0;
	while (i < rdlen) {
		uint8_t b = rd[i];
		if (b == 0xFE) { // long item — skip
			if (i + 2 < rdlen) i += 3 + rd[i + 1];
			else break;
			continue;
		}

		uint8_t sz = b & 0x03;
		if (sz == 3) sz = 4;
		if (i + 1 + sz > rdlen) break;

		// Read unsigned data
		uint32_t val = 0;
		if (sz >= 1) val = rd[i + 1];
		if (sz >= 2) val |= (uint32_t)rd[i + 2] << 8;
		if (sz >= 4) val |= (uint32_t)rd[i + 3] << 16 | (uint32_t)rd[i + 4] << 24;

		switch (b & 0xFC) {
		case 0x04: usage_page = (uint16_t)val; break;   // Usage Page
		case 0x74: report_size = (uint8_t)val; break;    // Report Size
		case 0x94: report_count = (uint8_t)val; break;   // Report Count
		case 0x84:                                        // Report ID
			current_rid = (uint8_t)val;
			bit_pos = 0;
			break;

		case 0x08: // Usage
			if (num_usages < 16) usages[num_usages++] = (uint8_t)val;
			break;
		case 0x18: usage_min = (uint16_t)val; break;     // Usage Minimum
		case 0x28: usage_max = (uint16_t)val; break;     // Usage Maximum

		case 0x80: { // Input
			if (num_usages == 0 && usage_max >= usage_min) {
				for (uint16_t u = usage_min; u <= usage_max && num_usages < 16; u++)
					usages[num_usages++] = (uint8_t)u;
			}

			for (uint8_t f = 0; f < report_count; f++) {
				uint8_t u = (f < num_usages) ? usages[f] :
				            (num_usages > 0 ? usages[num_usages - 1] : 0);

				if (usage_page == 0x01) { // Generic Desktop
					if (u == 0x30) { // X
						ml->x_bit = bit_pos;
						ml->x_size = report_size;
						ml->report_id = current_rid;
					} else if (u == 0x31) { // Y
						ml->y_bit = bit_pos;
						ml->y_size = report_size;
						ml->y_report_id = current_rid;
					} else if (u == 0x38) { // Wheel
						ml->wheel_bit = bit_pos;
						ml->wheel_size = report_size;
						ml->wheel_report_id = current_rid;
					}
				}
				bit_pos += report_size;
			}
			// Clear local state after Main item
			num_usages = 0;
			usage_min = 0;
			usage_max = 0;
			break;
		}
		case 0xA0: // Collection
			num_usages = 0;
			usage_min = 0;
			usage_max = 0;
			break;
		case 0xC0: // End Collection
			num_usages = 0;
			break;
		}

		i += 1 + sz;
	}

	ml->data_off = ml->report_id ? 1 : 0;
	ml->valid = (ml->x_size > 0 && ml->y_size > 0);
	ml->x_max = ml->x_size > 0 ? (int16_t)((1 << (ml->x_size - 1)) - 1) : 0;
	ml->y_max = ml->y_size > 0 ? (int16_t)((1 << (ml->y_size - 1)) - 1) : 0;
	ml->w_max = ml->wheel_size > 0 ? (int16_t)((1 << (ml->wheel_size - 1)) - 1) : 0;

	ml->fast_path = false;
	ml->w_byte = 0xFF;

	if (ml->valid &&
	    (ml->x_bit & 7) == 0 &&
	    (ml->y_bit & 7) == 0 &&
	    (ml->x_size == 8 || ml->x_size == 16) &&
	    (ml->y_size == 8 || ml->y_size == 16) &&
	    ml->report_id == ml->y_report_id) {

		ml->x_byte = (uint8_t)(ml->x_bit / 8) + ml->data_off;
		ml->y_byte = (uint8_t)(ml->y_bit / 8) + ml->data_off;
		ml->x_is16 = (ml->x_size == 16);
		ml->y_is16 = (ml->y_size == 16);

		if (ml->wheel_bit != 0xFFFF &&
		    (ml->wheel_bit & 7) == 0 &&
		    (ml->wheel_size == 8 || ml->wheel_size == 16) &&
		    ml->wheel_report_id == ml->report_id) {
			ml->w_byte = (uint8_t)(ml->wheel_bit / 8) + ml->data_off;
			ml->w_is16 = (ml->wheel_size == 16);
		}
		ml->fast_path = true;
	}
}

int32_t hid_layout_read_field(const uint8_t *buf, uint8_t buf_len,
                              uint16_t bit_off, uint8_t bit_size, uint8_t data_off)
{
	uint16_t abs_bit = bit_off + (uint16_t)data_off * 8;
	uint16_t byte_idx = abs_bit >> 3;
	uint8_t  bit_idx = abs_bit & 7;

	if (__builtin_expect(bit_idx == 0, 1)) {
		if (bit_size == 16) {
			if (byte_idx + 2 > buf_len) return 0;
			return (int16_t)(buf[byte_idx] | ((uint16_t)buf[byte_idx + 1] << 8));
		}
		if (bit_size == 8) {
			if (byte_idx + 1 > buf_len) return 0;
			return (int8_t)buf[byte_idx];
		}
	}

	uint32_t raw = 0;
	uint8_t bytes_needed = (bit_idx + bit_size + 7) >> 3;
	if (byte_idx + bytes_needed > buf_len) return 0;
	for (uint8_t b = 0; b < bytes_needed; b++)
		raw |= (uint32_t)buf[byte_idx + b] << (b * 8);
	raw = (raw >> bit_idx) & ((1u << bit_size) - 1);
	if (raw & (1u << (bit_size - 1)))
		raw |= ~((1u << bit_size) - 1); // sign extend
	return (int32_t)raw;
}
