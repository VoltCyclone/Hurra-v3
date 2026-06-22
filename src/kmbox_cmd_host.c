// src/kmbox_cmd_host.c — Board B (host role) injection sinks.
//
// actions.c (shared) calls kmbox_cmd_inject_* / schedule_* via the kmbox.h shim.
// On the single-board V3F these encode ICC records; on Board B's V5F the parser
// runs locally and injection must cross the SPI link to Board A, so these pack the
// same icc_record_t byte layout into the inject FIFO (inject_link), drained as
// TYPE_INJECT frames by the host loop. The byte layout MUST match kmbox_cmd.c so
// Board A's usb_merge_apply_record decodes it unchanged.
#include "inject_link.h"
#include "icc.h"          // ICC_TAG_* values (the wire tags)
#include <string.h>

void kmbox_cmd_inject_mouse(int16_t dx, int16_t dy, uint8_t buttons, int8_t wheel)
{
	uint8_t b[7] = {0};
	b[0]=(uint8_t)dx; b[1]=(uint8_t)((uint16_t)dx>>8);
	b[2]=(uint8_t)dy; b[3]=(uint8_t)((uint16_t)dy>>8);
	b[4]=buttons;     b[5]=(uint8_t)wheel;
	inject_link_push(ICC_TAG_INJECT_MOUSE, b);
}

void kmbox_cmd_inject_keyboard(uint8_t modifier, const uint8_t keys[6])
{
	uint8_t b[7] = {0};
	b[0]=modifier;
	memcpy(&b[1], keys, 6);
	inject_link_push(ICC_TAG_INJECT_KEYBOARD, b);
}

void kmbox_cmd_schedule_click_release(uint8_t button_mask, uint32_t delay_ms)
{
	uint8_t b[7] = {0};
	b[0]=button_mask;
	b[1]=(uint8_t)delay_ms;       b[2]=(uint8_t)(delay_ms>>8);
	b[3]=(uint8_t)(delay_ms>>16); b[4]=(uint8_t)(delay_ms>>24);
	inject_link_push(ICC_TAG_CLICK_RELEASE, b);
}

void kmbox_cmd_schedule_kb_release(uint8_t key, uint32_t delay_ms)
{
	uint8_t b[7] = {0};
	b[0]=key;
	b[1]=(uint8_t)delay_ms;       b[2]=(uint8_t)(delay_ms>>8);
	b[3]=(uint8_t)(delay_ms>>16); b[4]=(uint8_t)(delay_ms>>24);
	inject_link_push(ICC_TAG_KB_RELEASE, b);
}
