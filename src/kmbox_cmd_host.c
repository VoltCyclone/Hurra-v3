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

// ── Link-stats / baud stubs (host role) ──────────────────────────────────────
// On Board B the command transport is the CDC virtual COM port, not USART1, so
// uart.c (the V3F driver) is NOT linked into the host V5F image. The protocol
// parser (hurra.c/ferrum.c, now linked here for Board B) still references the
// kmbox link-stats/baud shim (kmbox_cmd_set_baud + uart_current_baud / _tx_room /
// _overrun). Provide minimal stubs so the parser links and its STATS/telemetry
// replies stay sane over CDC:
//   - set_baud: no-op (the CDC line rate is virtual; baud is meaningless).
//   - current_baud: report the build-time CMD_BAUD so a STATS reply isn't zero.
//   - tx_room: a generous constant so TinyFrame telemetry (routed to the CDC TX
//     ring via proto_set_tx) is never gated off by a false "ring full"; the CDC
//     driver applies real backpressure in cdc_fs_tx_write.
//   - overrun: 0 (no UART RX path on this transport).
void     kmbox_cmd_set_baud(uint32_t baud) { (void)baud; }
uint32_t uart_current_baud(void) { return (uint32_t)CMD_BAUD; }
uint16_t uart_tx_room(void) { return 256; }
uint32_t uart_overrun(void) { return 0; }
