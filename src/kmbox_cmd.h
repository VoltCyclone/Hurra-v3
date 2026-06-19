#pragma once
#include <stdint.h>
#include <stdbool.h>

void kmbox_cmd_init(void);        // proto_init + bind tx to uart + act_init
void kmbox_cmd_poll(void);        // drain UART -> proto_feed; flush tx
bool kmbox_cmd_rx_pending(void);

// Injection sinks: encode an ICC record to V5F. Implementations behind the
// kmbox.h shim that actions.c calls.
void kmbox_cmd_inject_mouse(int16_t dx, int16_t dy, uint8_t buttons, int8_t wheel);
void kmbox_cmd_inject_keyboard(uint8_t modifier, const uint8_t keys[6]);
void kmbox_cmd_schedule_click_release(uint8_t button_mask, uint32_t delay_ms);
void kmbox_cmd_schedule_kb_release(uint8_t key, uint32_t delay_ms);
void kmbox_cmd_set_baud(uint32_t baud);
void kmbox_cmd_set_human_level(uint8_t level);

// V3F-local accessors for status display (no telemetry).
uint32_t kmbox_cmd_inj_mouse_count(void);
uint32_t kmbox_cmd_inj_kbd_count(void);
