#pragma once
// Shim: actions.c and the protocol parsers (hurra.c/ferrum.c) call kmbox_*
// functions; on the V3F core these route into the inter-core channel via
// kmbox_cmd_* (injection sinks) or the UART transport via uart_* (link stats /
// baud). The HID-merge half lives on V5F as usb_merge.c.
#include <stdint.h>
#include "kmbox_cmd.h"
#include "uart.h"

// ── Injection sinks (encode an ICC record to V5F) ───────────────────────────
#define kmbox_inject_mouse           kmbox_cmd_inject_mouse
#define kmbox_inject_keyboard        kmbox_cmd_inject_keyboard
#define kmbox_schedule_click_release kmbox_cmd_schedule_click_release
#define kmbox_schedule_kb_release    kmbox_cmd_schedule_kb_release

// ── Link control / status (map to UART transport + kmbox_cmd) ───────────────
#define kmbox_set_baud               kmbox_cmd_set_baud   // schedules UART baud change
#define kmbox_current_baud           uart_current_baud    // active link baud
#define kmbox_tx_room                uart_tx_room         // free bytes in TX ring

// Thin wrapper over uart_overrun(); read only for the STATS reply. No separate
// driver-level overrun exists on this transport.
static inline uint32_t kmbox_rx_drv_overrun(void) { return uart_overrun(); }
