// src/ferrum.h — Ferrum ASCII protocol parser
//
// Wire format: km.<name>(<args>)\r\n   (or just \n; alias m(x,y) for km.move)
// Read commands reply value\r\n.  Write commands reply nothing.
// No echo, no >>> prompt.
#pragma once
#include <stdint.h>
#include <stdbool.h>

void ferrum_init(void);

// Feed one received byte from UART.  Internal line buffer accumulates
// until \r or \n, then parses and dispatches.  Replies (for read
// commands) and callback events are sent via the transport callback.
void ferrum_feed_byte(uint8_t b);

// Reset parser state — call after a UART error to discard a partial line.
void ferrum_reset(void);

// Poll-loop tick — call from kmbox_poll() at any rate.  Drives the
// catch_xy deadline check independent of UART RX activity so a host that
// idles after km.catch_xy(dur) still gets its reply on time.
void ferrum_tick(void);

// Transport hook — ferrum.c calls this to send a reply or callback line.
// kmbox.c implements it and points it at uart_tx_frame.
typedef void (*ferrum_tx_fn)(const uint8_t *data, uint16_t len);
void ferrum_set_tx(ferrum_tx_fn tx);

// Callback hooks — call these from kmbox_merge_report when injection
// state changes or HID input is observed.  ferrum.c emits the documented
// callback lines if the corresponding callback is enabled.
void ferrum_notify_buttons(uint8_t buttons_bitmap);
void ferrum_notify_axes(int16_t dx, int16_t dy, int8_t scroll);
void ferrum_notify_keys(const uint8_t keys[6]);
