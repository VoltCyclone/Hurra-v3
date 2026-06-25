// src/ferrum.h — Ferrum ASCII protocol parser public API.
//
// Wire format: km.<name>(<args>)\r\n   (or just \n; alias m(x,y) for km.move)
// Read commands reply value\r\n. Write commands reply nothing. No echo, no prompt.
#pragma once
#include <stdint.h>
#include <stdbool.h>

void ferrum_init(void);

// Feed one received UART byte. The line buffer accumulates until \r or \n,
// then parses and dispatches. Replies and callback events go via the transport.
void ferrum_feed_byte(uint8_t b);

// Reset parser state; call after a UART error to discard a partial line.
void ferrum_reset(void);

// Poll-loop tick (any rate). Drives the catch_xy deadline check independent of
// UART RX so a host that idles after km.catch_xy(dur) still gets its reply.
void ferrum_tick(void);

// Transport hook: ferrum.c calls this to send a reply or callback line.
typedef void (*ferrum_tx_fn)(const uint8_t *data, uint16_t len);
void ferrum_set_tx(ferrum_tx_fn tx);

// Callback hooks: call from the merge path when injection state changes or HID
// input is observed. ferrum.c emits the callback line if the callback is enabled.
void ferrum_notify_buttons(uint8_t buttons_bitmap);
void ferrum_notify_axes(int16_t dx, int16_t dy, int8_t scroll);
void ferrum_notify_keys(const uint8_t keys[6]);
