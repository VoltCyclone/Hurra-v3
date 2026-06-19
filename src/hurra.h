// src/hurra.h — Hurra binary protocol parser public API.
#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef void (*hurra_tx_fn)(const uint8_t *buf, uint16_t len);

void hurra_init(void);
void hurra_reset(void);
void hurra_set_tx(hurra_tx_fn tx);

void hurra_feed_byte(uint8_t b);

// Feed a contiguous span of received bytes in one call.
void hurra_feed(const uint8_t *buf, uint16_t len);
void hurra_tick(void);

void hurra_notify_buttons(uint8_t buttons_bitmap);
void hurra_notify_axes(int16_t dx, int16_t dy, int8_t scroll);
void hurra_notify_keys(const uint8_t keys[6]);

// Physical-only telemetry (`monitor`). The merge path calls these with pre-merge,
// pre-mask physical values; they emit TLM_PHYS_* only while CB_PHYS is enabled.
// hurra_phys_enabled() lets the hot path skip capture when monitoring is off.
bool hurra_phys_enabled(void);
void hurra_notify_phys_buttons(uint8_t buttons_bitmap);
void hurra_notify_phys_axes(int16_t dx, int16_t dy, int8_t wheel);
void hurra_notify_phys_keys(uint8_t modifier, const uint8_t keys[6]);
