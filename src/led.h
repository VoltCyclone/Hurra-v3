#pragma once
#include <stdint.h>

void led_init(void);
void led_on(void);
void led_off(void);
void led_toggle(void);
void led_blink_forever(uint8_t count, uint16_t on_ms, uint16_t off_ms);
void led_heartbeat_start(void);
void led_heartbeat_set_rate(uint16_t centihz);  /* 100 = 1.00 Hz */
