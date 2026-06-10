#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "desc_capture.h"

void kmbox_init(void);

// Split poll: fast runs unconditionally each iteration (timers, tx, LED
// timeout); heavy only when kmbox_rx_pending() reports UART bytes pending.
void kmbox_poll_fast(void);
void kmbox_poll_heavy(void);
bool kmbox_rx_pending(void);

void kmbox_merge_report(uint8_t iface_protocol, uint8_t * restrict report, uint8_t len);

void kmbox_cache_endpoints(const captured_descriptors_t *desc);

void kmbox_send_pending(void);

void kmbox_inject_mouse(int16_t dx, int16_t dy, uint8_t buttons,
                        int8_t wheel);
void kmbox_inject_keyboard(uint8_t modifier, const uint8_t keys[6]);

void kmbox_schedule_click_release(uint8_t button_mask, uint32_t delay_ms);
void kmbox_schedule_kb_release(uint8_t key, uint32_t delay_ms);

uint32_t kmbox_frame_count(void);
uint32_t kmbox_error_count(void);
uint32_t kmbox_rx_byte_count(void);
uint32_t kmbox_tx_byte_count(void);
uint32_t kmbox_uart_overrun(void);  // OR: FIFO overrun
uint32_t kmbox_uart_framing(void);  // FE: baud mismatch / signal
uint32_t kmbox_uart_noise(void);    // NF: electrical noise
uint32_t kmbox_tx_overflow(void);   // bytes dropped when tx_ring was full
uint32_t kmbox_rx_drv_overrun(void);
uint16_t kmbox_tx_room(void);
void     kmbox_set_baud(uint32_t baud);
uint32_t kmbox_current_baud(void);
