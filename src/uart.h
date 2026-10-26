#pragma once
#include <stdint.h>
#include <stdbool.h>

void     uart_init(uint32_t baud);
// Foreground: schedule a nonzero baud; flush applies it after ring + TC drain.
void     uart_set_baud(uint32_t baud);
uint32_t uart_current_baud(void);

// RX: bytes currently buffered in the ring.
uint16_t uart_rx_available(void);
// Copy up to `max` received bytes into `dst`; returns count copied.
uint16_t uart_rx_read(uint8_t *dst, uint16_t max);
// Foreground parser must reset on loss, including a read that returns zero.
bool     uart_rx_take_loss(void);

// TX: single foreground producer; enqueue the whole write or return zero.
uint16_t uart_tx_write(const uint8_t *src, uint16_t len);
uint16_t uart_tx_room(void);
void     uart_tx_flush(void);    // arm TXE if data is pending

// Error counters.
uint32_t uart_overrun(void);
uint32_t uart_framing(void);
uint32_t uart_noise(void);
uint32_t uart_rx_drop(void);   /* bytes dropped due to a full RX ring (software) */
uint32_t uart_rx_byte_count(void);
uint32_t uart_tx_byte_count(void);
