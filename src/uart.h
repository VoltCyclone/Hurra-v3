#pragma once
#include <stdint.h>
#include <stdbool.h>

void     uart_init(uint32_t baud);
void     uart_set_baud(uint32_t baud);
uint32_t uart_current_baud(void);

// RX: DMA circular ring. Returns bytes available and a pointer to read from.
uint16_t uart_rx_available(void);
// Copy up to `max` received bytes into `dst`; returns count copied.
uint16_t uart_rx_read(uint8_t *dst, uint16_t max);

// TX: enqueue bytes; returns count accepted (may be < len if TX busy/full).
uint16_t uart_tx_write(const uint8_t *src, uint16_t len);
uint16_t uart_tx_room(void);
void     uart_tx_flush(void);    // kick DMA if idle and data pending

// Error counters (mirrors v2 kmbox stats).
uint32_t uart_overrun(void);
uint32_t uart_framing(void);
uint32_t uart_noise(void);
uint32_t uart_rx_byte_count(void);
uint32_t uart_tx_byte_count(void);
