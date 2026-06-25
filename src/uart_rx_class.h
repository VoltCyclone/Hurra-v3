#pragma once
#include <stdint.h>
#include <stdbool.h>

// Classification of a USART RX status word (STATR snapshot) into the error
// counters to bump and whether the received byte must be dropped.
//
// On the CH32H417 USART, ORE/FE/NE/PE are cleared by the software sequence
// "read STATR then read DATAR" — so the ISR snapshots STATR before reading the
// data register, then calls this to decide accounting. Pulled into a header so
// the bitmask logic is unit-testable without the USART peripheral.
//
// Counting policy (matches the A4 fix):
//   - ORE (overrun): a hardware overrun. Count, but keep the byte — DATAR still
//     holds a valid frame; dropping it would lose good data on a transient.
//   - FE (framing) / NE (noise): the frame is corrupt. Count AND drop the byte.
//   - PE (parity): not used (8N1, no parity) — ignored here.

#define UART_RX_FLAG_ORE  0x0008u
#define UART_RX_FLAG_NE   0x0004u
#define UART_RX_FLAG_FE   0x0002u

typedef struct {
	bool count_or;   // increment hardware-overrun counter
	bool count_fe;   // increment framing-error counter
	bool count_ne;   // increment noise-error counter
	bool drop;       // true => do not enqueue the byte (corrupt frame)
} uart_rx_class_t;

static inline uart_rx_class_t uart_rx_classify(uint16_t statr)
{
	uart_rx_class_t c = {0};
	c.count_or = (statr & UART_RX_FLAG_ORE) != 0;
	c.count_fe = (statr & UART_RX_FLAG_FE)  != 0;
	c.count_ne = (statr & UART_RX_FLAG_NE)  != 0;
	c.drop     = c.count_fe || c.count_ne;   // ORE alone keeps the (valid) byte
	return c;
}
