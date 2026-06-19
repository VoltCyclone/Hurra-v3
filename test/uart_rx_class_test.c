// test/uart_rx_class_test.c — host-native tests for USART RX status
// classification (which error counters to bump, whether to drop the byte).
//
// uart.c's IRQ handler is pure USART MMIO and cannot run off-target; only the
// status-word decision is extracted (uart_rx_class.h) and verified here.

#include <stdio.h>
#include <stdint.h>
#include "uart_rx_class.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    printf("FAIL: %s\n", msg); failures++; } else { \
    printf("ok: %s\n", msg); } } while (0)

int main(void) {
	// Clean byte (RXNE only, no error bits): count nothing, keep the byte.
	{
		uart_rx_class_t c = uart_rx_classify(0x0020 /* RXNE */);
		CHECK(!c.count_or && !c.count_fe && !c.count_ne && !c.drop,
		      "clean byte: no counters, not dropped");
	}

	// Overrun: count overrun, but keep the byte (DATAR holds a valid frame).
	{
		uart_rx_class_t c = uart_rx_classify(UART_RX_FLAG_ORE);
		CHECK(c.count_or && !c.drop, "ORE: counted, byte kept");
		CHECK(!c.count_fe && !c.count_ne, "ORE: no framing/noise miscount");
	}

	// Framing error: count framing AND drop the corrupt byte.
	{
		uart_rx_class_t c = uart_rx_classify(UART_RX_FLAG_FE);
		CHECK(c.count_fe && c.drop, "FE: counted and dropped");
		CHECK(!c.count_or && !c.count_ne, "FE: no overrun/noise miscount");
	}

	// Noise error: count noise AND drop.
	{
		uart_rx_class_t c = uart_rx_classify(UART_RX_FLAG_NE);
		CHECK(c.count_ne && c.drop, "NE: counted and dropped");
	}

	// Combined ORE+FE (overrun on a corrupt frame): both counted, dropped.
	{
		uart_rx_class_t c = uart_rx_classify(UART_RX_FLAG_ORE | UART_RX_FLAG_FE);
		CHECK(c.count_or && c.count_fe && c.drop,
		      "ORE+FE: both counted, dropped");
	}

	if (failures == 0) printf("ALL PASS\n");
	else printf("%d FAILURE(S)\n", failures);
	return failures ? 1 : 0;
}
