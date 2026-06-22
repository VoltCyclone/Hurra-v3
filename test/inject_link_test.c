// test/inject_link_test.c — Board B inject FIFO: enqueue, drain-as-frames,
// overflow-drops-oldest.
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "inject_link.h"
#include "spi_frame.h"
#include "two_board.h"

static int failures = 0;
#define CHECK(c,m) do { if(!(c)){printf("FAIL: %s\n",m);failures++;} else printf("ok: %s\n",m);} while(0)

// Capture frames the drain emits.
static uint8_t cap_tag[64]; static uint8_t cap_b0[64]; static int cap_n;
static void fake_xchg(const uint8_t tx[32], uint8_t rx[32]) {
	(void)rx;
	uint8_t type, seq, len; const uint8_t *p;
	if (spi_frame_unpack(tx, &type, &seq, &p, &len) == SPI_FRAME_OK &&
	    type == TWO_BOARD_TYPE_INJECT) {
		cap_tag[cap_n] = p[0]; cap_b0[cap_n] = p[1]; cap_n++;
	}
}

int main(void) {
	inject_link_init();
	uint8_t seq = 0;

	// 1. Two records drain in FIFO order as INJECT frames.
	{
		uint8_t b1[7]={5,0,0,0,0,0,0}, b2[7]={9,0,0,0,0,0,0};
		CHECK(inject_link_push(1 /*INJECT_MOUSE*/, b1), "push 1");
		CHECK(inject_link_push(1, b2), "push 2");
		cap_n = 0;
		inject_link_drain(&seq, fake_xchg);
		CHECK(cap_n == 2, "drained 2 frames");
		CHECK(cap_b0[0] == 5 && cap_b0[1] == 9, "FIFO order preserved");
		CHECK(seq == 2, "seq advanced by 2");
	}

	// 2. Overflow drops oldest: push DEPTH+2 without draining.
	{
		inject_link_init();
		for (int i = 0; i < INJECT_LINK_DEPTH + 2; i++) {
			uint8_t b[7] = { (uint8_t)i, 0,0,0,0,0,0 };
			inject_link_push(1, b);
		}
		// Capacity = INJECT_LINK_DEPTH-1 (7) with the keep-one-open ring.
		// Pushing DEPTH+2 (10) evicts the first 10-7=3 (indices 0,1,2);
		// survivors are 3..9.
		CHECK(inject_link_drops == 3, "three oldest dropped on overflow");
		cap_n = 0; seq = 0;
		inject_link_drain(&seq, fake_xchg);
		CHECK(cap_n == INJECT_LINK_DEPTH - 1, "drain yields capacity frames");
		CHECK(cap_b0[0] == 3, "oldest surviving is index 3 (0,1,2 dropped)");
	}

	printf(failures ? "\nFAILED (%d)\n" : "\nPASS\n", failures);
	return failures ? 1 : 0;
}
