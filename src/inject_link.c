#include "inject_link.h"
#include "spi_frame.h"
#include "two_board.h"
#include <string.h>

typedef struct { uint8_t tag; uint8_t b[7]; } inject_rec_t;

static inject_rec_t s_ring[INJECT_LINK_DEPTH];
static uint8_t s_head, s_tail;   // s_head==s_tail => empty; one slot kept open
uint32_t inject_link_drops;

void inject_link_init(void) { s_head = s_tail = 0; inject_link_drops = 0; }

static uint8_t next(uint8_t i) { return (uint8_t)((i + 1) % INJECT_LINK_DEPTH); }

bool inject_link_push(uint8_t tag, const uint8_t b[7])
{
	bool dropped = false;
	if (next(s_tail) == s_head) {           // full: evict oldest
		s_head = next(s_head);
		inject_link_drops++;
		dropped = true;
	}
	s_ring[s_tail].tag = tag;
	memcpy(s_ring[s_tail].b, b, 7);
	s_tail = next(s_tail);
	return !dropped;
}

void inject_link_drain(uint8_t *seq, inject_exchange_fn xchg)
{
	while (s_head != s_tail) {
		const inject_rec_t *r = &s_ring[s_head];
		uint8_t pay[8];
		pay[0] = r->tag;
		memcpy(&pay[1], r->b, 7);
		uint8_t tx[SPI_FRAME_SLOT_SIZE], rx[SPI_FRAME_SLOT_SIZE];
		if (spi_frame_pack(tx, TWO_BOARD_TYPE_INJECT, (*seq)++, pay, sizeof pay)
		    == SPI_FRAME_OK) {
			xchg(tx, rx);
		}
		s_head = next(s_head);
	}
}
