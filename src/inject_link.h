#pragma once
#include <stdint.h>
#include <stdbool.h>

// Board B (host role) injection FIFO. The V5F command parser's act_* calls land
// in kmbox_cmd_host.c, which pushes records here; the host loop drains them as
// TWO_BOARD_TYPE_INJECT SPI frames to Board A. Depth-8: injection is human-rate
// and the SPI link is far faster, so this is burst margin, not flow control.
#define INJECT_LINK_DEPTH 8

typedef void (*inject_exchange_fn)(const uint8_t tx[32], uint8_t rx[32]);

extern uint32_t inject_link_drops;   // cumulative oldest-dropped count (diag)

void inject_link_init(void);
// Copy one record (tag + 7 payload bytes) into the FIFO. On full, drop the OLDEST
// (bump inject_link_drops) and enqueue this one. Returns false iff a drop occurred.
bool inject_link_push(uint8_t tag, const uint8_t b[7]);
// Pop every queued record, pack each as a TYPE_INJECT frame with SEQ from *seq
// (post-incremented per frame), and send via xchg. No-op when empty.
void inject_link_drain(uint8_t *seq, inject_exchange_fn xchg);
