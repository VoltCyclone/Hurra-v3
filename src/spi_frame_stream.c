// spi_frame_stream.c — SOF-scanning frame streamer. See spi_frame_stream.h.
// Pure, MMIO-free; host-tested by test/spi_frame_stream_test.c.
#include "spi_frame_stream.h"
#include <string.h>

void spi_frame_stream_init(spi_frame_stream_t *st)
{
    st->len = 0;
}

// Slide the window so it restarts at the next SOF (0x68) byte AFTER index 0,
// discarding everything before it. Leaves buf/len holding the bytes from that SOF
// onward. If no later SOF exists, the window is cleared.
static void resync(spi_frame_stream_t *st)
{
    uint8_t i;
    for (i = 1; i < st->len; i++) {
        if (st->buf[i] == SPI_FRAME_SOF) break;
    }
    if (i >= st->len) {
        // No SOF after position 0 — drop everything (the last byte, if it is a SOF,
        // is handled by the caller appending fresh; keep a trailing lone SOF).
        if (st->len > 0 && st->buf[st->len - 1] == SPI_FRAME_SOF) {
            st->buf[0] = SPI_FRAME_SOF;
            st->len = 1;
        } else {
            st->len = 0;
        }
        return;
    }
    uint8_t n = (uint8_t)(st->len - i);
    memmove(st->buf, st->buf + i, n);
    st->len = n;
}

bool spi_frame_stream_push(spi_frame_stream_t *st, uint8_t byte, uint8_t *out_slot)
{
    // While unsynced (empty window), only a SOF byte starts a frame; skip noise.
    if (st->len == 0) {
        if (byte != SPI_FRAME_SOF) return false;
        st->buf[0] = byte;
        st->len = 1;
        return false;
    }

    // Append into the window.
    st->buf[st->len++] = byte;

    if (st->len < SPI_FRAME_SLOT_SIZE) return false;

    // Window full — validate.
    uint8_t type, seq, len;
    const uint8_t *payload;
    if (spi_frame_unpack(st->buf, &type, &seq, &payload, &len) == SPI_FRAME_OK) {
        if (out_slot) memcpy(out_slot, st->buf, SPI_FRAME_SLOT_SIZE);
        st->len = 0;                 // emitted — start fresh
        return true;
    }

    // Bad frame at this alignment — slide to the next SOF candidate and keep going.
    resync(st);
    return false;
}
