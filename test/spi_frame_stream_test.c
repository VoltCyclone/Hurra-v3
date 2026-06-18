// Host unit test for the SOF-scanning frame streamer (src/spi_frame_stream.c).
// The streamer turns a raw SPI byte stream (no frame boundary) into aligned,
// CRC-validated 32-byte slots — the receiver half of the IRQ-driven SPI slave.
// Pure, host-tested, in `make test`.
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "spi_frame_stream.h"
#include "spi_frame.h"

#define CHECK(cond) do { if (!(cond)) { \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); assert(0); } } while (0)

// Push a buffer of bytes through the streamer; collect every emitted slot into
// out[] (each SPI_FRAME_SLOT_SIZE bytes). Returns the number of slots emitted.
static int push_all(spi_frame_stream_t *st, const uint8_t *buf, int n,
                    uint8_t *out, int out_max)
{
    int got = 0;
    for (int i = 0; i < n; i++) {
        uint8_t slot[SPI_FRAME_SLOT_SIZE];
        if (spi_frame_stream_push(st, buf[i], slot)) {
            if (got < out_max) memcpy(out + got * SPI_FRAME_SLOT_SIZE, slot,
                                      SPI_FRAME_SLOT_SIZE);
            got++;
        }
    }
    return got;
}

int main(void)
{
    uint8_t a[SPI_FRAME_SLOT_SIZE], b[SPI_FRAME_SLOT_SIZE];
    const uint8_t payA[] = { 1, 2, 3, 4, 5 };
    const uint8_t payB[] = { 0x10, 0x20, 0x30 };
    spi_frame_pack(a, 0x02 /*type*/, 0x11 /*seq*/, payA, sizeof payA);
    spi_frame_pack(b, 0x03, 0x12, payB, sizeof payB);

    /* 1. A single clean frame is emitted byte-for-byte. */
    {
        spi_frame_stream_t st; spi_frame_stream_init(&st);
        uint8_t out[SPI_FRAME_SLOT_SIZE];
        CHECK(push_all(&st, a, SPI_FRAME_SLOT_SIZE, out, 1) == 1);
        CHECK(memcmp(out, a, SPI_FRAME_SLOT_SIZE) == 0);
    }

    /* 2. Two back-to-back frames both emit, in order. */
    {
        spi_frame_stream_t st; spi_frame_stream_init(&st);
        uint8_t stream[2 * SPI_FRAME_SLOT_SIZE];
        memcpy(stream, a, SPI_FRAME_SLOT_SIZE);
        memcpy(stream + SPI_FRAME_SLOT_SIZE, b, SPI_FRAME_SLOT_SIZE);
        uint8_t out[2 * SPI_FRAME_SLOT_SIZE];
        CHECK(push_all(&st, stream, sizeof stream, out, 2) == 2);
        CHECK(memcmp(out, a, SPI_FRAME_SLOT_SIZE) == 0);
        CHECK(memcmp(out + SPI_FRAME_SLOT_SIZE, b, SPI_FRAME_SLOT_SIZE) == 0);
    }

    /* 3. JUNK PREFIX: leading garbage (incl. a stray 0x68 that fails CRC) before a
     * real frame — the streamer must resync and still emit the real frame. */
    {
        spi_frame_stream_t st; spi_frame_stream_init(&st);
        uint8_t stream[8 + SPI_FRAME_SLOT_SIZE];
        uint8_t junk[8] = { 0x00, 0xFF, 0x68, 0x01, 0x99, 0x68, 0xAA, 0x55 };
        memcpy(stream, junk, 8);
        memcpy(stream + 8, a, SPI_FRAME_SLOT_SIZE);
        uint8_t out[SPI_FRAME_SLOT_SIZE];
        CHECK(push_all(&st, stream, sizeof stream, out, 1) == 1);
        CHECK(memcmp(out, a, SPI_FRAME_SLOT_SIZE) == 0);
    }

    /* 4. MID-STREAM DESYNC: a truncated frame (only part of `a`, no valid CRC at the
     * 32-byte mark) followed by a clean `b` — the streamer drops the bad window and
     * recovers `b`. */
    {
        spi_frame_stream_t st; spi_frame_stream_init(&st);
        uint8_t stream[10 + SPI_FRAME_SLOT_SIZE];
        memcpy(stream, a, 10);                 // first 10 bytes of a (truncated)
        memcpy(stream + 10, b, SPI_FRAME_SLOT_SIZE);
        uint8_t out[SPI_FRAME_SLOT_SIZE];
        int got = push_all(&st, stream, sizeof stream, out, 1);
        CHECK(got == 1);
        CHECK(memcmp(out, b, SPI_FRAME_SLOT_SIZE) == 0);
    }

    /* 5. SOF byte value appearing inside a payload doesn't break a valid frame:
     * frame `a` whose payload happens to contain 0x68 still emits as-is (the scanner
     * only re-scans when a window FAILS validation). */
    {
        uint8_t c[SPI_FRAME_SLOT_SIZE];
        const uint8_t payC[] = { 0x68, 0x68, 0x68 };   // payload full of SOF bytes
        spi_frame_pack(c, 0x02, 0x20, payC, sizeof payC);
        spi_frame_stream_t st; spi_frame_stream_init(&st);
        uint8_t out[SPI_FRAME_SLOT_SIZE];
        CHECK(push_all(&st, c, SPI_FRAME_SLOT_SIZE, out, 1) == 1);
        CHECK(memcmp(out, c, SPI_FRAME_SLOT_SIZE) == 0);
    }

    printf("spi_frame_stream_test: all passed\n");
    return 0;
}
