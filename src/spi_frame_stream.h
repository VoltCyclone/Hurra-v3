// spi_frame_stream.h — SOF-scanning frame streamer.
//
// Turns a raw SPI byte stream (no frame boundary — the IRQ-driven slave just
// captures whatever the master clocks) into aligned, CRC-validated 32-byte slots.
// Feed it one received byte at a time; it emits a slot when a 32-byte window
// starting at a SOF (0x68) marker passes spi_frame_unpack. On a window that fails
// validation it slides forward to the next SOF candidate and keeps scanning, so it
// self-resyncs after junk, a truncated frame, or a mid-stream desync — the failure
// mode that broke the polled slave.
//
// Pure, MMIO-free, host-tested (test/spi_frame_stream_test.c).
#ifndef SPI_FRAME_STREAM_H
#define SPI_FRAME_STREAM_H

#include <stdint.h>
#include <stdbool.h>
#include "spi_frame.h"   // SPI_FRAME_SLOT_SIZE, SPI_FRAME_SOF

typedef struct {
    uint8_t  buf[SPI_FRAME_SLOT_SIZE];   // window being assembled
    uint8_t  len;                        // bytes currently in buf
} spi_frame_stream_t;

// Reset the streamer to empty / unsynced.
void spi_frame_stream_init(spi_frame_stream_t *st);

// Push one received byte. If this byte completes a valid 32-byte frame, copies it
// into out_slot[SPI_FRAME_SLOT_SIZE] and returns true (the streamer resets to
// receive the next frame). Otherwise returns false. out_slot may be NULL to just
// advance state. Self-resyncs on junk / misalignment.
bool spi_frame_stream_push(spi_frame_stream_t *st, uint8_t byte, uint8_t *out_slot);

#endif // SPI_FRAME_STREAM_H
