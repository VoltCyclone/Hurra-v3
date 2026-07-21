// desc_xfer.h — descriptor blob chunk/reassemble codec for the SPI link.
//
// The captured_descriptors_t blob (~7 KB) is far larger than one 26-byte SPI
// hot-path payload, so Board B (host) ships it as a sequence of self-describing
// chunks and Board A (device) reassembles it before enumerating. Each chunk's
// payload is:  [idx:u16 LE][total:u16 LE][<= DATA_PER_CHUNK data bytes]. The
// header makes every chunk self-describing, so the receiver needs no out-of-band
// setup and can detect a gap or a fresh restart.
//
// Reliability model is restart-on-gap (no ACK channel): the receiver tracks the
// next expected index; a CRC failure (caught by the frame codec before this layer)
// or an index gap returns DESC_XFER_RESTART and the caller resets. Board B re-sends
// the whole blob on a loop, so a reset receiver catches the next pass.
//
// Parallel to report_xfer.h (the HID-report reassembler), which shares this
// restart-on-gap shape but uses u8 headers + routing fields and must reset on a
// frame-loss gap; here the blob re-sends are byte-identical, so a gap is benign.
//
// Pure, MMIO-free, host-tested (test/desc_xfer_test.c, in `make test`).
#ifndef DESC_XFER_H
#define DESC_XFER_H

#include <stdint.h>
#include "desc_capture.h"   // sizeof(captured_descriptors_t) bounds the rx buffer

// Per-chunk header (inside the SPI frame payload) and data capacity.
#define DESC_XFER_HDR             4    // idx:u16 + total:u16, little-endian
#define DESC_XFER_PAYLOAD_MAX     26   // == SPI_FRAME_MAX_PAYLOAD
#define DESC_XFER_DATA_PER_CHUNK  (DESC_XFER_PAYLOAD_MAX - DESC_XFER_HDR)  // 22

// Reassembly buffer: must hold the entire serialized blob. The blob we ship is a
// raw memcpy of captured_descriptors_t, so size to that.
#define DESC_XFER_BUF_MAX         ((uint16_t)sizeof(captured_descriptors_t))

typedef enum {
    DESC_XFER_IN_PROGRESS = 0,  // chunk accepted, more expected
    DESC_XFER_COMPLETE,         // final chunk accepted; ctx.buf/ctx.total valid
    DESC_XFER_RESTART,          // gap / inconsistency; caller should reset + wait
} desc_xfer_result_t;

typedef struct {
    uint8_t  buf[DESC_XFER_BUF_MAX];
    uint16_t total;        // total blob length being assembled (from chunk header)
    uint16_t next_idx;     // next chunk index expected
    uint16_t have;         // bytes written so far
    uint8_t  active;       // a transfer is in progress
} desc_xfer_ctx_t;

// Number of chunks needed to send a `total`-byte blob (ceil division). 0 if total==0.
uint16_t desc_xfer_chunk_count(uint16_t total);

// Pack chunk `idx` of `blob` (length `total`) into `out` (>= DESC_XFER_PAYLOAD_MAX
// bytes). Returns the payload length written (header + data for this chunk), or 0
// if idx is out of range. The returned payload is what the caller hands to
// spi_frame_pack as the SPI frame payload.
uint8_t desc_xfer_pack_chunk(const uint8_t *blob, uint16_t total, uint16_t idx,
                             uint8_t *out);

// Reset a receive context to idle (no transfer in progress).
void desc_xfer_reset(desc_xfer_ctx_t *ctx);

// Feed one received chunk payload (as recovered by spi_frame_unpack) into the
// reassembler. Returns IN_PROGRESS / COMPLETE / RESTART. On RESTART the context is
// auto-reset, and an index-0 chunk in the SAME call would have been adopted as a
// fresh transfer — so a stream that restarts at index 0 resyncs cleanly without
// the caller doing anything. On COMPLETE, ctx.buf holds ctx.total assembled bytes.
desc_xfer_result_t desc_xfer_accept(desc_xfer_ctx_t *ctx, const uint8_t *payload,
                                    uint8_t len);

#endif // DESC_XFER_H
