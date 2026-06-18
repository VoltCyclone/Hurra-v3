// desc_xfer.c — descriptor blob chunk/reassemble codec. See desc_xfer.h.
// Pure, MMIO-free; host-tested by test/desc_xfer_test.c.
#include "desc_xfer.h"
#include <string.h>

uint16_t desc_xfer_chunk_count(uint16_t total)
{
    if (total == 0) return 0;
    return (uint16_t)((total + DESC_XFER_DATA_PER_CHUNK - 1) / DESC_XFER_DATA_PER_CHUNK);
}

uint8_t desc_xfer_pack_chunk(const uint8_t *blob, uint16_t total, uint16_t idx,
                             uint8_t *out)
{
    uint16_t nchunks = desc_xfer_chunk_count(total);
    if (idx >= nchunks) return 0;

    uint16_t off = (uint16_t)(idx * DESC_XFER_DATA_PER_CHUNK);
    uint16_t n   = (uint16_t)(total - off);
    if (n > DESC_XFER_DATA_PER_CHUNK) n = DESC_XFER_DATA_PER_CHUNK;

    out[0] = (uint8_t)(idx & 0xFF);
    out[1] = (uint8_t)(idx >> 8);
    out[2] = (uint8_t)(total & 0xFF);
    out[3] = (uint8_t)(total >> 8);
    memcpy(&out[DESC_XFER_HDR], &blob[off], n);
    return (uint8_t)(DESC_XFER_HDR + n);
}

void desc_xfer_reset(desc_xfer_ctx_t *ctx)
{
    ctx->total    = 0;
    ctx->next_idx = 0;
    ctx->have     = 0;
    ctx->active   = 0;
}

desc_xfer_result_t desc_xfer_accept(desc_xfer_ctx_t *ctx, const uint8_t *payload,
                                    uint8_t len)
{
    // A valid chunk has at least the header. Anything shorter is junk -> restart.
    if (len < DESC_XFER_HDR) { desc_xfer_reset(ctx); return DESC_XFER_RESTART; }

    uint16_t idx   = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    uint16_t total = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
    uint8_t  dlen  = (uint8_t)(len - DESC_XFER_HDR);

    // Index 0 (re)starts a transfer: adopt this chunk's total and begin fresh.
    // This is how a restarted/looping sender resyncs a reset receiver, and how a
    // mid-stream new blob (different total) takes over cleanly.
    if (idx == 0) {
        if (total == 0 || total > DESC_XFER_BUF_MAX) {
            desc_xfer_reset(ctx);
            return DESC_XFER_RESTART;
        }
        ctx->total    = total;
        ctx->next_idx = 0;
        ctx->have     = 0;
        ctx->active   = 1;
    } else {
        // A non-zero index must continue the active transfer in order, with a
        // matching total. Anything else is a gap/inconsistency -> restart.
        if (!ctx->active || idx != ctx->next_idx || total != ctx->total) {
            desc_xfer_reset(ctx);
            return DESC_XFER_RESTART;
        }
    }

    // Bounds-check the write (a corrupt dlen must never overflow the buffer).
    uint16_t off = (uint16_t)(idx * DESC_XFER_DATA_PER_CHUNK);
    if ((uint32_t)off + dlen > ctx->total) {
        desc_xfer_reset(ctx);
        return DESC_XFER_RESTART;
    }
    memcpy(&ctx->buf[off], &payload[DESC_XFER_HDR], dlen);
    ctx->have     = (uint16_t)(off + dlen);
    ctx->next_idx = (uint16_t)(idx + 1);

    if (ctx->have >= ctx->total) {
        ctx->active = 0;
        return DESC_XFER_COMPLETE;
    }
    return DESC_XFER_IN_PROGRESS;
}
