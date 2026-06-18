// Host unit test for the descriptor chunk/reassemble codec (src/desc_xfer.c).
// Pure, no MMIO — built/run via `make test`. This is the step-4b transport that
// moves a large captured_descriptors_t blob across the 26-byte SPI payload.
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "desc_xfer.h"

#define CHECK(cond) do { if (!(cond)) { \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); assert(0); } } while (0)

// Build a deterministic pseudo-blob so reassembly can be checked byte-for-byte.
static void fill_blob(uint8_t *b, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) b[i] = (uint8_t)(i * 7u + 3u);
}

int main(void)
{
    /* ---- chunk_count: ceil(total / DATA_PER_CHUNK) ---- */
    CHECK(desc_xfer_chunk_count(0) == 0);
    CHECK(desc_xfer_chunk_count(1) == 1);
    CHECK(desc_xfer_chunk_count(DESC_XFER_DATA_PER_CHUNK) == 1);
    CHECK(desc_xfer_chunk_count(DESC_XFER_DATA_PER_CHUNK + 1) == 2);
    CHECK(desc_xfer_chunk_count(100) == (uint16_t)((100 + DESC_XFER_DATA_PER_CHUNK - 1)
                                                   / DESC_XFER_DATA_PER_CHUNK));

    /* ---- round-trip: pack every chunk, accept in order, recover the blob ---- */
    {
        const uint16_t N = 100;            // ~5 chunks at 22 data bytes each
        uint8_t blob[N];
        fill_blob(blob, N);

        desc_xfer_ctx_t ctx;
        desc_xfer_reset(&ctx);

        uint16_t nchunks = desc_xfer_chunk_count(N);
        desc_xfer_result_t r = DESC_XFER_IN_PROGRESS;
        for (uint16_t i = 0; i < nchunks; i++) {
            uint8_t pay[DESC_XFER_PAYLOAD_MAX];
            uint8_t plen = desc_xfer_pack_chunk(blob, N, i, pay);
            CHECK(plen > 0);
            CHECK(plen <= DESC_XFER_PAYLOAD_MAX);
            r = desc_xfer_accept(&ctx, pay, plen);
            if (i < nchunks - 1) CHECK(r == DESC_XFER_IN_PROGRESS);
        }
        CHECK(r == DESC_XFER_COMPLETE);
        CHECK(ctx.total == N);
        CHECK(memcmp(ctx.buf, blob, N) == 0);
    }

    /* ---- single-chunk blob completes immediately ---- */
    {
        const uint16_t N = 10;
        uint8_t blob[N]; fill_blob(blob, N);
        desc_xfer_ctx_t ctx; desc_xfer_reset(&ctx);
        uint8_t pay[DESC_XFER_PAYLOAD_MAX];
        uint8_t plen = desc_xfer_pack_chunk(blob, N, 0, pay);
        CHECK(desc_xfer_accept(&ctx, pay, plen) == DESC_XFER_COMPLETE);
        CHECK(ctx.total == N);
        CHECK(memcmp(ctx.buf, blob, N) == 0);
    }

    /* ---- exact multiple of DATA_PER_CHUNK (no short final chunk) ---- */
    {
        const uint16_t N = DESC_XFER_DATA_PER_CHUNK * 3;
        uint8_t blob[N]; fill_blob(blob, N);
        desc_xfer_ctx_t ctx; desc_xfer_reset(&ctx);
        uint16_t nchunks = desc_xfer_chunk_count(N);
        CHECK(nchunks == 3);
        desc_xfer_result_t r = DESC_XFER_IN_PROGRESS;
        for (uint16_t i = 0; i < nchunks; i++) {
            uint8_t pay[DESC_XFER_PAYLOAD_MAX];
            uint8_t plen = desc_xfer_pack_chunk(blob, N, i, pay);
            r = desc_xfer_accept(&ctx, pay, plen);
        }
        CHECK(r == DESC_XFER_COMPLETE);
        CHECK(memcmp(ctx.buf, blob, N) == 0);
    }

    /* ---- restart-on-gap: a missing middle chunk forces RESTART ---- */
    {
        const uint16_t N = 100;
        uint8_t blob[N]; fill_blob(blob, N);
        desc_xfer_ctx_t ctx; desc_xfer_reset(&ctx);

        uint8_t pay[DESC_XFER_PAYLOAD_MAX]; uint8_t plen;
        // chunk 0 (start), then SKIP chunk 1 and feed chunk 2 -> RESTART.
        plen = desc_xfer_pack_chunk(blob, N, 0, pay);
        CHECK(desc_xfer_accept(&ctx, pay, plen) == DESC_XFER_IN_PROGRESS);
        plen = desc_xfer_pack_chunk(blob, N, 2, pay);
        CHECK(desc_xfer_accept(&ctx, pay, plen) == DESC_XFER_RESTART);

        // After a RESTART the receiver re-syncs on the next index-0 chunk and the
        // full sequence then completes cleanly.
        uint16_t nchunks = desc_xfer_chunk_count(N);
        desc_xfer_result_t r = DESC_XFER_IN_PROGRESS;
        for (uint16_t i = 0; i < nchunks; i++) {
            plen = desc_xfer_pack_chunk(blob, N, i, pay);
            r = desc_xfer_accept(&ctx, pay, plen);
        }
        CHECK(r == DESC_XFER_COMPLETE);
        CHECK(memcmp(ctx.buf, blob, N) == 0);
    }

    /* ---- mid-stream restart: a fresh index-0 with a DIFFERENT total resyncs ---- */
    {
        uint8_t a[60], b[80];
        fill_blob(a, 60); for (uint16_t i=0;i<80;i++) b[i]=(uint8_t)(i*3u+1u);
        desc_xfer_ctx_t ctx; desc_xfer_reset(&ctx);
        uint8_t pay[DESC_XFER_PAYLOAD_MAX]; uint8_t plen;

        // Start blob A, feed chunk 0 only, then a NEW transfer (blob B) starts
        // over at index 0 — receiver must adopt B's total and not mix A's bytes.
        plen = desc_xfer_pack_chunk(a, 60, 0, pay);
        CHECK(desc_xfer_accept(&ctx, pay, plen) == DESC_XFER_IN_PROGRESS);

        uint16_t nb = desc_xfer_chunk_count(80);
        desc_xfer_result_t r = DESC_XFER_IN_PROGRESS;
        for (uint16_t i = 0; i < nb; i++) {
            plen = desc_xfer_pack_chunk(b, 80, i, pay);
            r = desc_xfer_accept(&ctx, pay, plen);
        }
        CHECK(r == DESC_XFER_COMPLETE);
        CHECK(ctx.total == 80);
        CHECK(memcmp(ctx.buf, b, 80) == 0);
    }

    printf("desc_xfer_test: all passed\n");
    return 0;
}
