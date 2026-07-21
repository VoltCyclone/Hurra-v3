#include "report_xfer.h"
#include "spi_frame.h"   /* SPI_FRAME_MAX_PAYLOAD — coupling target */

#include <string.h>

/* A fragment payload must fit exactly in one SPI frame's payload region; if the
 * slot size ever drifts, spi_frame_pack would silently reject every fragment
 * (ERR_LEN) at runtime. Catch the drift at compile time instead. */
_Static_assert(REPORT_XFER_PAYLOAD_SIZE == SPI_FRAME_MAX_PAYLOAD,
    "report_xfer payload must equal the SPI frame payload capacity");

static report_xfer_result_t report_xfer_reject(report_xfer_ctx_t *ctx)
{
    report_xfer_reset(ctx);
    return REPORT_XFER_RESTART;
}

uint8_t report_xfer_fragment_count(uint16_t report_len)
{
    if (report_len == 0u || report_len > REPORT_XFER_MAX_REPORT)
        return 0u;

    return (uint8_t)((report_len + REPORT_XFER_DATA_PER_FRAGMENT - 1u) /
                     REPORT_XFER_DATA_PER_FRAGMENT);
}

uint8_t report_xfer_pack_fragment(const uint8_t *report, uint16_t report_len,
                                  uint8_t dev_ep, uint8_t iface_protocol,
                                  uint8_t fragment_index, uint8_t *out)
{
    uint8_t fragment_count = report_xfer_fragment_count(report_len);
    uint16_t offset;
    uint16_t data_len;

    if (fragment_count == 0u || dev_ep == 0u || dev_ep > 15u ||
        fragment_index >= fragment_count)
        return 0u;

    offset = (uint16_t)fragment_index * REPORT_XFER_DATA_PER_FRAGMENT;
    data_len = report_len - offset;
    if (data_len > REPORT_XFER_DATA_PER_FRAGMENT)
        data_len = REPORT_XFER_DATA_PER_FRAGMENT;

    out[0] = fragment_index;
    out[1] = (uint8_t)report_len;
    out[2] = dev_ep;
    out[3] = iface_protocol;
    memcpy(&out[REPORT_XFER_HEADER_SIZE], &report[offset], data_len);
    return (uint8_t)(REPORT_XFER_HEADER_SIZE + data_len);
}

void report_xfer_reset(report_xfer_ctx_t *ctx)
{
    ctx->total_len = 0u;
    ctx->bytes_received = 0u;
    ctx->expected_index = 0u;
    ctx->dev_ep = 0u;
    ctx->iface_protocol = 0u;
    ctx->active = 0u;
}

report_xfer_result_t report_xfer_accept(report_xfer_ctx_t *ctx,
                                        const uint8_t *payload,
                                        uint8_t payload_len)
{
    uint8_t index;
    uint8_t total_len;
    uint8_t dev_ep;
    uint8_t iface_protocol;
    uint8_t data_len;
    uint8_t expected_data;
    uint16_t offset;

    if (payload_len < REPORT_XFER_HEADER_SIZE + 1u)
        return report_xfer_reject(ctx);

    index = payload[0];
    total_len = payload[1];
    dev_ep = payload[2];
    iface_protocol = payload[3];
    data_len = (uint8_t)(payload_len - REPORT_XFER_HEADER_SIZE);

    if (total_len == 0u || total_len > REPORT_XFER_MAX_REPORT ||
        dev_ep == 0u || dev_ep > 15u)
        return report_xfer_reject(ctx);

    offset = (uint16_t)index * REPORT_XFER_DATA_PER_FRAGMENT;
    if (offset >= total_len)
        return report_xfer_reject(ctx);

    expected_data = (uint8_t)(total_len - offset);
    if (expected_data > REPORT_XFER_DATA_PER_FRAGMENT)
        expected_data = REPORT_XFER_DATA_PER_FRAGMENT;
    if (data_len != expected_data)
        return report_xfer_reject(ctx);

    if (index == 0u) {
        ctx->total_len = total_len;
        ctx->bytes_received = 0u;
        ctx->expected_index = 0u;
        ctx->dev_ep = dev_ep;
        ctx->iface_protocol = iface_protocol;
        ctx->active = 1u;
    } else if (ctx->active == 0u || index != ctx->expected_index ||
               total_len != ctx->total_len || dev_ep != ctx->dev_ep ||
               iface_protocol != ctx->iface_protocol) {
        return report_xfer_reject(ctx);
    }

    memcpy(&ctx->report[offset], &payload[REPORT_XFER_HEADER_SIZE], data_len);
    ctx->bytes_received = (uint8_t)(offset + data_len);
    ctx->expected_index = (uint8_t)(index + 1u);

    if (ctx->bytes_received == ctx->total_len) {
        ctx->active = 0u;
        return REPORT_XFER_COMPLETE;
    }

    return REPORT_XFER_IN_PROGRESS;
}
