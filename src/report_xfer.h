#pragma once

// Parallel restart-on-gap SPI reassembler; see desc_xfer.h, which reassembles
// the descriptor blob with the same shape. This one differs: u8 index/total
// header + dev_ep/iface_protocol routing fields, a 64-byte report buffer, and
// exact-length fragment validation. Splice-relevant difference: consecutive HID
// reports carry different bytes, so the caller must reset on a frame-loss gap
// (desc_xfer's blob re-sends are byte-identical, so its analogous gap is benign).

#include <stdint.h>

#define REPORT_XFER_MAX_REPORT         64u
#define REPORT_XFER_HEADER_SIZE         4u
#define REPORT_XFER_PAYLOAD_SIZE       26u  // == SPI_FRAME_MAX_PAYLOAD (asserted in report_xfer.c)
#define REPORT_XFER_DATA_PER_FRAGMENT  (REPORT_XFER_PAYLOAD_SIZE - REPORT_XFER_HEADER_SIZE)

typedef enum {
    REPORT_XFER_IN_PROGRESS = 0,
    REPORT_XFER_COMPLETE,
    REPORT_XFER_RESTART,
} report_xfer_result_t;

typedef struct {
    uint8_t report[REPORT_XFER_MAX_REPORT];
    uint8_t total_len;
    uint8_t bytes_received;
    uint8_t expected_index;
    uint8_t dev_ep;
    uint8_t iface_protocol;
    uint8_t active;
} report_xfer_ctx_t;

// Each payload is [index,total,endpoint,protocol,data...], with 1..22 data bytes.
uint8_t report_xfer_fragment_count(uint16_t report_len);
uint8_t report_xfer_pack_fragment(const uint8_t *report, uint16_t report_len,
                                  uint8_t dev_ep, uint8_t iface_protocol,
                                  uint8_t fragment_index, uint8_t *out);

void report_xfer_reset(report_xfer_ctx_t *ctx);

// Invalid input resets the context and returns RESTART. A valid fragment 0 starts
// a new transfer even when another transfer was already active.
report_xfer_result_t report_xfer_accept(report_xfer_ctx_t *ctx,
                                        const uint8_t *payload,
                                        uint8_t payload_len);
