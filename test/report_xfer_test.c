// Host unit test for the 1..64-byte HID report transfer codec.
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "report_xfer.h"

#define CHECK(cond) do { if (!(cond)) { \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); assert(0); } } while (0)

static void fill_report(uint8_t *report, uint8_t length)
{
    uint8_t i;

    for (i = 0; i < length; i++)
        report[i] = (uint8_t)(i * 37u + length * 11u + 5u);
}

static void round_trip(uint8_t length, uint8_t ep, uint8_t protocol)
{
    uint8_t report[REPORT_XFER_MAX_REPORT];
    uint8_t payload[REPORT_XFER_PAYLOAD_SIZE];
    report_xfer_ctx_t ctx;
    uint8_t fragment_count;
    uint8_t fragment_index;

    fill_report(report, length);
    report_xfer_reset(&ctx);
    fragment_count = report_xfer_fragment_count(length);
    CHECK(fragment_count == (uint8_t)((length + REPORT_XFER_DATA_PER_FRAGMENT - 1u) /
                                      REPORT_XFER_DATA_PER_FRAGMENT));

    for (fragment_index = 0; fragment_index < fragment_count; fragment_index++) {
        uint16_t offset = (uint16_t)fragment_index * REPORT_XFER_DATA_PER_FRAGMENT;
        uint8_t expected_data = (uint8_t)(length - offset);
        uint8_t payload_len;
        report_xfer_result_t result;

        if (expected_data > REPORT_XFER_DATA_PER_FRAGMENT)
            expected_data = REPORT_XFER_DATA_PER_FRAGMENT;

        memset(payload, 0xA5, sizeof(payload));
        payload_len = report_xfer_pack_fragment(report, length, ep, protocol,
                                                fragment_index, payload);
        CHECK(payload_len == (uint8_t)(REPORT_XFER_HEADER_SIZE + expected_data));
        CHECK(payload_len <= REPORT_XFER_PAYLOAD_SIZE);
        CHECK(payload[0] == fragment_index);
        CHECK(payload[1] == length);
        CHECK(payload[2] == ep);
        CHECK(payload[3] == protocol);
        CHECK(memcmp(&payload[REPORT_XFER_HEADER_SIZE], &report[offset], expected_data) == 0);

        result = report_xfer_accept(&ctx, payload, payload_len);
        CHECK(ctx.total_len == length);
        CHECK(ctx.bytes_received == (uint8_t)(offset + expected_data));
        CHECK(ctx.expected_index == (uint8_t)(fragment_index + 1u));
        CHECK(ctx.dev_ep == ep);
        CHECK(ctx.iface_protocol == protocol);
        if ((uint8_t)(fragment_index + 1u) == fragment_count) {
            CHECK(result == REPORT_XFER_COMPLETE);
            CHECK(ctx.active == 0u);
        } else {
            CHECK(result == REPORT_XFER_IN_PROGRESS);
            CHECK(ctx.active == 1u);
        }
    }

    CHECK(memcmp(ctx.report, report, length) == 0);
}

static void complete_clean_round_trip(report_xfer_ctx_t *ctx,
                                      const uint8_t *report,
                                      uint8_t length,
                                      uint8_t ep,
                                      uint8_t protocol)
{
    uint8_t payload[REPORT_XFER_PAYLOAD_SIZE];
    uint8_t count = report_xfer_fragment_count(length);
    uint8_t index;

    for (index = 0; index < count; index++) {
        uint8_t payload_len = report_xfer_pack_fragment(report, length, ep, protocol,
                                                        index, payload);
        report_xfer_result_t result = report_xfer_accept(ctx, payload, payload_len);

        if ((uint8_t)(index + 1u) == count)
            CHECK(result == REPORT_XFER_COMPLETE);
        else
            CHECK(result == REPORT_XFER_IN_PROGRESS);
    }
    CHECK(ctx->active == 0u);
    CHECK(ctx->total_len == length);
    CHECK(ctx->dev_ep == ep);
    CHECK(ctx->iface_protocol == protocol);
    CHECK(memcmp(ctx->report, report, length) == 0);
}

typedef enum {
    MALFORMED_MISSING_INDEX = 0,
    MALFORMED_REPEATED_NONZERO_INDEX,
    MALFORMED_OUT_OF_ORDER_INDEX,
    MALFORMED_INCONSISTENT_TOTAL,
    MALFORMED_INCONSISTENT_ENDPOINT,
    MALFORMED_INCONSISTENT_PROTOCOL,
    MALFORMED_SHORT_NONFINAL_DATA,
    MALFORMED_LONG_FINAL_DATA,
    MALFORMED_TOTAL_ZERO,
    MALFORMED_TOTAL_TOO_LARGE,
    MALFORMED_ENDPOINT_ZERO,
    MALFORMED_ENDPOINT_TOO_LARGE,
    MALFORMED_TOO_SHORT,
} malformed_kind_t;

static void reject_then_recover(malformed_kind_t kind)
{
    uint8_t report[REPORT_XFER_MAX_REPORT];
    uint8_t payload[REPORT_XFER_PAYLOAD_SIZE + 1u];
    report_xfer_ctx_t ctx;
    uint8_t length = REPORT_XFER_MAX_REPORT;
    const uint8_t ep = 7u;
    const uint8_t protocol = 2u;
    uint8_t payload_len;

    fill_report(report, length);
    report_xfer_reset(&ctx);

    payload_len = report_xfer_pack_fragment(report, length, ep, protocol, 0u, payload);
    CHECK(report_xfer_accept(&ctx, payload, payload_len) == REPORT_XFER_IN_PROGRESS);
    CHECK(ctx.active == 1u);

    if (kind == MALFORMED_MISSING_INDEX) {
        report_xfer_reset(&ctx);
        payload_len = report_xfer_pack_fragment(report, length, ep, protocol, 1u, payload);
    } else if (kind == MALFORMED_REPEATED_NONZERO_INDEX) {
        payload_len = report_xfer_pack_fragment(report, length, ep, protocol, 1u, payload);
        CHECK(report_xfer_accept(&ctx, payload, payload_len) == REPORT_XFER_IN_PROGRESS);
        payload_len = report_xfer_pack_fragment(report, length, ep, protocol, 1u, payload);
    } else if (kind == MALFORMED_OUT_OF_ORDER_INDEX) {
        payload_len = report_xfer_pack_fragment(report, length, ep, protocol, 2u, payload);
    } else if (kind == MALFORMED_LONG_FINAL_DATA) {
        length = 23u;
        report_xfer_reset(&ctx);
        payload_len = report_xfer_pack_fragment(report, length, ep, protocol, 0u, payload);
        CHECK(report_xfer_accept(&ctx, payload, payload_len) == REPORT_XFER_IN_PROGRESS);
        payload_len = report_xfer_pack_fragment(report, length, ep, protocol, 1u, payload);
        payload[payload_len] = 0x5Au;
        payload_len++;
    } else if (kind == MALFORMED_TOTAL_ZERO ||
               kind == MALFORMED_TOTAL_TOO_LARGE ||
               kind == MALFORMED_ENDPOINT_ZERO ||
               kind == MALFORMED_ENDPOINT_TOO_LARGE) {
        payload_len = report_xfer_pack_fragment(report, length, ep, protocol, 0u, payload);
        if (kind == MALFORMED_TOTAL_ZERO)
            payload[1] = 0u;
        else if (kind == MALFORMED_TOTAL_TOO_LARGE)
            payload[1] = REPORT_XFER_MAX_REPORT + 1u;
        else if (kind == MALFORMED_ENDPOINT_ZERO)
            payload[2] = 0u;
        else
            payload[2] = 16u;
    } else if (kind == MALFORMED_TOO_SHORT) {
        payload[0] = 1u;
        payload[1] = length;
        payload[2] = ep;
        payload[3] = protocol;
        payload_len = REPORT_XFER_HEADER_SIZE;
    } else {
        payload_len = report_xfer_pack_fragment(report, length, ep, protocol, 1u, payload);
        if (kind == MALFORMED_INCONSISTENT_TOTAL)
            payload[1]--;
        else if (kind == MALFORMED_INCONSISTENT_ENDPOINT)
            payload[2]++;
        else if (kind == MALFORMED_INCONSISTENT_PROTOCOL)
            payload[3]++;
        else if (kind == MALFORMED_SHORT_NONFINAL_DATA)
            payload_len--;
    }

    CHECK(report_xfer_accept(&ctx, payload, payload_len) == REPORT_XFER_RESTART);
    CHECK(ctx.active == 0u);
    complete_clean_round_trip(&ctx, report, length, ep, protocol);
}

static void test_boundary_cases(void)
{
    static const struct {
        uint8_t len;
        uint8_t fragments;
        uint8_t final_payload_len;
    } cases[] = {
        { 1u,  1u,  5u }, { 22u, 1u, 26u }, { 23u, 2u,  5u },
        { 44u, 2u, 26u }, { 45u, 3u,  5u }, { 64u, 3u, 24u },
    };
    uint8_t report[REPORT_XFER_MAX_REPORT];
    uint8_t payload[REPORT_XFER_PAYLOAD_SIZE];
    size_t case_index;

    for (case_index = 0; case_index < sizeof(cases) / sizeof(cases[0]); case_index++) {
        uint8_t final_index = (uint8_t)(cases[case_index].fragments - 1u);
        uint8_t payload_len;

        fill_report(report, cases[case_index].len);
        CHECK(report_xfer_fragment_count(cases[case_index].len) == cases[case_index].fragments);
        payload_len = report_xfer_pack_fragment(report, cases[case_index].len, 15u, 0xA7u,
                                                final_index, payload);
        CHECK(payload_len == cases[case_index].final_payload_len);
        round_trip(cases[case_index].len, 15u, 0xA7u);
    }
}

static void test_all_legal_lengths_and_endpoints(void)
{
    uint8_t length;

    for (length = 1u; length <= REPORT_XFER_MAX_REPORT; length++) {
        uint8_t ep;

        for (ep = 1u; ep <= 15u; ep++)
            round_trip(length, ep, (uint8_t)(length * 3u + ep));
    }
}

static void test_sender_rejections(void)
{
    uint8_t report[REPORT_XFER_MAX_REPORT];
    uint8_t payload[REPORT_XFER_PAYLOAD_SIZE];

    fill_report(report, REPORT_XFER_MAX_REPORT);
    CHECK(report_xfer_fragment_count(0u) == 0u);
    CHECK(report_xfer_fragment_count(65u) == 0u);
    CHECK(report_xfer_fragment_count(256u) == 0u);
    CHECK(report_xfer_pack_fragment(report, 0u, 1u, 0u, 0u, payload) == 0u);
    CHECK(report_xfer_pack_fragment(report, 65u, 1u, 0u, 0u, payload) == 0u);
    CHECK(report_xfer_pack_fragment(report, 256u, 1u, 0u, 0u, payload) == 0u);
    CHECK(report_xfer_pack_fragment(report, 1u, 0u, 0u, 0u, payload) == 0u);
    CHECK(report_xfer_pack_fragment(report, 1u, 16u, 0u, 0u, payload) == 0u);
    CHECK(report_xfer_pack_fragment(report, 64u, 1u, 0u,
                                    report_xfer_fragment_count(64u), payload) == 0u);
}

static void test_receiver_rejections(void)
{
    malformed_kind_t kind;

    for (kind = MALFORMED_MISSING_INDEX; kind <= MALFORMED_TOO_SHORT; kind++)
        reject_then_recover(kind);
}

static void test_fragment_zero_restarts_active_transfer(void)
{
    uint8_t first_report[REPORT_XFER_MAX_REPORT];
    uint8_t second_report[REPORT_XFER_MAX_REPORT];
    uint8_t payload[REPORT_XFER_PAYLOAD_SIZE];
    report_xfer_ctx_t ctx;
    uint8_t payload_len;

    fill_report(first_report, 64u);
    fill_report(second_report, 23u);
    second_report[0] ^= 0xFFu;
    report_xfer_reset(&ctx);

    payload_len = report_xfer_pack_fragment(first_report, 64u, 1u, 2u, 0u, payload);
    CHECK(report_xfer_accept(&ctx, payload, payload_len) == REPORT_XFER_IN_PROGRESS);

    payload_len = report_xfer_pack_fragment(second_report, 23u, 15u, 9u, 0u, payload);
    CHECK(report_xfer_accept(&ctx, payload, payload_len) == REPORT_XFER_IN_PROGRESS);
    CHECK(ctx.total_len == 23u);
    CHECK(ctx.bytes_received == 22u);
    CHECK(ctx.expected_index == 1u);
    CHECK(ctx.dev_ep == 15u);
    CHECK(ctx.iface_protocol == 9u);

    payload_len = report_xfer_pack_fragment(second_report, 23u, 15u, 9u, 1u, payload);
    CHECK(report_xfer_accept(&ctx, payload, payload_len) == REPORT_XFER_COMPLETE);
    CHECK(memcmp(ctx.report, second_report, 23u) == 0);
}

// Two same-endpoint 44-byte reports (2 fragments each). If report A's tail
// fragment and report B's fragment 0 are both lost, the codec ALONE accepts B's
// fragment 1 onto A's stale buffer and reports COMPLETE — a spliced report. This
// documents why the caller must reset on a frame-loss gap (see the reset in
// two_board.c's receive loop).
static void test_cross_report_splice_documents_codec_gap(void)
{
    uint8_t report_a[REPORT_XFER_MAX_REPORT];
    uint8_t report_b[REPORT_XFER_MAX_REPORT];
    uint8_t payload[REPORT_XFER_PAYLOAD_SIZE];
    report_xfer_ctx_t ctx;
    const uint8_t ep = 1u, protocol = 2u;
    uint8_t payload_len, i;

    fill_report(report_a, 44u);
    for (i = 0; i < 44u; i++)
        report_b[i] = (uint8_t)~report_a[i];

    report_xfer_reset(&ctx);
    payload_len = report_xfer_pack_fragment(report_a, 44u, ep, protocol, 0u, payload);
    CHECK(report_xfer_accept(&ctx, payload, payload_len) == REPORT_XFER_IN_PROGRESS);
    CHECK(ctx.active == 1u && ctx.expected_index == 1u);

    /* report_a fragment 1 lost, report_b fragment 0 lost — report_b fragment 1
     * arrives next; index==expected_index and total/ep/proto all match. */
    payload_len = report_xfer_pack_fragment(report_b, 44u, ep, protocol, 1u, payload);
    CHECK(report_xfer_accept(&ctx, payload, payload_len) == REPORT_XFER_COMPLETE);

    /* Frankenreport: report_a head + report_b tail, matching neither full report. */
    CHECK(memcmp(ctx.report, report_a, 22u) == 0);
    CHECK(memcmp(ctx.report + 22, report_b + 22, 22u) == 0);
    CHECK(memcmp(ctx.report, report_a, 44u) != 0);
    CHECK(memcmp(ctx.report, report_b, 44u) != 0);
}

// Same scenario, but the caller resets the context on the detected frame-loss gap
// (what two_board.c now does). The splice survivor is rejected, and a clean
// resend of report_b recovers.
static void test_gap_reset_prevents_splice(void)
{
    uint8_t report_a[REPORT_XFER_MAX_REPORT];
    uint8_t report_b[REPORT_XFER_MAX_REPORT];
    uint8_t payload[REPORT_XFER_PAYLOAD_SIZE];
    report_xfer_ctx_t ctx;
    const uint8_t ep = 1u, protocol = 2u;
    uint8_t payload_len, i;

    fill_report(report_a, 44u);
    for (i = 0; i < 44u; i++)
        report_b[i] = (uint8_t)~report_a[i];

    report_xfer_reset(&ctx);
    payload_len = report_xfer_pack_fragment(report_a, 44u, ep, protocol, 0u, payload);
    CHECK(report_xfer_accept(&ctx, payload, payload_len) == REPORT_XFER_IN_PROGRESS);

    report_xfer_reset(&ctx);   /* caller-driven reset on the seq gap */

    payload_len = report_xfer_pack_fragment(report_b, 44u, ep, protocol, 1u, payload);
    CHECK(report_xfer_accept(&ctx, payload, payload_len) == REPORT_XFER_RESTART);
    CHECK(ctx.active == 0u);

    complete_clean_round_trip(&ctx, report_b, 44u, ep, protocol);
}

int main(void)
{
    test_boundary_cases();
    test_all_legal_lengths_and_endpoints();
    test_sender_rejections();
    test_receiver_rejections();
    test_fragment_zero_restarts_active_transfer();
    test_cross_report_splice_documents_codec_gap();
    test_gap_reset_prevents_splice();
    printf("report_xfer_test: all passed\n");
    return 0;
}
