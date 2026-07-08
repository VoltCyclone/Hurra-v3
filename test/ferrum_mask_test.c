// Host-native unit test for the Ferrum km.mask() read-getter wiring.
// Drives the real Ferrum public API byte-by-byte (ferrum_feed_byte), captures
// emitted TX bytes via a redirected ferrum_tx_fn, and asserts that a
// km.mask(<key>) read reflects a prior km.mask(<key>, <mode>) write — i.e. the
// read path calls act_kb_mask_get() rather than the old always-0 stub.
//
//   cc -std=c11 -O2 -Wall -Isrc -o /tmp/ferrum_mask_test \
//        test/ferrum_mask_test.c src/ferrum.c src/actions.c -lm
//
// ferrum.c + actions.c both run on V3F in the command path; this single-threaded
// host model faithfully mirrors the target (no cross-core sharing of this state).

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "ferrum.h"
#include "actions.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    printf("FAIL: %s\n", msg); failures++; } } while (0)

// ── stubbed environment ──────────────────────────────────────────────────────
// millis(): constant is fine — keeps ferrum's idle-gap reset (25ms) from firing
// between bytes of a line, and catch_xy is inactive after ferrum_init().
uint32_t millis(void) { return 0; }

// Injection / link sinks pulled by ferrum.c (via the kmbox.h shim) and actions.c.
// None affect the mask read path; they only need to link.
void kmbox_cmd_inject_mouse(int16_t dx, int16_t dy, uint8_t b, int8_t w) {
    (void)dx; (void)dy; (void)b; (void)w;
}
void kmbox_cmd_inject_keyboard(uint8_t m, const uint8_t k[6]) { (void)m; (void)k; }
void kmbox_cmd_schedule_click_release(uint8_t b, uint32_t d) { (void)b; (void)d; }
void kmbox_cmd_schedule_kb_release(uint8_t k, uint32_t d) { (void)k; (void)d; }
void kmbox_cmd_set_baud(uint32_t baud) { (void)baud; }

// ── TX capture ───────────────────────────────────────────────────────────────
static uint8_t g_tx[256];
static int     g_tx_n;
static void tx_capture(const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len && g_tx_n < (int)sizeof(g_tx); i++)
        g_tx[g_tx_n++] = data[i];
}
static void tx_reset(void) { g_tx_n = 0; }

// Feed a whole ASCII command string one byte at a time, exactly as the UART ISR
// would deliver it.
static void feed(const char *s) {
    while (*s) ferrum_feed_byte((uint8_t)*s++);
}

// A bool reply is exactly "X\r\n" (3 bytes) where X is '0' or '1'.
static int reply_is(char digit) {
    return g_tx_n == 3 && g_tx[0] == (uint8_t)digit &&
           g_tx[1] == '\r' && g_tx[2] == '\n';
}

int main(void) {
    ferrum_init();
    ferrum_set_tx(tx_capture);
    act_init();       // zero the masked-key table
    act_kb_init();    // flush a zero HID keyboard report (mirrors km.init())

    // (A) Baseline: reading an unmasked key emits "0\r\n".
    tx_reset();
    feed("km.mask(4)\r\n");
    CHECK(reply_is('0'), "A: unmasked key reads 0");

    // (B) Write path: km.mask(4, 1) masks HID key 0x04 with mode 1 and emits
    //     nothing (write commands reply nothing).
    tx_reset();
    feed("km.mask(4, 1)\r\n");
    CHECK(g_tx_n == 0, "B: write km.mask(k,mode) emits no reply");
    CHECK(act_kb_mask_get(4) == 1, "B: write stored mode 1 in the mask table");

    // (C) THE FIX: reading the now-masked key must emit "1\r\n". Against the old
    //     stub (emit_bool(0)) this reads "0" and FAILS.
    tx_reset();
    feed("km.mask(4)\r\n");
    CHECK(reply_is('1'), "C: masked key reads 1 (getter wired)");

    // (D) A different, still-unmasked key reads "0\r\n".
    tx_reset();
    feed("km.mask(5)\r\n");
    CHECK(reply_is('0'), "D: other unmasked key still reads 0");

    // (E) Unmask via km.mask(4, 0), then the read returns to "0\r\n".
    tx_reset();
    feed("km.mask(4, 0)\r\n");
    CHECK(g_tx_n == 0, "E: unmask write emits no reply");
    tx_reset();
    feed("km.mask(4)\r\n");
    CHECK(reply_is('0'), "E: read after unmask returns 0");

    // (F) Nonzero non-1 modes still read as masked ("1\r\n"), since the read is
    //     emit_bool(get != 0), not the raw mode.
    tx_reset();
    feed("km.mask(7, 2)\r\n");
    CHECK(act_kb_mask_get(7) == 2, "F: mode 2 stored");
    tx_reset();
    feed("km.mask(7)\r\n");
    CHECK(reply_is('1'), "F: mode-2 key reads 1 (bool of nonzero)");

    if (failures == 0) printf("\nALL PASSED\n");
    else               printf("\n%d FAILURE(S)\n", failures);
    return failures ? 1 : 0;
}
