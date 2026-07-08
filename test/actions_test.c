// Host-native unit test for the act_click stuck-button fix + unified tick
// scheduler (ported from Hurra-v2). Compiles actions.c with system gcc; stubs
// the kmbox_* injection sinks and a controllable millis() so the click
// sequence can be stepped deterministically across simulated time.
//
//   cc -std=c11 -O2 -Isrc -o /tmp/actions_test test/actions_test.c src/actions.c -lm
//
// Model, per the v3 dual-core architecture: act_click() (command parser) and
// act_click_tick() (poll loop) both run on V3F, in kmbox_cmd_poll(), on the same
// DTCM-local g_click_sched/g_buttons. There is no cross-core sharing of that
// state (V5F links actions.c only for the physical-mask half and never calls
// these), so this single-threaded host model faithfully mirrors the target.

#include <stdio.h>
#include <stdint.h>
#include "actions.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    printf("FAIL: %s\n", msg); failures++; } } while (0)

// ── stubbed environment ──────────────────────────────────────────────────────
static uint32_t g_ms;
uint32_t millis(void) { return g_ms; }

// Injection sink: record each injected buttons byte so we can reconstruct the
// press/release edge sequence. Stub names match the kmbox.h shim (kmbox_cmd_*).
#define LOG_MAX 256
static uint8_t g_btn_log[LOG_MAX];
static int     g_log_n;
void kmbox_cmd_inject_mouse(int16_t dx, int16_t dy, uint8_t buttons, int8_t wheel) {
    (void)dx; (void)dy; (void)wheel;
    if (g_log_n < LOG_MAX) g_btn_log[g_log_n++] = buttons;
}
void kmbox_cmd_inject_keyboard(uint8_t m, const uint8_t k[6]) { (void)m; (void)k; }
void kmbox_cmd_schedule_click_release(uint8_t b, uint32_t d) { (void)b; (void)d; }
void kmbox_cmd_schedule_kb_release(uint8_t k, uint32_t d) { (void)k; (void)d; }

static void reset_log(void) { g_log_n = 0; }

// Count rising (press) and falling (release) edges of a given button bit across
// the recorded injection stream, starting from an all-released state.
static void count_edges(uint8_t bit, int *presses, int *releases) {
    int p = 0, r = 0;
    uint8_t prev = 0;
    for (int i = 0; i < g_log_n; i++) {
        uint8_t was = prev & bit, now = g_btn_log[i] & bit;
        if (!was && now) p++;
        if (was && !now) r++;
        prev = g_btn_log[i];
    }
    *presses = p; *releases = r;
}

int main(void) {
    // (A) stuck-button fix: starting a click on button B while button A is still
    //     pressed from a prior click sequence must release A first, so A can't
    //     stay stuck down. Bit order: button 1 = 0x01, button 2 = 0x02.
    act_init();
    reset_log();
    g_ms = 0;
    act_click(1, 1, 100);            // press A; release scheduled far in the future
    CHECK(g_buttons == 0x01, "A: button A pressed by first click");
    act_click(2, 1, 100);            // start B before A's scheduled release fires
    CHECK(g_buttons == 0x02, "A: starting click on B released stuck A and pressed B");
    CHECK((g_buttons & 0x01) == 0,  "A: button A is not left stuck down");
    // Drive time forward so B's own release fires; nothing should remain pressed.
    for (int i = 0; i < 4; i++) { g_ms += 100; act_click_tick(); }
    CHECK(g_buttons == 0x00, "A: button B releases via tick, nothing stuck");

    // (B) multi-click count=N yields N full press/release cycles as the tick is
    //     pumped across simulated time.
    act_init();
    reset_log();
    g_ms = 1000;
    act_click(1, 3, 10);            // triple-click on button 1
    for (int i = 0; i < 12; i++) { g_ms += 10; act_click_tick(); }
    int presses, releases;
    count_edges(0x01, &presses, &releases);
    CHECK(presses == 3,  "B: count=3 produced exactly 3 presses");
    CHECK(releases == 3, "B: count=3 produced exactly 3 releases");
    CHECK(g_buttons == 0x00, "B: sequence ends with the button released");
    // A further tick after completion must be an inert no-op (sched cleared).
    int n_before = g_log_n;
    g_ms += 100; act_click_tick();
    CHECK(g_log_n == n_before, "B: tick after completion injects nothing");

    // (C) count==0 or an invalid button is a no-op (no press, no injection).
    act_init();
    reset_log();
    g_ms = 0;
    act_click(1, 0, 0);            // count==0
    CHECK(g_buttons == 0 && g_log_n == 0, "C: count==0 is a no-op");
    act_click(0, 1, 0);            // button 0 -> mask==0
    CHECK(g_buttons == 0 && g_log_n == 0, "C: invalid button 0 is a no-op");
    act_click(99, 1, 0);          // out-of-range button -> mask==0
    CHECK(g_buttons == 0 && g_log_n == 0, "C: out-of-range button is a no-op");
    // A tick with nothing scheduled is also inert.
    act_click_tick();
    CHECK(g_log_n == 0, "C: tick with no schedule injects nothing");

    // (D) act_kb_mask_get getter round-trips the stored mode.
    act_init();
    act_kb_mask(0x04, 2);          // mask HID 'a' with mode 2
    CHECK(act_kb_mask_get(0x04) == 2, "D: kb_mask_get returns the stored mode");
    CHECK(act_kb_mask_get(0x05) == 0, "D: kb_mask_get returns 0 for an unmasked key");
    act_kb_mask(0x04, 0);          // unmask
    CHECK(act_kb_mask_get(0x04) == 0, "D: kb_mask_get returns 0 after unmask");

    // (E) wrap-safety: a click scheduled just before the 32-bit millis() wrap
    //     must still release after millis() rolls over. A naive `now < next_at`
    //     leaves the button stuck for ~49 days; the wrap-safe subtraction fires.
    //     next_at = 0xFFFFFFE0 + 0x10 = 0xFFFFFFF0, then millis() wraps to 0x05.
    act_init();
    reset_log();
    g_ms = 0xFFFFFFE0u;
    act_click(1, 1, 0x10);         // press button 1; release scheduled at 0xFFFFFFF0
    CHECK(g_buttons == 0x01, "E: button pressed just before millis wrap");
    g_ms = 0x00000005u;            // millis() wrapped; deadline 0xFFFFFFF0 is now past
    act_click_tick();
    CHECK(g_buttons == 0x00, "E: click releases after millis wrap (not left stuck)");

    // (F) act_tick() (the shared pump both command loops call) must step the click
    //     sequence, not just motion. A regression that dropped act_click_tick()
    //     from act_tick() would strand clicks in every pump — guard it here.
    act_init();
    reset_log();
    g_ms = 5000;
    act_click(1, 1, 10);           // press; release scheduled at 5010
    CHECK(g_buttons == 0x01, "F: click pressed");
    g_ms += 20; act_tick();        // act_tick (not act_click_tick) must release it
    CHECK(g_buttons == 0x00, "F: act_tick() pumps the click release");

    if (failures == 0) printf("\nALL PASSED\n");
    else               printf("\n%d FAILURE(S)\n", failures);
    return failures ? 1 : 0;
}
