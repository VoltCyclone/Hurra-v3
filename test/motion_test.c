// Host-native unit test for the Feature A motion program (act_motion_*).
// Compiles actions.c with system gcc; stubs the kmbox_* injection sinks and a
// controllable millis() so the trajectory can be stepped deterministically.
//
//   cc -std=c11 -O2 -Isrc -o /tmp/motion_test test/motion_test.c src/actions.c -lm

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "actions.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    printf("FAIL: %s\n", msg); failures++; } } while (0)

// ── stubbed environment ──────────────────────────────────────────────────────
static uint32_t g_ms;
uint32_t millis(void) { return g_ms; }

// Injection sink: accumulate the deltas the motion program emits.
static long g_sum_x, g_sum_y;
static int  g_emits;
// Stub names match the kmbox.h shim: actions.c calls kmbox_cmd_* (the v3 ICC
// router), so the host-test sinks are named kmbox_cmd_* to satisfy the link.
void kmbox_cmd_inject_mouse(int16_t dx, int16_t dy, uint8_t buttons, int8_t wheel) {
    (void)buttons; (void)wheel;
    g_sum_x += dx; g_sum_y += dy;
    if (dx || dy) g_emits++;
}
void kmbox_cmd_inject_keyboard(uint8_t m, const uint8_t k[6]) { (void)m; (void)k; }
void kmbox_cmd_schedule_click_release(uint8_t b, uint32_t d) { (void)b; (void)d; }
void kmbox_cmd_schedule_kb_release(uint8_t k, uint32_t d) { (void)k; (void)d; }

static void reset_sink(void) { g_sum_x = g_sum_y = 0; g_emits = 0; }

/* ── Plan 5 Task 4: fake motion source (hoisted to file scope for C11 portability) ── */
static int32_t f_dx, f_dy;
static int     f_steps;
static int     f_cancelled;

static void fake_begin(int32_t dx, int32_t dy, uint16_t dur, int bz,
                       int32_t x1, int32_t y1, int32_t x2, int32_t y2) {
    (void)dur; (void)bz; (void)x1; (void)y1; (void)x2; (void)y2;
    f_dx = dx; f_dy = dy; f_steps = 3;
}
static int fake_tick(int16_t *odx, int16_t *ody) {
    if (f_steps <= 0) return 0;
    *odx = (int16_t)(f_dx / 3); *ody = (int16_t)(f_dy / 3);
    f_steps--; return 1;
}
static void fake_cancel(void) { f_cancelled = 1; f_steps = 0; }

/* ── v3 Task 5: stream-filter fake (hoisted to file scope for C11 portability) ── */
static void sf_add_one(int16_t in_dx, int16_t in_dy, int16_t *ox, int16_t *oy) {
    *ox = (int16_t)(in_dx + 1); *oy = in_dy;
}

// Step a program to completion, advancing time `step_ms` per tick.
static void run_program(uint32_t dur_ms, uint32_t step_ms) {
    // tick once at t=0, then advance until past dur, plus one final tick at end.
    act_motion_tick();
    for (uint32_t t = 0; t <= dur_ms + step_ms; t += step_ms) {
        g_ms += step_ms;
        act_motion_tick();
    }
}

int main(void) {
    act_init();

    // (A) automove conservation + exact endpoint, fine stepping (1 ms ticks).
    reset_sink();
    g_ms = 1000;
    act_motion_move_dur(200, -120, 500);
    run_program(500, 1);
    CHECK(g_sum_x == 200,  "automove: total X lands exactly on endpoint");
    CHECK(g_sum_y == -120, "automove: total Y lands exactly on endpoint");
    CHECK(g_emits > 10,    "automove: motion is spread across many frames, not one jump");

    // (B) automove with coarse, irregular stepping still hits the endpoint
    //     (position-based emission is cadence-independent).
    reset_sink();
    g_ms = 5000;
    act_motion_move_dur(-77, 333, 200);
    act_motion_tick();
    g_ms += 37; act_motion_tick();
    g_ms += 5;  act_motion_tick();
    g_ms += 140; act_motion_tick();
    g_ms += 90;  act_motion_tick();   // now past dur → final tick clamps to end
    CHECK(g_sum_x == -77,  "automove coarse: X exact despite irregular ticks");
    CHECK(g_sum_y == 333,  "automove coarse: Y exact despite irregular ticks");

    // (C) bezier conservation + exact endpoint.
    reset_sink();
    g_ms = 0;
    act_motion_bezier(300, 0, 400, 100, 200, 200, -200);
    run_program(400, 1);
    CHECK(g_sum_x == 300, "bezier: total X lands on endpoint");
    CHECK(g_sum_y == 0,   "bezier: total Y returns to endpoint (bowed path nets 0)");

    // (D) bezier with bowed control points actually leaves the straight line:
    //     a curve bowing in +Y must produce some +Y excursion mid-flight even
    //     though the endpoint Y is 0.
    reset_sink();
    g_ms = 0;
    act_motion_bezier(300, 0, 400, 0, 250, 300, 250);  // both control pts +Y
    int saw_positive_y = 0;
    long runy = 0;
    act_motion_tick();
    for (uint32_t t = 0; t <= 401; t += 1) {
        long before = g_sum_y;
        g_ms += 1; act_motion_tick();
        runy += (g_sum_y - before);
        if (runy > 5) saw_positive_y = 1;   // cumulative Y went meaningfully +
    }
    CHECK(saw_positive_y, "bezier: bowed control points curve off the straight line");
    CHECK(g_sum_y == 0,   "bezier: curved path still ends at endpoint Y=0");

    // (E) last-writer-wins: a plain act_move mid-flight cancels the program.
    reset_sink();
    g_ms = 0;
    act_motion_move_dur(1000, 0, 1000);
    act_motion_tick();
    g_ms += 100; act_motion_tick();        // partial progress
    long after_partial = g_sum_x;
    act_move(7, 0);                        // manual override
    long after_move = g_sum_x;
    g_ms += 500; act_motion_tick();        // should be a no-op now
    CHECK(after_move == after_partial + 7, "cancel: manual move applied once");
    CHECK(g_sum_x == after_move,           "cancel: program produced no further motion");

    // (F) dur_ms == 0 is an immediate full move.
    reset_sink();
    g_ms = 0;
    act_motion_move_dur(42, -9, 0);
    CHECK(g_sum_x == 42 && g_sum_y == -9, "dur=0: immediate move delivers full delta");
    act_motion_tick();                     // nothing in flight
    CHECK(g_sum_x == 42 && g_sum_y == -9, "dur=0: no trailing motion");

    /* ── Plan 5 Task 4: registered motion-source hook ── */
    {
        static const act_motion_source_t fake = {
            .begin = fake_begin, .tick = fake_tick, .cancel = fake_cancel };

        f_dx = f_dy = f_steps = f_cancelled = 0;
        act_init();
        long base_x = g_sum_x;
        act_motion_set_source(&fake);
        act_motion_move_dur(90, 30, 100);     /* routed to fake, not analytic */
        for (int i = 0; i < 5; i++) act_motion_tick();
        CHECK(g_sum_x - base_x == 90, "hook: total X emitted via fake source");
        CHECK(f_steps == 0, "hook: fake source ran to completion");

        /* act_move cancels an in-flight gesture source. */
        f_cancelled = 0;
        act_motion_move_dur(300, 0, 1000);
        act_motion_tick();
        act_move(5, 0);                       /* last-writer-wins */
        CHECK(f_cancelled == 1, "hook: act_move cancels the gesture source");

        act_motion_set_source(NULL);          /* restore analytic for any later tests */
    }

    /* ── v3 Task 5: stream-filter hook on act_move ── */
    {
        /* fake filter: add +1 to dx so we can see it ran */
        static const act_stream_filter_t fake = { .apply = sf_add_one };
        act_init();
        long before = g_sum_x;
        act_set_stream_filter(&fake);
        act_move(10, 0);
        CHECK(g_sum_x - before == 11, "stream filter applied to act_move (10 -> 11)");

        act_set_stream_filter(NULL);
        before = g_sum_x;
        act_move(10, 0);
        CHECK(g_sum_x - before == 10, "NULL filter = passthrough");
    }

    if (failures == 0) printf("\nALL PASSED\n");
    else               printf("\n%d FAILURE(S)\n", failures);
    return failures ? 1 : 0;
}
