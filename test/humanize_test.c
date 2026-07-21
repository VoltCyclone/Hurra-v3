#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "humanize.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    printf("FAIL: %s\n", msg); failures++; } } while (0)

int main(void) {
    humanize_init(1000);            /* 1 ms frame */
    CHECK(1, "scaffold");

    /* ── injected emit quantization (sub-pixel + clamp carry) ── */
    {
        /* (A) Sub-pixel carry: a stream of 0.4-count steps must accumulate into
         *     whole counts (no per-step truncation-to-zero loss). */
        humanize_init(1000);
        long acc = 0;
        for (int i = 0; i < 100; i++) {
            int16_t ox = 0, oy = 0;
            humanize_inject_emit(0.4f, 0.0f, &ox, &oy);
            acc += ox;
        }
        CHECK(labs(acc - 40L) <= 1, "inject_emit: sub-pixel residual conserves (0.4*100=40)");

        /* (B) Field-clamp carry: a huge single step never teleports, and the
         *     clamped overflow is redelivered over later calls (not dropped). */
        humanize_init(1000);
        int16_t bx = 0, by = 0; long total = 0; int maxframe = 0;
        humanize_inject_emit(30000.0f, 0.0f, &bx, &by);
        total += bx; if (abs(bx) > maxframe) maxframe = abs(bx);
        for (int i = 0; i < 4000; i++) {
            int16_t dx = 0, dy = 0;
            humanize_inject_emit(0.0f, 0.0f, &dx, &dy);
            total += dx; if (abs(dx) > maxframe) maxframe = abs(dx);
        }
        CHECK(maxframe <= 127, "inject_emit: no single frame exceeds human cap");
        CHECK(labs(total - 30000L) <= 4, "inject_emit: clamped overflow redelivered, not lost");

        /* (C) Two-axis independence: a diagonal stream conserves on both axes. */
        humanize_init(1000);
        long ax = 0, ay = 0;
        for (int i = 0; i < 200; i++) {
            int16_t dx = 0, dy = 0;
            humanize_inject_emit(1.5f, -0.75f, &dx, &dy);
            ax += dx; ay += dy;
        }
        CHECK(labs(ax - 300L) <= 1, "inject_emit: x-axis conserves (1.5*200=300)");
        CHECK(labs(ay + 150L) <= 1, "inject_emit: y-axis conserves (-0.75*200=-150)");
    }

    /* (D) Checkpoint restore: rewinding the fractional phase is bit-exact. */
    {
        humanize_init(1000);
        int16_t seed_x, seed_y;
        humanize_inject_emit(0.375f, -0.625f, &seed_x, &seed_y);

        humanize_checkpoint_t before, expected_after, restored, actual_after;
        humanize_checkpoint_save(&before);

        int16_t expected_x, expected_y;
        humanize_inject_emit(13.25f, -8.5f, &expected_x, &expected_y);
        humanize_checkpoint_save(&expected_after);

        humanize_checkpoint_restore(&before);
        humanize_checkpoint_save(&restored);
        CHECK(memcmp(&before, &restored, sizeof before) == 0,
              "checkpoint restore is bit-exact");

        int16_t actual_x, actual_y;
        humanize_inject_emit(13.25f, -8.5f, &actual_x, &actual_y);
        humanize_checkpoint_save(&actual_after);
        CHECK(actual_x == expected_x && actual_y == expected_y,
              "restored phase reproduces the next quantization");
        CHECK(memcmp(&expected_after, &actual_after, sizeof expected_after) == 0,
              "restored phase reproduces the next state");
    }

    /* (E) Idle: no injected motion in, zero out. */
    {
        humanize_init(1000);
        int moved = 0;
        for (int i = 0; i < 500; i++) {
            int16_t dx = 0, dy = 0;
            humanize_inject_emit(0.0f, 0.0f, &dx, &dy);
            if (dx || dy) moved = 1;
        }
        CHECK(!moved, "inject_emit: idle stream emits nothing");
    }

    printf(failures ? "\n%d FAILED\n" : "\nALL PASSED\n", failures);
    return failures ? 1 : 0;
}
