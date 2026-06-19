// test/display_rowpick_test.c — host-native tests for the display row picker
// (one-dirty-row-per-call with a rotating start cursor, no starvation).

#include <stdio.h>
#include <stdint.h>
#include "display_rowpick.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { \
    printf("FAIL: %s\n", msg); failures++; } else { \
    printf("ok: %s\n", msg); } } while (0)

int main(void) {
	// No dirty rows -> -1.
	CHECK(display_pick_row(0, 0, 10) == -1, "no dirty rows -> -1");

	// Single dirty row found regardless of start position.
	CHECK(display_pick_row(1u << 3, 0, 10) == 3, "row 3 dirty, start 0 -> 3");

	// Start cursor past the dirty row must still find it by wrapping.
	CHECK(display_pick_row(1u << 2, 5, 10) == 2, "row 2 dirty, start 5 -> wraps to 2");

	// Multiple dirty rows: pick the first AT or AFTER start (fairness/rotation).
	CHECK(display_pick_row((1u << 1) | (1u << 7), 3, 10) == 7,
	      "rows 1,7 dirty, start 3 -> 7 (first at/after start)");

	// Same mask, but start has moved past 7 -> wraps to 1.
	CHECK(display_pick_row((1u << 1) | (1u << 7), 8, 10) == 1,
	      "rows 1,7 dirty, start 8 -> wraps to 1");

	// Dirty row exactly at start is chosen immediately.
	CHECK(display_pick_row(1u << 4, 4, 10) == 4, "dirty row == start -> chosen");

	// Dirty bits outside [0,nrows) are ignored.
	CHECK(display_pick_row(1u << 15, 0, 10) == -1,
	      "dirty bit beyond nrows ignored -> -1");

	if (failures == 0) printf("ALL PASS\n");
	else printf("%d FAILURE(S)\n", failures);
	return failures ? 1 : 0;
}
