#pragma once
#include <stdint.h>

// Pick the next dirty row to render, scanning from a rotating start cursor so no
// row starves and at most one row is painted per call. This bounds display_render
// to a single fill_rect + draw_string per invocation instead of repainting every
// dirty row in one burst (which could block the V3F command loop for ms).
//
// `dirty` is a bitmask of rows needing repaint (bit r set => row r dirty).
// `start` is the row to begin scanning from (the previous pick + 1, wrapped).
// `nrows` is the row count (<= 32). Returns the chosen row index [0, nrows), or
// -1 if no row is dirty.
//
// Pulled into a header so the rotation/wrap logic is unit-testable without the
// ST7789 panel.
static inline int display_pick_row(uint32_t dirty, uint8_t start, uint8_t nrows)
{
	for (uint8_t i = 0; i < nrows; i++) {
		uint8_t r = (uint8_t)((start + i) % nrows);
		if (dirty & (1u << r))
			return (int)r;
	}
	return -1;
}
