#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Always-on humanization filter. Operates on the INJECTED mouse delta only;
 * real-mouse passthrough is never routed through it. */
void     humanize_init(uint32_t interval_us);   /* seed + level default */
void     humanize_filter(int16_t *dx, int16_t *dy); /* in-place, per frame */
uint32_t humanize_timing_next(uint32_t base_ldval);
void     humanize_set_level(uint8_t level);      /* 0=off..3=strong */
uint8_t  humanize_get_level(void);               /* current level 0..3 */
bool     humanize_pending(void);   /* true while owed motion remains to emit */
/* Return injected motion that the report's delta field could not carry this
 * frame (it was clamped), so the filter redelivers it as headroom opens.
 * Real-mouse passthrough keeps priority; only the injected overflow comes back. */
void     humanize_return(int16_t dx, int16_t dy);

/* ── Adaptive feed-rate (measured poll interval) ────────────────────────
 * Record the arrival of a real mouse report, timestamped from the free-running
 * 1 MHz GPT2 counter (microseconds). Only mouse reports should be passed.
 * Builds an EWMA of the *delivery* interval, rejecting dropouts/double-reports.
 * Safe to call from the main loop only (no ISR). */
void     humanize_record_arrival(uint32_t ts_us);

/* Target PIT LDVAL derived from the measured interval, or 0 when there is not
 * yet a confident measurement (caller keeps its current base in that case).
 * pit_clk_hz is the PIT input clock (PERCLK, 24 MHz on this board).
 * The caller is expected to SLEW its base toward this, not jump to it. */
uint32_t humanize_target_ldval(uint32_t pit_clk_hz);

/* Last EWMA-smoothed measured interval in microseconds (0 = none yet).
 * Exposed for diagnostics / status reporting. */
uint32_t humanize_measured_interval_us(void);
