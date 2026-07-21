#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float owed_x;
    float owed_y;
    float res_x;
    float res_y;
} humanize_checkpoint_t;

/* Sub-pixel quantizer for injected mouse motion. Operates on the injected
 * delta only; real-mouse passthrough is never routed through it. */
void     humanize_init(uint32_t interval_us);   /* reset injection state */
void     humanize_checkpoint_save(humanize_checkpoint_t *out);
void     humanize_checkpoint_restore(const humanize_checkpoint_t *checkpoint);
bool     humanize_pending(void);   /* true while owed motion remains to emit */
/* Return injected motion the report's delta field could not carry this frame
 * (it was clamped), so it is redelivered as headroom opens. Real-mouse
 * passthrough keeps priority; only the injected overflow comes back. */
void     humanize_return(int16_t dx, int16_t dy);

/* Quantize an injected float delta to an int16 report field. Carries the
 * sub-pixel residual across calls and the >HZ_MAX_PER_FRAME field-clamp
 * overflow (redelivered as headroom opens). Operates on the injected delta
 * only; never touches real-mouse passthrough. */
void     humanize_inject_emit(float dx, float dy, int16_t *out_dx, int16_t *out_dy);
