#pragma once
#include <stdint.h>
#include "desc_capture.h"

// HID-report merge module (V5F side of the MITM): overlays injected input from
// the V3F command core (via ICC) onto the real device's HID reports.
//
// Handles 8-bit vs 16-bit axis fields, report IDs, per-axis max-value clamping,
// wheel carry, physical-input masking, and keyboard merge, all driven by a
// mouse_layout parsed from the cloned HID report descriptor. usb_merge_drain_icc
// decodes ICC records into the injection accumulators.

void usb_merge_init(void);

// Cache the mouse/keyboard endpoints and parse the cloned HID report descriptor
// into mouse_layout. Call once after capture_descriptors().
void usb_merge_cache_endpoints(const captured_descriptors_t *desc);

// Overlay pending injection onto an in-place HID report captured from the real
// device, before it is forwarded to the PC. iface_protocol: 1=kbd, 2=mouse
// (HID boot-interface protocol code).
void usb_merge_report(uint8_t iface_protocol, uint8_t *report, uint8_t len);

// Drain the V3F->V5F ICC ring into the injection accumulators and run scheduled
// release/humanize bookkeeping. Call first per relay-loop iteration; resets the
// per-cycle merged_this_cycle flag consumed by send_pending.
void usb_merge_drain_icc(void);

// Decode one injection record (tag = ICC_TAG_*, b = the icc_record_t.b[] payload,
// ≥7 readable bytes) into the injection accumulators. Shared by usb_merge_drain_icc
// (ICC source) and the two-board SPI INJECT decode (Board A). Does not touch the
// per-cycle merge flag, the doorbell re-arm, or release bookkeeping — the caller
// owns those. ICC_TAG_DEV_TEMP is handled by the caller, not here.
void usb_merge_apply_record(uint8_t tag, const uint8_t *b);

// Standalone synth-injection path. When V3F injects motion while the physical
// mouse is silent, the merge path (which rides real reports) emits nothing; this
// synthesizes a standalone mouse report (ACK-paced by EP-free) and flushes any
// unconsumed wheel on a separate report ID. Call once per relay-loop iteration
// after the per-EP merge/send loop.
void usb_merge_send_pending(void);

// Test-only: zero injection accumulators + synth cadence state between cases.
void usb_merge_reset_for_test(void);

// Test-only: read the current held injection accumulators.
int16_t usb_merge_peek_inject_dx_for_test(void);
int16_t usb_merge_peek_inject_dy_for_test(void);
int8_t  usb_merge_peek_inject_wheel_for_test(void);

// Upper bound on held injection between device polls. Far above any single-frame
// move; exists only so a long no-consumer window (host suspend while V3F keeps
// injecting) cannot wrap the int16 accumulators and flip a flick's direction.
#define INJECT_ACCUM_CAP 4096
