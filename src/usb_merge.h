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

// Standalone synth-injection path. When V3F injects motion while the physical
// mouse is silent, the merge path (which rides real reports) emits nothing; this
// synthesizes a standalone mouse report (one-per-ms cap) and flushes any
// unconsumed wheel on a separate report ID. Call once per relay-loop iteration
// after the per-EP merge/send loop.
void usb_merge_send_pending(void);
