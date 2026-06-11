#pragma once
#include <stdint.h>
#include "desc_capture.h"

// HID-report merge module (V5F side of the MITM).
//
// Ported from Hurra-v2 src/kmbox.c's merge half: a HID-report-descriptor-AWARE
// overlay that injects synthetic input (from the V3F command core, delivered
// over the ICC) onto the real device's HID reports. It correctly handles 8-bit
// vs 16-bit axis fields, report IDs, per-axis max-value clamping, wheel carry,
// physical-input masking, and keyboard merge — all driven by a `mouse_layout`
// parsed from the cloned HID report descriptor.
//
// On v2 the injection accumulators were written directly by kmbox_inject_*
// (same core). On v3 the V3F core sends injection over the ICC; usb_merge
// DRAINS the ICC (usb_merge_drain_icc) into the SAME accumulators.

void usb_merge_init(void);

// Cache the mouse/keyboard endpoints + parse the cloned HID report descriptor
// into the mouse_layout. Call once after capture_descriptors().
void usb_merge_cache_endpoints(const captured_descriptors_t *desc);

// Overlay any pending injection onto an in-place HID report just captured from
// the real device, before it is forwarded to the PC. iface_protocol: 1=kbd,
// 2=mouse (HID boot-interface protocol code).
void usb_merge_report(uint8_t iface_protocol, uint8_t *report, uint8_t len);

// Drain the V3F->V5F ICC ring into the injection accumulators + run scheduled
// release/humanize bookkeeping. Call once per relay-loop iteration.
void usb_merge_drain_icc(void);
