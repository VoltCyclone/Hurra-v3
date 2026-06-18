// synth_mouse.h — synthetic HID boot-mouse descriptor + report generator.
//
// Two-board bench scaffold. An ISOLATED-LINK alternative to real USBHS host
// capture (Makefile BOARD=host HOST_SYNTH=1): Board B (SPI master) generates
// synthetic mouse reports and ships them over the SPI link; Board A (SPI slave)
// enumerates a synthetic boot mouse on its USBHSD/USBFS device port and replays
// whatever arrives. Used to prove the link without a real device. This module
// provides both halves of the fake:
//   * synth_mouse_build_descriptors() — a complete captured_descriptors_t for a
//     standard 3-byte HID boot mouse (1 interface, EP1-IN interrupt), so Board A
//     can call usb_device_init()/usb_merge_cache_endpoints() with no real capture.
//   * synth_mouse_next_report() — a 3-byte boot-mouse report (buttons, dx, dy)
//     driving a slow circular cursor wiggle, so "the cursor moves" is the gate.
//
// Pure data + arithmetic, no MMIO — host-compilable (no unit test yet; it is
// scaffolding, validated by the bench gate, not by `make test`).
#ifndef SYNTH_MOUSE_H
#define SYNTH_MOUSE_H

#include <stdint.h>
#include "desc_capture.h"

// The synthetic boot-mouse IN endpoint (address 0x81 => EP1 IN) and its report
// size. Board B tags each SPI frame with this EP/protocol; Board A maps it.
#define SYNTH_MOUSE_IN_EP        0x81u
#define SYNTH_MOUSE_REPORT_LEN   3u     // [buttons, dx, dy] boot protocol
#define SYNTH_MOUSE_IFACE_PROTO  2u     // HID boot protocol: 2 = mouse

// Fill `out` with a self-consistent captured_descriptors_t describing a HID boot
// mouse. Sets ->valid = true. bcdUSB is 0x0200 already (the USBHSD driver also
// enforces >= 0x0200, so this is belt-and-suspenders). Safe to call once at boot.
void synth_mouse_build_descriptors(captured_descriptors_t *out);

// Write the next 3-byte boot-mouse report into `report` (must hold >= 3 bytes).
// `tick` is a monotonically increasing counter (e.g. a loop/ms counter); the
// generator turns it into a small circular dx/dy motion with no buttons pressed,
// so the PC cursor traces a slow circle when the link is healthy. Returns the
// report length (always SYNTH_MOUSE_REPORT_LEN).
uint8_t synth_mouse_next_report(uint32_t tick, uint8_t *report);

#endif // SYNTH_MOUSE_H
