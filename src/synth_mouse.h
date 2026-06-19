// synth_mouse.h — synthetic HID boot-mouse descriptor + report generator.
//
// Two-board bench scaffold (Makefile BOARD=host HOST_SYNTH=1): Board B generates
// synthetic mouse reports over the SPI link; Board A enumerates a synthetic boot
// mouse on its USB device port and replays them, proving the link without a real
// device. Provides both halves of the fake:
//   * synth_mouse_build_descriptors() — a captured_descriptors_t for a 3-byte HID
//     boot mouse (1 interface, EP1-IN interrupt).
//   * synth_mouse_next_report() — a 3-byte boot-mouse report driving a slow
//     circular cursor motion.
//
// Pure data + arithmetic, no MMIO; host-compilable.
#ifndef SYNTH_MOUSE_H
#define SYNTH_MOUSE_H

#include <stdint.h>
#include "desc_capture.h"

// Synthetic boot-mouse IN endpoint (0x81 => EP1 IN), report size, and protocol.
// Board B tags each SPI frame with these; Board A maps them.
#define SYNTH_MOUSE_IN_EP        0x81u
#define SYNTH_MOUSE_REPORT_LEN   3u     // [buttons, dx, dy] boot protocol
#define SYNTH_MOUSE_IFACE_PROTO  2u     // HID boot protocol: 2 = mouse

// Fill `out` with a self-consistent captured_descriptors_t for a HID boot mouse
// (bcdUSB 0x0200) and set ->valid = true. Safe to call once at boot.
void synth_mouse_build_descriptors(captured_descriptors_t *out);

// Write the next 3-byte boot-mouse report into `report` (>= 3 bytes). `tick` is
// a monotonically increasing counter mapped to a small circular dx/dy motion
// with no buttons. Returns SYNTH_MOUSE_REPORT_LEN.
uint8_t synth_mouse_next_report(uint32_t tick, uint8_t *report);

#endif // SYNTH_MOUSE_H
