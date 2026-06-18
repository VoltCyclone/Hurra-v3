// usb_hs_desc.h — pure (host-testable) USB High-Speed descriptor helper.
//
// The one High-Speed-only descriptor a cloned device must synthesize: the
// DEVICE_QUALIFIER. Kept MMIO/SDK-free so it can be unit tested on the host
// (test/usb_hs_desc_test.c, in `make test`); the USBHSD driver in usb_device.c
// calls it only when cloning an HS device.
//
// NOTE: descriptors are otherwise served VERBATIM (faithful-clone correctness) —
// the earlier FS->HS bcdUSB/bInterval rewrites were removed in step 4a, since a
// MITM must present the device at its real speed, not force FS up to HS.
#ifndef USB_HS_DESC_H
#define USB_HS_DESC_H

#include <stdint.h>

// Synthesize a 10-byte USB DEVICE_QUALIFIER (type 0x06) from an 18-byte device
// descriptor. A High-Speed device must answer GET_DESCRIPTOR(QUALIFIER); the
// qualifier mirrors the device descriptor's class/subclass/protocol, EP0 max
// packet, and bNumConfigurations for "the other speed". `dev` must point at >= 18
// bytes; `out` receives exactly 10 bytes. Returns the bytes written (always 10).
uint8_t usb_hs_synth_qualifier(const uint8_t *dev, uint8_t *out);

#endif // USB_HS_DESC_H
