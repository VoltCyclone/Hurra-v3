// usb_hs_desc.c — pure USB High-Speed descriptor helper. See usb_hs_desc.h.
// No MMIO, stdint only; host-tested by test/usb_hs_desc_test.c.
#include "usb_hs_desc.h"

uint8_t usb_hs_synth_qualifier(const uint8_t *dev, uint8_t *out)
{
    out[0] = 10;        // bLength
    out[1] = 0x06;      // bDescriptorType = DEVICE_QUALIFIER
    out[2] = dev[2];    // bcdUSB (lo) — mirror the device descriptor
    out[3] = dev[3];    // bcdUSB (hi)
    out[4] = dev[4];    // bDeviceClass
    out[5] = dev[5];    // bDeviceSubClass
    out[6] = dev[6];    // bDeviceProtocol
    out[7] = dev[7];    // bMaxPacketSize0
    out[8] = dev[17];   // bNumConfigurations
    out[9] = 0x00;      // bReserved
    return 10;
}
