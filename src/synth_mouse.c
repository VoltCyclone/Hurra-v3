// synth_mouse.c — synthetic HID boot-mouse descriptor + report generator.
// See synth_mouse.h. Step-3 bench scaffold; no MMIO.
#include "synth_mouse.h"
#include <string.h>

/* Standard HID boot-mouse report descriptor (3 bytes: buttons, X, Y). This is
 * the canonical USB HID spec Appendix-E boot mouse — the host treats it as a
 * plain pointing device. */
static const uint8_t k_mouse_report_desc[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)
    0x05, 0x09,        //     Usage Page (Buttons)
    0x19, 0x01,        //     Usage Minimum (01)
    0x29, 0x03,        //     Usage Maximum (03)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x03,        //     Report Count (3)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data, Variable, Absolute) — 3 button bits
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x05,        //     Report Size (5)
    0x81, 0x01,        //     Input (Constant) — 5 bit padding
    0x05, 0x01,        //     Usage Page (Generic Desktop)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x15, 0x81,        //     Logical Minimum (-127)
    0x25, 0x7F,        //     Logical Maximum (127)
    0x75, 0x08,        //     Report Size (8)
    0x95, 0x02,        //     Report Count (2)
    0x81, 0x06,        //     Input (Data, Variable, Relative) — X, Y
    0xC0,              //   End Collection
    0xC0               // End Collection
};

/* 18-byte device descriptor. bcdUSB = 0x0200 (High-Speed capable). Class is
 * per-interface (0x00); a generic synthetic VID/PID. */
static const uint8_t k_device_desc[18] = {
    18,        // bLength
    0x01,      // bDescriptorType = DEVICE
    0x00, 0x02,// bcdUSB = 0x0200
    0x00,      // bDeviceClass (per interface)
    0x00,      // bDeviceSubClass
    0x00,      // bDeviceProtocol
    64,        // bMaxPacketSize0
    0xEF, 0xCA,// idVendor  = 0xCAEF (synthetic)
    0x01, 0x53,// idProduct = 0x5301 (synthetic)
    0x00, 0x01,// bcdDevice = 0x0100
    0x00,      // iManufacturer (none)
    0x00,      // iProduct (none)
    0x00,      // iSerialNumber (none)
    0x01       // bNumConfigurations
};

/* Config descriptor blob: configuration(9) + interface(9) + HID(9) +
 * endpoint(7) = 34 bytes. One HID interface (boot mouse) with a single EP1-IN
 * interrupt endpoint. bInterval=10 ms (FS framing); the USBHSD driver re-encodes
 * it to the HS microframe exponent when serving. */
#define SYNTH_HID_REPORT_DESC_LEN  ((uint8_t)sizeof k_mouse_report_desc)
static const uint8_t k_config_desc[34] = {
    /* configuration descriptor */
    9, 0x02, 34, 0x00, 1, 1, 0x00, 0xA0, 50,   // wTotalLength=34, 1 iface, bus-powered, 100mA
    /* interface descriptor */
    9, 0x04, 0, 0, 1, 0x03, 0x01, 0x02, 0x00,  // 1 EP, class HID, subclass Boot, proto Mouse
    /* HID descriptor */
    9, 0x21, 0x11, 0x01, 0x00, 1, 0x22, SYNTH_HID_REPORT_DESC_LEN, 0x00,
    /* endpoint descriptor: EP1 IN, interrupt, 8-byte, bInterval=10ms */
    7, 0x05, 0x81, 0x03, 8, 0x00, 10
};

void synth_mouse_build_descriptors(captured_descriptors_t *out)
{
    memset(out, 0, sizeof *out);

    memcpy(out->device_desc, k_device_desc, sizeof k_device_desc);
    out->device_desc_len = sizeof k_device_desc;

    memcpy(out->config_desc, k_config_desc, sizeof k_config_desc);
    out->config_desc_len = sizeof k_config_desc;
    out->config_string_idx = 0;

    out->num_ifaces = 1;
    captured_iface_t *itf = &out->ifaces[0];
    itf->iface_num            = 0;
    itf->iface_class          = 0x03;   // HID
    itf->iface_subclass       = 0x01;   // Boot
    itf->iface_protocol       = 0x02;   // Mouse
    itf->iface_string_idx     = 0;
    itf->interrupt_in_ep      = SYNTH_MOUSE_IN_EP;
    itf->interrupt_in_maxpkt  = 8;
    itf->interrupt_in_interval = 10;
    itf->interrupt_out_ep     = 0;      // no OUT endpoint
    itf->has_hid_desc         = true;
    memcpy(itf->hid_report_desc, k_mouse_report_desc, sizeof k_mouse_report_desc);
    itf->hid_report_desc_len  = sizeof k_mouse_report_desc;

    out->num_strings    = 0;
    out->langid_desc_len = 0;
    out->bos_desc_len   = 0;
    out->ms_os_desc_len = 0;

    out->ep0_maxpkt = 64;
    out->dev_addr   = 0;
    out->speed      = 0;   // USB_SPEED_FULL — the synthetic mouse clones as a FS
                           // device (faithful: a boot mouse is Full-Speed). The
                           // USBHSD controller is HS-capable but presents FS here.
    out->valid      = true;
}

uint8_t synth_mouse_next_report(uint32_t tick, uint8_t *report)
{
    // Trace a slow circle: step around a 16-phase ring, each phase a small
    // signed dx/dy so the cursor visibly drifts in a loop when the link is up.
    // Integer-only (no trig on the MCU): an 8-step square-ish path is plenty to
    // prove "the cursor moves" at the bench gate.
    static const int8_t dx_tab[8] = {  3,  3,  0, -3, -3, -3,  0,  3 };
    static const int8_t dy_tab[8] = {  0,  3,  3,  3,  0, -3, -3, -3 };
    uint8_t phase = (uint8_t)(tick & 0x7u);

    report[0] = 0x00;                       // buttons: none pressed
    report[1] = (uint8_t)dx_tab[phase];     // dx
    report[2] = (uint8_t)dy_tab[phase];     // dy
    return SYNTH_MOUSE_REPORT_LEN;
}
