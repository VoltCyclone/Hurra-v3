// usb_device.c — device-clone controller DISPATCHER.
//
// Faithful USB MITM cloning must present the clone at the SAME speed as the
// captured device. The two speeds use DIFFERENT controllers on the CH32H417:
//   * High-Speed clone -> USBHSD controller (PB8/PB9, USB-3 Type-A) — usb_device_hs.c
//   * Full/Low-Speed clone -> USBFS controller (PA11/PA12, USB-C) — usb_device_fs.c
// (The USBHS *device* controller does NOT enumerate in Full-Speed mode — verified
// on the bench: it only ever reaches SUSPEND, never a bus reset. WCH's own SDK
// uses two separate controllers for the two speeds; we match that.)
//
// This file owns the stable usb_device.h API and forwards each call to whichever
// backend was selected at runtime by usb_device_init() from the captured device's
// `speed`. The speed isn't known until the descriptor blob arrives (over SPI on
// the device board), so the dispatch is necessarily runtime, not compile-time.
// Each backend keeps ALL its own state and its OWN ISR + IRQ enable, so only the
// active controller's interrupt is live.

#include <stddef.h>          // NULL
#include "usb_device.h"
#include "usb_device_hs.h"   // usbhsd_*
#include "usb_device_fs.h"   // usbfsd_*
#include "usb_host.h"        // USB_SPEED_FULL/LOW/HIGH
#include "icc.h"             // dbg_stage() — UART-readable oracle marker

// Which backend is active: USB_SPEED_HIGH => HS (USBHSD), USB_SPEED_FULL/LOW => FS
// (USBFS), 0xFF => none initialized yet.
#define ACTIVE_NONE  0xFFu
static uint8_t s_active = ACTIVE_NONE;

static inline bool active_is_hs(void) { return s_active == USB_SPEED_HIGH; }
static inline bool active_is_fs(void) { return s_active == USB_SPEED_FULL ||
                                               s_active == USB_SPEED_LOW; }

uint8_t usb_device_active_speed(void) { return s_active; }

bool usb_device_init(const captured_descriptors_t *desc)
{
    if (desc == NULL || !desc->valid) return false;

    switch (desc->speed) {
    case USB_SPEED_HIGH:
        /* HS clone on the USBHSD controller (Type-A port). */
        dbg_stage(0x5C);                 // oracle: dispatching to HS backend
        if (!usbhsd_init(desc)) return false;
        s_active = USB_SPEED_HIGH;
        return true;

    case USB_SPEED_FULL:
    case USB_SPEED_LOW:
        /* FS/LS clone on the USBFS controller (USB-C port). */
        dbg_stage(0x5B);                 // oracle: dispatching to FS backend
        if (!usbfsd_init(desc)) return false;
        s_active = desc->speed;
        return true;

    default:
        return false;                    // unsupported speed — caller treats as fatal
    }
}

void usb_device_poll(void)
{
    if (active_is_hs())      usbhsd_poll();
    else if (active_is_fs()) usbfsd_poll();
}

bool usb_device_send_report(uint8_t ep_num, const uint8_t *data, uint16_t len)
{
    if (active_is_hs())      return usbhsd_send_report(ep_num, data, len);
    else if (active_is_fs()) return usbfsd_send_report(ep_num, data, len);
    return false;
}

bool usb_device_is_configured(void)
{
    if (active_is_hs())      return usbhsd_is_configured();
    else if (active_is_fs()) return usbfsd_is_configured();
    return false;
}

int usb_device_poll_out(uint8_t ep_num, uint8_t **data_ptr)
{
    if (active_is_hs())      return usbhsd_poll_out(ep_num, data_ptr);
    else if (active_is_fs()) return usbfsd_poll_out(ep_num, data_ptr);
    return -1;
}

int usb_device_poll_ep0_report(uint8_t **data_ptr, uint16_t *wValue, uint16_t *wIndex)
{
    if (active_is_hs())      return usbhsd_poll_ep0_report(data_ptr, wValue, wIndex);
    else if (active_is_fs()) return usbfsd_poll_ep0_report(data_ptr, wValue, wIndex);
    return 0;
}

void usb_device_ep0_report_done(void)
{
    if (active_is_hs())      usbhsd_ep0_report_done();
    else if (active_is_fs()) usbfsd_ep0_report_done();
}
