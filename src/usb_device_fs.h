// usb_device_fs.h — USBFS (Full-Speed) device backend.
//
// One of the two device-clone backends behind the usb_device.h dispatcher
// (src/usb_device.c). Recovered from the pre-step-3 USBFS driver (git 22dc6d3),
// renamed to usbfsd_* so it coexists with the USBHSD backend (usb_device_hs.c).
// Selected at runtime for FS/LS clones (captured_descriptors_t.speed ==
// USB_SPEED_FULL or USB_SPEED_LOW) — the USBHS device controller does not
// enumerate at Full-Speed, so FS devices clone on the separate USBFS controller
// (PA11/PA12, USB-C port). See src/usb_device_fs.c.
#ifndef USB_DEVICE_FS_H
#define USB_DEVICE_FS_H

#include <stdint.h>
#include <stdbool.h>
#include "desc_capture.h"

bool usbfsd_init(const captured_descriptors_t *desc);
void usbfsd_poll(void);
bool usbfsd_send_report(uint8_t ep_num, const uint8_t *data, uint16_t len);
bool usbfsd_is_configured(void);
int  usbfsd_poll_out(uint8_t ep_num, uint8_t **data_ptr);
int  usbfsd_poll_ep0_report(uint8_t **data_ptr, uint16_t *wValue, uint16_t *wIndex);
void usbfsd_ep0_report_done(void);

#endif // USB_DEVICE_FS_H
