// usb_device_hs.h — USBHSD (High-Speed) device-clone backend behind the
// usb_device.h dispatcher.
//
// usbhsd_* names coexist with the USBFS backend (usb_device_fs.c). Selected at
// runtime for HS clones (captured_descriptors_t.speed == USB_SPEED_HIGH).
#ifndef USB_DEVICE_HS_H
#define USB_DEVICE_HS_H

#include <stdint.h>
#include <stdbool.h>
#include "desc_capture.h"

bool usbhsd_init(const captured_descriptors_t *desc);
void usbhsd_poll(void);
bool usbhsd_send_report(uint8_t ep_num, const uint8_t *data, uint16_t len);
bool usbhsd_in_ep_free(uint8_t ep_num);
bool usbhsd_is_configured(void);
int  usbhsd_poll_out(uint8_t ep_num, uint8_t **data_ptr);
int  usbhsd_poll_ep0_report(uint8_t **data_ptr, uint16_t *wValue, uint16_t *wIndex);
void usbhsd_ep0_report_done(void);

#endif // USB_DEVICE_HS_H
