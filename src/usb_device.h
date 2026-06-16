#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "desc_capture.h"

// Maximum endpoints: EP0 (control) + up to 7 more
#define USB_DEV_NUM_ENDPOINTS  8

// Device state
typedef enum {
	USB_DEV_STATE_DEFAULT,
	USB_DEV_STATE_ADDRESS,
	USB_DEV_STATE_CONFIGURED
} usb_dev_state_t;

// Maximum interrupt IN endpoints for composite device support.
// Hardware ceiling is USB_DEV_NUM_ENDPOINTS - 1 (EP0 reserved for control).
#define MAX_INT_EPS 7

// Maximum interrupt OUT endpoints — symmetric with MAX_INT_EPS, supports
// passing host→device traffic on vendor interfaces (Logitech HID++, etc.)
#define MAX_INT_OUT_EPS 7

// Public API
bool usb_device_init(const captured_descriptors_t *desc);
void usb_device_poll(void);
bool usb_device_send_report(uint8_t ep_num, const uint8_t *data, uint16_t len);
bool usb_device_is_configured(void);

// Drain a completed RX from the given device OUT EP. Returns:
//   > 0  : bytes received, *data points into DMA buffer (valid until next call)
//   = 0  : no completion yet
//   < 0  : EP not configured / error
int usb_device_poll_out(uint8_t ep_num, uint8_t **data_ptr);

// Drain a completed EP0 HID SET_REPORT (host->device control write captured on
// the device side) for replay onto the real device's EP0. Returns:
//   > 0 : payload length; *data points into the staging buffer (valid until
//         usb_device_ep0_report_done()), *wValue/*wIndex hold the setup fields
//   = 0 : nothing pending
// Call usb_device_ep0_report_done() once the payload has been forwarded.
int  usb_device_poll_ep0_report(uint8_t **data_ptr, uint16_t *wValue,
                                uint16_t *wIndex);
void usb_device_ep0_report_done(void);
