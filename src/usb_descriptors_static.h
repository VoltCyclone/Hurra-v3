// usb_descriptors_static.h — static USB 2.0 Full-Speed boot-mouse descriptors.
//
// Phase 4 bring-up: the USBFS device driver (src/usb_device.c) enumerates a
// SINGLE HID boot mouse to the PC using the static arrays declared here.
//
// We deliberately use a single-interface boot mouse (NOT the vendored
// composite keyboard+mouse in usb_reference/.../usb_desc.c) because Phase 4
// only needs one interrupt-IN endpoint to prove the EP0 enumeration state
// machine and the EP1 IN report path. A single boot mouse is the smallest
// self-consistent descriptor set that a PC will fully enumerate.
//
// Report format on EP1 IN (4 bytes), matching usb_device_send_report(1, ...):
//     byte0: buttons (bit0 left, bit1 right, bit2 middle)
//     byte1: dx   (signed -127..127)
//     byte2: dy   (signed -127..127)
//     byte3: wheel(signed -127..127)
//
// PLACEHOLDER VID/PID — Phase 5 swaps these (and the whole descriptor set)
// for descriptors cloned from the real downstream device (desc_capture.c).

#pragma once
#include <stdint.h>

/* ---- Sizing constants -------------------------------------------------- */
#define USBD_EP0_SIZE              64    /* control endpoint max packet */
#define USBD_EP1_IN_SIZE          4     /* boot-mouse report size (bytes) */
#define USBD_EP1_IN_INTERVAL      10    /* bInterval, 10 ms polling */

/* PLACEHOLDER ids — Phase 5 replaces with cloned descriptors. */
#define USBD_STATIC_VID            0x1F6A
#define USBD_STATIC_PID            0x0001

/* ---- Device Descriptor (18 bytes) -------------------------------------- */
static const uint8_t USBD_DeviceDescr[] = {
    0x12,                                   // bLength
    0x01,                                   // bDescriptorType: DEVICE
    0x00, 0x02,                             // bcdUSB 2.00
    0x00,                                   // bDeviceClass (per-interface)
    0x00,                                   // bDeviceSubClass
    0x00,                                   // bDeviceProtocol
    USBD_EP0_SIZE,                          // bMaxPacketSize0
    (uint8_t)USBD_STATIC_VID, (uint8_t)(USBD_STATIC_VID >> 8),   // idVendor
    (uint8_t)USBD_STATIC_PID, (uint8_t)(USBD_STATIC_PID >> 8),   // idProduct
    0x00, 0x01,                             // bcdDevice 1.00
    0x01,                                   // iManufacturer (string 1)
    0x02,                                   // iProduct      (string 2)
    0x00,                                   // iSerialNumber (none)
    0x01,                                   // bNumConfigurations
};

/* ---- Configuration Descriptor Set --------------------------------------
 * Layout (wTotalLength = 0x22 = 34):
 *   [0]  Configuration descriptor          9 bytes
 *   [9]  Interface descriptor (HID mouse)  9 bytes
 *   [18] HID descriptor                    9 bytes  <- offset 18, like vendor
 *   [27] Endpoint descriptor (EP1 IN)      7 bytes
 * The HID descriptor begins at byte offset 18 so the GET_DESCRIPTOR(HID)
 * handler can return &USBD_ConfigDescr[18] exactly as the vendor driver does.
 */
#define USBD_REPORT_DESC_LEN      52    /* sizeof(USBD_ReportDescr), see below */

static const uint8_t USBD_ConfigDescr[] = {
    /* Configuration Descriptor */
    0x09,                                   // bLength
    0x02,                                   // bDescriptorType: CONFIGURATION
    0x22, 0x00,                             // wTotalLength = 34
    0x01,                                   // bNumInterfaces
    0x01,                                   // bConfigurationValue
    0x00,                                   // iConfiguration
    0xA0,                                   // bmAttributes: bus-powered, remote-wakeup
    0x32,                                   // bMaxPower: 100 mA

    /* Interface Descriptor (HID boot mouse) — offset 9 */
    0x09,                                   // bLength
    0x04,                                   // bDescriptorType: INTERFACE
    0x00,                                   // bInterfaceNumber
    0x00,                                   // bAlternateSetting
    0x01,                                   // bNumEndpoints
    0x03,                                   // bInterfaceClass: HID
    0x01,                                   // bInterfaceSubClass: Boot
    0x02,                                   // bInterfaceProtocol: Mouse
    0x00,                                   // iInterface

    /* HID Descriptor — offset 18 */
    0x09,                                   // bLength
    0x21,                                   // bDescriptorType: HID
    0x11, 0x01,                             // bcdHID 1.11
    0x00,                                   // bCountryCode
    0x01,                                   // bNumDescriptors
    0x22,                                   // bDescriptorType: REPORT
    USBD_REPORT_DESC_LEN, 0x00,            // wDescriptorLength = 52

    /* Endpoint Descriptor (EP1 IN, interrupt) — offset 27 */
    0x07,                                   // bLength
    0x05,                                   // bDescriptorType: ENDPOINT
    0x81,                                   // bEndpointAddress: IN, EP1
    0x03,                                   // bmAttributes: Interrupt
    USBD_EP1_IN_SIZE, 0x00,                // wMaxPacketSize = 4
    USBD_EP1_IN_INTERVAL,                  // bInterval = 10 ms
};

/* ---- HID Report Descriptor (52 bytes) ----------------------------------
 * Standard 4-byte boot mouse: 3 buttons + 5 padding bits, then X/Y/Wheel.
 */
static const uint8_t USBD_ReportDescr[] = {
    0x05, 0x01,             // Usage Page (Generic Desktop)
    0x09, 0x02,             // Usage (Mouse)
    0xA1, 0x01,             // Collection (Application)
    0x09, 0x01,             //   Usage (Pointer)
    0xA1, 0x00,             //   Collection (Physical)
    0x05, 0x09,             //     Usage Page (Buttons)
    0x19, 0x01,             //     Usage Minimum (Button 1)
    0x29, 0x03,             //     Usage Maximum (Button 3)
    0x15, 0x00,             //     Logical Minimum (0)
    0x25, 0x01,             //     Logical Maximum (1)
    0x95, 0x03,             //     Report Count (3)
    0x75, 0x01,             //     Report Size (1)
    0x81, 0x02,             //     Input (Data,Var,Abs)  -> 3 button bits
    0x95, 0x01,             //     Report Count (1)
    0x75, 0x05,             //     Report Size (5)
    0x81, 0x01,             //     Input (Const)         -> 5 padding bits
    0x05, 0x01,             //     Usage Page (Generic Desktop)
    0x09, 0x30,             //     Usage (X)
    0x09, 0x31,             //     Usage (Y)
    0x09, 0x38,             //     Usage (Wheel)
    0x15, 0x81,             //     Logical Minimum (-127)
    0x25, 0x7F,             //     Logical Maximum (127)
    0x75, 0x08,             //     Report Size (8)
    0x95, 0x03,             //     Report Count (3)
    0x81, 0x06,             //     Input (Data,Var,Rel)  -> X, Y, Wheel
    0xC0,                   //   End Collection
    0xC0,                   // End Collection
};

/* ---- String Descriptors ------------------------------------------------- */
/* String 0: LANGID table — 0x0409 (English-US) */
static const uint8_t USBD_LangDescr[] = {
    0x04,                   // bLength
    0x03,                   // bDescriptorType: STRING
    0x09, 0x04,             // LANGID 0x0409
};

/* String 1: Manufacturer "Hurra" (UTF-16LE) */
static const uint8_t USBD_ManuDescr[] = {
    0x0C,                   // bLength = 2 + 5*2
    0x03,                   // bDescriptorType: STRING
    'H', 0, 'u', 0, 'r', 0, 'r', 0, 'a', 0,
};

/* String 2: Product "Hurra Mouse" (UTF-16LE) */
static const uint8_t USBD_ProdDescr[] = {
    0x18,                   // bLength = 2 + 11*2
    0x03,                   // bDescriptorType: STRING
    'H', 0, 'u', 0, 'r', 0, 'r', 0, 'a', 0, ' ', 0,
    'M', 0, 'o', 0, 'u', 0, 's', 0, 'e', 0,
};

/* Compile-time guard: report descriptor length must match wDescriptorLength
 * advertised in the HID descriptor and the EP1 report size used elsewhere. */
_Static_assert(sizeof(USBD_ReportDescr) == USBD_REPORT_DESC_LEN,
               "report descriptor length mismatch");
