#pragma once
#include <stdint.h>
#include <stdbool.h>
typedef struct __attribute__((packed)) {
	uint8_t  bmRequestType;
	uint8_t  bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} usb_setup_t;
#define USB_SPEED_FULL  0  // 12 Mbps
#define USB_SPEED_LOW   1  // 1.5 Mbps
#define USB_SPEED_HIGH  2  // 480 Mbps
#define USB_REQ_GET_DESCRIPTOR  6
#define USB_REQ_SET_ADDRESS     5
#define USB_REQ_SET_CONFIG      9
#define USB_DESC_DEVICE         1
#define USB_DESC_CONFIGURATION  2
#define USB_DESC_STRING         3
#define USB_DESC_INTERFACE      4
#define USB_DESC_ENDPOINT       5
#define USB_DESC_BOS            0x0F
#define USB_DESC_HID            0x21
#define USB_DESC_HID_REPORT     0x22
bool usb_host_init(void);
bool usb_host_device_connected(void);
void usb_host_port_reset(void);
uint8_t usb_host_device_speed(void);
void usb_host_power_on(void);
int usb_host_control_transfer(uint8_t addr, uint8_t maxpkt,
	const usb_setup_t *setup, uint8_t *data, uint32_t timeout_ms);

// Fire-and-forget control OUT: sets up DMA, kicks hardware, returns immediately.
// Caller must check usb_host_control_async_busy() before calling again.
void usb_host_control_transfer_fire(uint8_t addr, uint8_t maxpkt,
	const usb_setup_t *setup, uint8_t *data);
bool usb_host_control_async_busy(void);

#define MAX_INTR_EPS 7

void usb_host_interrupt_init(uint8_t index, uint8_t addr, uint8_t ep,
	uint16_t maxpkt);
int usb_host_interrupt_poll(uint8_t index, uint8_t *data, uint16_t len);
// Zero-copy poll: returns a pointer into the DMA buffer, valid until the next
// poll. Caller must finish reading/modifying before polling this index again.
int usb_host_interrupt_poll_zerocopy(uint8_t index, uint8_t **data_ptr, uint16_t len);
void usb_host_interrupt_dump_state(void);

#define MAX_INTR_OUT_EPS 7

void usb_host_interrupt_out_init(uint8_t index, uint8_t addr, uint8_t ep,
	uint16_t maxpkt);
// Returns true if the send was armed (QTD primed). Returns false if a previous
// send on this slot is still in flight — caller should retry next poll cycle.
bool usb_host_interrupt_out_send(uint8_t index, const uint8_t *data, uint16_t len);
