#pragma once
// usb_cdc_fs.h — CDC-ACM virtual COM port on Board B's idle USBFS controller.
//
// Board B (host role) captures the real device on its USBHS host port and ships
// it to Board A over SPI. Its USBFS controller (PA11/PA12, USB-C) is otherwise
// idle, so this driver brings it up as a USB CDC-ACM device — a virtual serial
// port the control machine opens with no driver install — to carry an in-band
// command/response channel (kmbox-style injection text).
//
// This is the SOLE owner of the host image's USBFS_IRQHandler: the host build
// drops usb_device.c / usb_device_fs.c / usb_device_hs.c (see the Makefile
// `ifeq ($(BOARD),host)` block), so there is no duplicate ISR. The HID-clone
// USBFS driver (usb_device_fs.c) is only linked into the device image.
//
// Conventions mirror src/usb_device_fs.c (NOT the WCH vendor SimulateCDC
// example): DMA buffers live in the .usbdma section and are accessed through
// the +0x20000000 CPU-side alias; the shared USBHS 480 MHz PLL is brought up
// idempotently and never torn down; the V5F IRQ is routed with
// NVIC_SetAllocateIRQ(USBFS_IRQn, Core_ID_V5F) before enabling.
//
// EP layout (CDC-ACM, fixed — no cloned device):
//   EP0      control, MPS 64 (enumeration + CDC class requests)
//   EP1 IN   interrupt notification (required by CDC-ACM; unused, never armed)
//   EP2 OUT  bulk, host->device command bytes -> rx ring
//   EP3 IN   bulk, device->host response bytes <- tx ring

#include <stdint.h>
#include <stdbool.h>

// Bring up RCC (idempotent shared-PLL reuse), the USBFS controller, EP0/EP2/EP3,
// route + enable the V5F IRQ. Safe to call once at startup. The controller drives
// PA11/PA12 directly (no GPIO AF config needed, matching usb_device_fs.c).
void cdc_fs_init(void);

// Foreground EP servicing: re-arm bulk OUT (EP2) after the ring has been drained,
// and flush any queued bulk IN (EP3) when the endpoint is idle. Call from the
// main loop. The ISR does the wire-level work; this just moves the rings.
void cdc_fs_poll(void);

// Pull up to `max` received command bytes out of the rx ring into `buf`.
// Returns the number of bytes copied (0 if none pending).
uint16_t cdc_fs_rx_read(uint8_t *buf, uint16_t max);

// Queue up to `len` response bytes into the tx ring for the host to read on
// EP3 IN. Returns the number of bytes accepted (may be < len if the ring fills).
uint16_t cdc_fs_tx_write(const uint8_t *buf, uint16_t len);

// True once the host has issued SET_CONFIGURATION (the COM port is open-able).
bool cdc_fs_is_configured(void);

// The single host-image USBFS interrupt service routine. Must match the vector
// name in core/startup_v5f.S.
void USBFS_IRQHandler(void);
