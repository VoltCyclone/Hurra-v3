// two_board.h — two-board role loops (USB-over-SPI MITM split across two boards).
//
// The two entry points are selected at build time by the Makefile BOARD= flag and
// called from main_v5f.c instead of the single-board relay (see AGENTS.md):
//   * BOARD=host   -> -DBOARD_ROLE_HOST   -> two_board_host_run()
//   * BOARD=device -> -DBOARD_ROLE_DEVICE -> two_board_device_run()
// Neither returns. Both keep the dbg_stage UART oracle and the SEQ-gap drop
// counter live so a dead cursor is attributable to a specific hop (SPI-RX vs USB
// clone).
//
// Board B (host)   = SPI MASTER: capture the device on USBHS host (or a synthetic
//                    mouse with HOST_SYNTH=1), ship descriptors + reports over SPI
//                    as 32-byte frames.
// Board A (device) = SPI SLAVE:  receive frames, reassemble the descriptor blob,
//                    enumerate the clone on USBHSD/USBFS, merge+inject, send to PC.
#ifndef TWO_BOARD_H
#define TWO_BOARD_H

// SPI frame TYPE for a HID report fragment (high bit clear = data class).
#define TWO_BOARD_TYPE_REPORT  0x02u

// SPI frame TYPE for a descriptor-blob chunk. Board B ships the captured descriptor
// set as a sequence of these; Board A reassembles them (desc_xfer) and enumerates.
#define TWO_BOARD_TYPE_DESC   0x03u

// SPI frame TYPE for device->host telemetry on the MISO slot.
// Payload: [enum(0/1), clone_speed(USB_SPEED_*), dev_temp(int8)].
#define TWO_BOARD_TYPE_TELEM      0x04u

// SPI frame TYPE for the host's periodic telemetry poll (empty payload). Clocked
// when no report was sent recently so the device's MISO slot keeps flowing and the
// host's freshness heartbeat advances during an idle mouse.
#define TWO_BOARD_TYPE_TELEM_REQ  0x05u

// SPI frame TYPE for a host->device injection command. Payload is the raw
// icc_record_t byte layout: payload[0]=tag (ICC_TAG_*), payload[1..]=b[]. Lets
// Board B's V5F parser forward injection to Board A over the existing SPI link,
// decoded by usb_merge_apply_record into the same accumulators the ICC drain
// feeds. High bit clear = data class.
#define TWO_BOARD_TYPE_INJECT  0x06u

// Board B (USB host) main loop: never returns.
void two_board_host_run(void);

// Board A (USB device) main loop: never returns.
void two_board_device_run(void);

#endif // TWO_BOARD_H
