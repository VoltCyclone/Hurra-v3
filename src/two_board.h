// two_board.h — two-board role loops (USB-over-SPI MITM split across two boards).
//
// These two entry points are selected at build time by the Makefile BOARD= flag
// and called from main_v5f.c instead of the single-board relay (see AGENTS.md):
//   * BOARD=host   -> -DBOARD_ROLE_HOST   -> two_board_host_run()
//   * BOARD=device -> -DBOARD_ROLE_DEVICE -> two_board_device_run()
// Neither returns (each is the board's main loop). They keep the dbg_stage UART
// oracle and the SEQ-gap drop counter live so a dead cursor is attributable to a
// specific hop (SPI-RX vs USB clone).
//
// Board B (host)   = SPI MASTER: capture the real device on USBHS host (or a
//                    synthetic mouse with HOST_SYNTH=1), ship its descriptors +
//                    reports over the SPI link as 32-byte frames.
// Board A (device) = SPI SLAVE:  receive frames, reassemble the descriptor blob,
//                    enumerate the clone on USBHSD/USBFS, merge+inject, send to PC.
#ifndef TWO_BOARD_H
#define TWO_BOARD_H

// SPI hot-path frame TYPE for a synthetic mouse report (high bit clear = data
// class; arbitrary low value, distinct from any control type we add later).
#define TWO_BOARD_TYPE_MOUSE  0x02u

// SPI frame TYPE for a descriptor-blob chunk (step 4b). Board B ships the captured
// descriptor set as a sequence of these before/between mouse frames; Board A
// reassembles them (desc_xfer) and then enumerates the clone.
#define TWO_BOARD_TYPE_DESC   0x03u

// SPI frame TYPE for device->host telemetry returned on the MISO slot (step: TFT
// pass-through). Payload: [enum(0/1), clone_speed(USB_SPEED_*), dev_temp(int8)].
#define TWO_BOARD_TYPE_TELEM      0x04u

// SPI frame TYPE for the host's periodic telemetry poll (empty payload). The host
// clocks this when no report has been sent recently so the device's MISO slot keeps
// flowing and the host's freshness heartbeat keeps advancing during idle mouse.
#define TWO_BOARD_TYPE_TELEM_REQ  0x05u

// Board B (USB host stand-in) main loop: never returns.
void two_board_host_run(void);

// Board A (USB device) main loop: never returns.
void two_board_device_run(void);

#endif // TWO_BOARD_H
