// spi_link.h — board-to-board SPI link driver (SPI1 on GPIOA AF5, see board.h
// LINK_*: SCK PA5, NSS PA4, MISO PA6, MOSI PA7, DATA_READY PA3). Carries the fixed
// 32-byte hot-path slot (src/spi_frame.h) between the USB-host board (SPI master)
// and the USB-device board (SPI slave).
//
// The forward path (master -> slave descriptor/report stream) is the proven one:
// the master clocks polled full-duplex slots; the slave receives them either polled
// (spi_link_slave_exchange, paced isolated frames) or interrupt-driven
// (spi_link_slave_init_irq + a SOF-scanner, for the continuous descriptor burst).
// The reverse channel (DATA_READY on PA3) is wired but currently unused.
//
// This file touches MMIO (WCH StdPeriph), so it is NOT host-testable; its gate is
// the bench. The wire FORMAT it carries is host-tested in test/spi_frame_test.c.
#ifndef SPI_LINK_H
#define SPI_LINK_H

#include <stdint.h>
#include "spi_frame.h"

// One full-duplex transfer moves exactly one slot each way.
#define SPI_LINK_SLOT  SPI_FRAME_SLOT_SIZE   // 32

// ── Master (USB-host board) ─────────────────────────────────────────────────
// Init SPI1 as master on PA5/6/7 (AF5), CS driven as a plain GPIO on PA4 so a
// glitch can't propagate past one slot, and PA3 (DATA_READY) as a floating input.
// Must be called once before any exchange.
void spi_link_master_init(void);

// Blocking full-duplex exchange: drive CS low, clock SPI_LINK_SLOT bytes (sending
// tx[], receiving into rx[]), drive CS high. tx and rx may alias the same buffer.
// Either pointer may be NULL (NULL tx sends zeros; NULL rx discards). The caller
// builds tx with spi_frame_pack() and validates rx with spi_frame_unpack().
void spi_link_master_exchange(const uint8_t tx[SPI_LINK_SLOT],
                              uint8_t rx[SPI_LINK_SLOT]);

// Nonzero if the slave is asserting DATA_READY (has reverse-path data pending).
// Reads the PA3 level directly — usable without enabling any interrupt.
int  spi_link_master_drdy(void);

// DIAG: count of master flag-waits that timed out (a wedge caught + slot aborted +
// block recovered). 0 on a healthy link; a rising value localizes the gate-4b
// "frozen heartbeat" to the master's SPI block stalling. Written by the master path.
extern volatile uint32_t spi_link_master_wedges;

// ── Slave (USB-device board) ────────────────────────────────────────────────
// Init SPI1 as slave on PA5/6/7 (AF5), software-NSS held selected, and PA3
// (DATA_READY) as a GPIO output (deasserted). Must be called once before any
// exchange.
void spi_link_slave_init(void);

// Blocking full-duplex exchange from the slave side: waits for the master's clock
// and moves SPI_LINK_SLOT bytes each way (sending tx[], receiving into rx[]). The
// slave MUST have tx staged before the master starts clocking, so the caller
// arms the return slot (tx) here and reads the received slot from rx afterward.
// tx/rx may alias; either may be NULL.
void spi_link_slave_exchange(const uint8_t tx[SPI_LINK_SLOT],
                             uint8_t rx[SPI_LINK_SLOT]);

// Drive DATA_READY (PA3) to tell the master reverse-path data is pending (or not).
void spi_link_slave_set_drdy(int asserted);

// Publish a 32-byte telemetry slot for the IRQ slave to stage on MISO. The RXNE
// ISR cycles these bytes onto the data register (one per clocked byte, repeating
// the slot), so the master sees a continuous stream of this slot on the return
// path. Double-buffered: the copy is atomic w.r.t. the ISR (a publish in progress
// never yields a torn slot to the wire). Pass a slot built with spi_frame_pack.
void spi_link_slave_set_telem(const uint8_t slot[SPI_LINK_SLOT]);

// ── Slave RX, interrupt-driven (stream-capable) ─────────────────────────────
// The polled spi_link_slave_exchange above only reliably catches PACED ISOLATED
// frames — it loses byte alignment under a continuous frame stream (the descriptor
// transfer). For streaming RX, init the slave with spi_link_slave_init_irq(): the
// SPI RXNE interrupt captures EVERY clocked byte into a ring buffer independent of
// what the foreground is doing, and spi_link_slave_rx_byte() pops bytes for a
// software SOF-scanner (spi_frame_stream). No master handshake, no mutual stall.
void spi_link_slave_init_irq(void);

// Pop one received byte from the RX ring into *out. Returns 1 if a byte was
// available, 0 if the ring is empty. Non-blocking; call in a foreground loop.
int  spi_link_slave_rx_byte(uint8_t *out);

#endif // SPI_LINK_H
