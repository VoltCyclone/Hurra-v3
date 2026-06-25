// spi_link.h — board-to-board SPI link driver (SPI1 on GPIOA AF5, see board.h
// LINK_*: SCK PA5, NSS PA4, MISO PA6, MOSI PA7, DATA_READY PA3). Carries the fixed
// 32-byte hot-path slot (src/spi_frame.h) between the USB-host board (SPI master)
// and the USB-device board (SPI slave).
//
// Forward path (master -> slave descriptor/report stream): the master clocks polled
// full-duplex slots; the slave receives them either polled (spi_link_slave_exchange,
// paced isolated frames) or interrupt-driven (spi_link_slave_init_irq + SOF-scanner,
// for the continuous descriptor burst). The reverse channel (DATA_READY on PA3)
// signals enumeration state.
//
// MMIO (WCH StdPeriph), bench-gated. The wire format it carries is host-tested in
// test/spi_frame_test.c.
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

// Count of master flag-waits that timed out (wedge caught, slot aborted, block
// recovered). 0 on a healthy link; a rising value localizes a frozen heartbeat to
// the master's SPI block stalling.
extern volatile uint32_t spi_link_master_wedges;

// ── Slave (USB-device board) ────────────────────────────────────────────────
// Init SPI1 as slave on PA5/6/7 (AF5), software-NSS held selected, and PA3
// (DATA_READY) as a GPIO output (deasserted). Must be called once before any
// exchange.
void spi_link_slave_init(void);

// Blocking full-duplex exchange from the slave side: waits for the master's clock
// and moves SPI_LINK_SLOT bytes each way (sending tx[], receiving into rx[]). The
// slave must have tx staged before the master starts clocking, so the caller
// arms the return slot (tx) here and reads the received slot from rx afterward.
// tx/rx may alias; either may be NULL.
void spi_link_slave_exchange(const uint8_t tx[SPI_LINK_SLOT],
                             uint8_t rx[SPI_LINK_SLOT]);

// Drive DATA_READY (PA3) to tell the master reverse-path data is pending (or not).
void spi_link_slave_set_drdy(int asserted);

// Publish a 32-byte telemetry slot for the IRQ slave to stage on MISO. The RXNE ISR
// cycles these bytes onto the data register (two per clocked word, repeating the
// slot), so the master sees a continuous stream on the return path. Double-buffered:
// the copy is atomic w.r.t. the ISR. Pass a slot built with spi_frame_pack.
void spi_link_slave_set_telem(const uint8_t slot[SPI_LINK_SLOT]);

// ── Slave RX, interrupt-driven (stream-capable) ─────────────────────────────
// The polled spi_link_slave_exchange only reliably catches paced isolated frames; it
// loses byte alignment under a continuous frame stream. For streaming RX, init the
// slave with spi_link_slave_init_irq(): the RXNE interrupt captures every clocked
// byte into a ring buffer, and spi_link_slave_rx_byte() pops bytes for a software
// SOF-scanner (spi_frame_stream). No master handshake, no mutual stall.
void spi_link_slave_init_irq(void);

// Pop one received byte from the RX ring into *out. Returns 1 if a byte was
// available, 0 if the ring is empty. Non-blocking; call in a foreground loop.
int  spi_link_slave_rx_byte(uint8_t *out);

#endif // SPI_LINK_H
