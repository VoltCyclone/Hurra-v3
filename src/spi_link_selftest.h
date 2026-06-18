// spi_link_selftest.h — board-to-board SPI link bench harness (step-2 gate).
// Built only when -DSPI_LINK_SELFTEST is set (Makefile: SELFTEST=master|slave).
// Runs a continuous full-duplex echo test over the SPI1 link and shows pass/fail
// on the V5F LED (PC3): slow ~1 Hz blink = healthy, fast ~8 Hz = errors/no link.
// NEVER RETURNS — this is a dedicated test firmware, not part of the relay.
#ifndef SPI_LINK_SELFTEST_H
#define SPI_LINK_SELFTEST_H

#ifdef SPI_LINK_SELFTEST
// Run the role selected at build time (-DSPI_LINK_ROLE_MASTER => master, else
// slave). Call after the V5F clocks, timebase, and LED are up. Does not return.
void spi_link_selftest_run(void);
#endif

#endif // SPI_LINK_SELFTEST_H
