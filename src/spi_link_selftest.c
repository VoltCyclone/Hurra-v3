// spi_link_selftest.c — board-to-board SPI link bench harness. See header.
// Built only under -DSPI_LINK_SELFTEST.
//
// TEST PROTOCOL (full-duplex echo, hardware-NSS framed):
//   Master (board A, SPI master): every round it CLOCKS one 32-byte exchange.
//     TX = a PING frame (TYPE=PING, SEQ=counter, payload = 4-byte counter LE).
//     RX = whatever the slave had staged = the slave's echo of the PREVIOUS ping.
//     Master checks: RX unpacks OK (SOF+LEN+CRC), TYPE==ECHO, and the echoed
//     counter == the ping counter it sent one round earlier. One mismatch =>
//     error. The very first round has no prior echo, so it's skipped in scoring.
//   Slave (board B, SPI slave): it stages an ECHO frame, then blocks in the
//     exchange until the master clocks it. On each completed exchange it unpacks
//     the received PING, and re-stages ECHO(TYPE=ECHO, SEQ=ping.seq, payload =
//     the same counter bytes) for the NEXT exchange. It scores its own CRC checks.
//
// LED READOUT (V5F = PC3), no logic analyzer needed:
//   - healthy: slow ~1 Hz blink (toggles once per LED_HEALTHY_TOGGLE_ROUNDS good rounds)
//   - errors / no link: fast ~8 Hz blink
//   Both roles use the same convention, so a glance at either board tells you.
//
// The master free-runs at a modest rate (a short delay per round) so the LED is
// watchable and the slave always has time to re-stage. This is a correctness +
// signal-integrity gate, NOT the 8 kHz throughput test (that comes with DMA).
#include "spi_link_selftest.h"

#ifdef SPI_LINK_SELFTEST

#include "spi_link.h"
#include "spi_frame.h"
#include "led.h"
#include "timebase_v5f.h"   // timebase_v5f_delay_ms
#include "ch32h417_conf.h"

// Frame TYPE tags for the test (high bit clear = these are test frames, distinct
// from the real ICC record tags reused by the hot path).
#define ST_TYPE_PING  0x70u
#define ST_TYPE_ECHO  0x71u

#define ST_ROUND_DELAY_MS         2u    // master pacing: ~500 Hz, easily watchable
#define LED_HEALTHY_TOGGLE_ROUNDS 250u  // ~1 Hz toggle at 500 Hz round rate
#define LED_FAULT_TOGGLE_ROUNDS    30u  // ~8 Hz toggle => visibly faster "error" blink

// Pack a 32-bit counter little-endian into a payload buffer.
static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v);       p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// Drive the LED from a rolling health view: while errors keep happening the LED
// blinks fast; while rounds stay clean it blinks slow. `good_streak` is reset to
// 0 by the caller on any error; the harness toggles at a cadence chosen by
// whether we're currently in a healthy streak.
static void led_update(uint32_t round, int healthy)
{
    uint32_t period = healthy ? LED_HEALTHY_TOGGLE_ROUNDS : LED_FAULT_TOGGLE_ROUNDS;
    if ((round % period) == 0) led_toggle();
}

#ifdef SPI_LINK_ROLE_MASTER
static void run_master(void)
{
    spi_link_master_init();

    uint8_t  tx[SPI_LINK_SLOT];
    uint8_t  rx[SPI_LINK_SLOT];
    uint8_t  pay[4];
    uint32_t counter   = 0;     // ping counter, increments each round
    uint32_t prev_sent = 0;     // counter we expect echoed back this round
    int      have_prev = 0;     // first round has no prior echo to check
    uint32_t round     = 0;
    int      healthy   = 0;     // becomes 1 after the first good echo

    for (;;) {
        put_u32(pay, counter);
        spi_frame_pack(tx, ST_TYPE_PING, (uint8_t)counter, pay, sizeof pay);

        spi_link_master_exchange(tx, rx);

        // Score the echo of the PREVIOUS ping (skip round 0 — nothing staged yet).
        if (have_prev) {
            uint8_t type, seq, len; const uint8_t *p;
            int ok = (spi_frame_unpack(rx, &type, &seq, &p, &len) == SPI_FRAME_OK)
                     && type == ST_TYPE_ECHO
                     && len  == sizeof pay
                     && get_u32(p) == prev_sent;
            healthy = ok;
        }

        prev_sent = counter;
        have_prev = 1;
        counter++;
        led_update(round++, healthy);

        timebase_v5f_delay_ms(ST_ROUND_DELAY_MS);
    }
}
#else
static void run_slave(void)
{
    spi_link_slave_init();

    uint8_t  tx[SPI_LINK_SLOT];
    uint8_t  rx[SPI_LINK_SLOT];
    uint8_t  pay[4];
    uint32_t round   = 0;
    int      healthy = 0;

    // Stage an initial (empty) echo so the master's first exchange has something
    // to clock out of us; it won't be scored (master skips round 0).
    put_u32(pay, 0);
    spi_frame_pack(tx, ST_TYPE_ECHO, 0, pay, sizeof pay);

    for (;;) {
        // Block until the master clocks the exchange; rx gets this round's PING.
        spi_link_slave_exchange(tx, rx);

        uint8_t type, seq, len; const uint8_t *p;
        int ok = (spi_frame_unpack(rx, &type, &seq, &p, &len) == SPI_FRAME_OK)
                 && type == ST_TYPE_PING
                 && len  == sizeof pay;
        healthy = ok;

        // Re-stage the echo of THIS ping for the next exchange.
        if (ok) {
            put_u32(pay, get_u32(p));
            spi_frame_pack(tx, ST_TYPE_ECHO, seq, pay, sizeof pay);
        }
        led_update(round++, healthy);
    }
}
#endif

void spi_link_selftest_run(void)
{
#ifdef SPI_LINK_ROLE_MASTER
    run_master();
#else
    run_slave();
#endif
}

#endif // SPI_LINK_SELFTEST
