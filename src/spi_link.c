// spi_link.c — board-to-board SPI1 link driver (polled, full-duplex). See
// spi_link.h. SPI1 on GPIOA AF5 (board.h LINK_*): SCK PA5, NSS PA4 (hardware),
// MISO PA6, MOSI PA7, DATA_READY PA3. See board.h for pin-selection rationale.
//
// Design notes:
//  - Polled, not DMA: the SPI1 DMAMUX TX/RX request-source IDs are not enumerated
//    in the vendored SDK, and guessing one yields a silently-never-firing DMA. This
//    uses TXE/RXNE polling; a DMA path can replace it once the IDs are confirmed.
//  - Integrity is the software CRC in spi_frame.c (host-tested, KAT 0x29B1), not the
//    SPI hardware CRC engine. The link layer moves bytes; the frame codec validates.
//  - Mode 3 (CPOL=1, CPHA=2edge), MSB-first, 16-bit frames, matching the ST7789 SPI2
//    clocking convention. Master and slave must agree on all four.
//  - CS: the master drives PA4 as a plain GPIO (one pulse per slot) so a bit-slip
//    cannot propagate past one frame. The slave uses PA4 as the hardware NSS input
//    for automatic per-frame re-sync.
#include "spi_link.h"
#include "board.h"
#include "ch32h417_conf.h"
#include "ch32h417_port.h"   // WCH_IRQ attribute for SPI1_IRQHandler
#include "timebase_v5f.h"    // wrap-safe µs budget for waits

// Per-byte wait budget for the master. Every flag (TXE/RXNE/BSY) is driven by the
// master's own clock, so a healthy slot completes in microseconds; a flag that
// never comes means the block stopped clocking (e.g. MODF auto-cleared SPE/MSTR,
// see spi_link_master_recover). 2 ms is ~1000× the real per-byte time at the
// bring-up prescaler, so it never trips normally but bounds a wedge to a finite
// stall. Diagnostics in spi_link_master_wedges.
#define LINK_MASTER_WAIT_US  2000u

// Incremented each time a master flag-wait times out (wedge caught, slot aborted).
// 0 on a healthy link. extern-visible for the oracle/LED.
volatile uint32_t spi_link_master_wedges;

// Wait for STATR `bit` to reach `want_set` (1 = set, 0 = clear) on the master SPI,
// bounded by LINK_MASTER_WAIT_US. Returns 1 if it arrived, 0 on timeout (caller
// aborts + recovers). Wrap-safe. Reads STATR directly rather than via the HAL so the
// per-word hot loop stays inlinable.
static int link_master_wait_bit(uint16_t bit, int want_set)
{
    uint32_t t0 = timebase_v5f_us();
    while (!!(LINK_SPI->STATR & bit) != !!want_set) {
        if ((timebase_v5f_us() - t0) >= LINK_MASTER_WAIT_US) {
            spi_link_master_wedges++;
            return 0;
        }
    }
    return 1;
}

// SPI1 baud is f_PCLK/prescaler. Mode1 = /4; the SPI1 PB clock is ~100 MHz under the
// V5F profile, so this is ~25 MHz SCK. That is the upper end of the signal-integrity
// headroom for a board-to-board jumper: if spi_link_rx_overflows or
// spi_link_master_wedges rise off zero, drop to Mode2 (/8) or Mode3 (/16).
#define LINK_SPI_PRESCALER  SPI_BaudRatePrescaler_Mode1

// ── shared pin/clock bring-up ───────────────────────────────────────────────
static void link_clocks_on(void)
{
    RCC_HB2PeriphClockCmd(LINK_GPIO_RCC_HB2, ENABLE); // AFIO + GPIOA
    RCC_HB2PeriphClockCmd(LINK_SPI_RCC_HB2, ENABLE);  // SPI1 (HB2 bus)
}

// Configure SCK/MISO/MOSI as AF5. The AF mux handles drive direction on this IP, so
// master and slave configure the same pads as AF push-pull per the EVT convention.
static void link_spi_pins_af(void)
{
    GPIO_InitTypeDef g = {0};
    g.GPIO_Speed = GPIO_Speed_Very_High;

    GPIO_PinAFConfig(LINK_SCK_PORT,  LINK_SCK_PINSRC,  LINK_SPI_AF);
    GPIO_PinAFConfig(LINK_MISO_PORT, LINK_MISO_PINSRC, LINK_SPI_AF);
    GPIO_PinAFConfig(LINK_MOSI_PORT, LINK_MOSI_PINSRC, LINK_SPI_AF);

    g.GPIO_Mode = GPIO_Mode_AF_PP;
    g.GPIO_Pin  = LINK_SCK_PIN | LINK_MOSI_PIN;
    GPIO_Init(LINK_SCK_PORT, &g);

    g.GPIO_Mode = GPIO_Mode_AF_PP;
    g.GPIO_Pin  = LINK_MISO_PIN;
    GPIO_Init(LINK_MISO_PORT, &g);
}

static void link_spi_configure(uint16_t mode, uint16_t nss)
{
    SPI_InitTypeDef spi = {0};
    spi.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode              = mode;
    spi.SPI_DataSize          = SPI_DataSize_16b;  // 2 bytes/transfer; see link_xfer_slot_master
    spi.SPI_CPOL              = SPI_CPOL_High;   // Mode 3, matches ST7789 SPI2
    spi.SPI_CPHA              = SPI_CPHA_2Edge;
    spi.SPI_NSS               = nss;
    spi.SPI_BaudRatePrescaler = LINK_SPI_PRESCALER; // ignored in slave mode
    spi.SPI_FirstBit          = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial     = 7;               // unused (SW CRC); harmless default
    SPI_Init(LINK_SPI, &spi);
    SPI_Cmd(LINK_SPI, ENABLE);
}

// Master slot transfer: the master controls the clock, so load-then-receive per
// word is correct (writing DATAR starts the clock; RXNE completes it). Bounded: on
// timeout it aborts the slot and returns 0 so the caller can recover.
//
// Each transfer moves one 16-bit word, so a 32-byte slot is 16 words (16 TXE/RXNE
// waits). Bytes pack MSB-first into the word (tx[2i] -> bits 15:8, tx[2i+1] ->
// bits 7:0), keeping the on-wire byte order MSB-first as the SOF-scanner expects.
// SPI_LINK_SLOT is fixed at 32 (even), so there is no odd trailing byte.
static int link_xfer_slot_master(const uint8_t *tx, uint8_t *rx)
{
    for (uint32_t i = 0; i < SPI_LINK_SLOT; i += 2) {
        uint16_t out = tx ? (uint16_t)((tx[i] << 8) | tx[i + 1]) : 0x0000u;
        if (!link_master_wait_bit(SPI_STATR_TXE, 1)) return 0;
        LINK_SPI->DATAR = out;                              /* starts the clock */
        if (!link_master_wait_bit(SPI_STATR_RXNE, 1)) return 0;
        uint16_t in = LINK_SPI->DATAR;                      /* clears RXNE */
        if (rx) { rx[i] = (uint8_t)(in >> 8); rx[i + 1] = (uint8_t)in; }
    }
    return 1;
}

// Slave slot transfer: the slave does not control the clock, so its TX word must be
// in the data register before the master clocks it out on MISO. Loading after the
// clock starts (master-style) ships stale DR contents and the master sees a garbage
// echo. Pre-load word 0, then per word wait RXNE (read) and refill the next TX word
// ahead of its clock. 16-bit words, MSB-first packing (see link_xfer_slot_master);
// waits are unbounded since the master drives the clock.
static void link_xfer_slot_slave(const uint8_t *tx, uint8_t *rx)
{
    // Prime MISO with word 0 before the master's first clock edge.
    while (!(LINK_SPI->STATR & SPI_STATR_TXE)) { }
    LINK_SPI->DATAR = tx ? (uint16_t)((tx[0] << 8) | tx[1]) : 0x0000u;

    for (uint32_t i = 0; i < SPI_LINK_SLOT; i += 2) {
        // Refill the next outgoing word once the shift register frees up, before
        // blocking on this word's RXNE (TXE for word i+2 comes up as word i finishes).
        if (i + 2 < SPI_LINK_SLOT) {
            while (!(LINK_SPI->STATR & SPI_STATR_TXE)) { }
            LINK_SPI->DATAR = tx ? (uint16_t)((tx[i + 2] << 8) | tx[i + 3]) : 0x0000u;
        }
        while (!(LINK_SPI->STATR & SPI_STATR_RXNE)) { }
        uint16_t in = LINK_SPI->DATAR;
        if (rx) { rx[i] = (uint8_t)(in >> 8); rx[i + 1] = (uint8_t)in; }
    }
}

// ── master (USB-host board) ─────────────────────────────────────────────────
static inline void link_cs_high(void) { GPIO_SetBits(LINK_NSS_PORT, LINK_NSS_PIN); }
static inline void link_cs_low(void)  { GPIO_ResetBits(LINK_NSS_PORT, LINK_NSS_PIN); }

void spi_link_master_init(void)
{
    link_clocks_on();
    link_spi_pins_af();

    // CS (PA4) as a plain push-pull GPIO, idle high. PA4 is the SPI1 hardware NSS
    // pin, but the master drives it as a software-managed GPIO (one pulse per slot);
    // the slave uses PA4 as its HW-NSS input, so each CS edge gives the slave
    // automatic per-frame select + bit re-alignment.
    GPIO_InitTypeDef g = {0};
    g.GPIO_Speed = GPIO_Speed_Very_High;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Pin   = LINK_NSS_PIN;
    GPIO_Init(LINK_NSS_PORT, &g);
    link_cs_high();

    // DATA_READY (PA3) as a floating input — the slave drives it.
    g.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    g.GPIO_Pin  = LINK_DRDY_PIN;
    GPIO_Init(LINK_DRDY_PORT, &g);

    // Master: software NSS (we manage CS ourselves on PA4), internal NSS held
    // high so the SPI block stays master and doesn't drop to slave on a low NSS.
    link_spi_configure(SPI_Mode_Master, SPI_NSS_Soft);
    SPI_NSSInternalSoftwareConfig(LINK_SPI, SPI_NSSInternalSoft_Set);
}

// Recover a wedged master. On this IP a MODF (mode fault) auto-clears SPE and MSTR,
// silently demoting the master to a disabled slave so every flag-wait times out.
// Re-asserting CS high, disabling + re-enabling the block, and re-forcing internal
// NSS high (so it stays master) restores clocking. Idempotent; safe after any
// aborted slot.
static void spi_link_master_recover(void)
{
    link_cs_high();
    SPI_Cmd(LINK_SPI, DISABLE);
    SPI_NSSInternalSoftwareConfig(LINK_SPI, SPI_NSSInternalSoft_Set);
    SPI_Cmd(LINK_SPI, ENABLE);
}

void spi_link_master_exchange(const uint8_t tx[SPI_LINK_SLOT],
                              uint8_t rx[SPI_LINK_SLOT])
{
    link_cs_low();
    if (!link_xfer_slot_master(tx, rx)) {
        // A flag-wait timed out mid-slot: abort and recover so the next slot can
        // proceed. The dropped slot is harmless — the blob is re-sent periodically
        // and the SOF-scanner re-aligns downstream.
        spi_link_master_recover();
        return;
    }
    // Wait for the shift register to empty before raising CS so the last bit fully
    // clocks out. Bounded: if BSY never clears, recover rather than hang.
    if (!link_master_wait_bit(SPI_STATR_BSY, 0)) {
        spi_link_master_recover();
        return;
    }
    link_cs_high();
}

int spi_link_master_drdy(void)
{
    return GPIO_ReadInputDataBit(LINK_DRDY_PORT, LINK_DRDY_PIN) ? 1 : 0;
}

// ── slave (USB-device board) ────────────────────────────────────────────────
void spi_link_slave_init(void)
{
    link_clocks_on();
    link_spi_pins_af();

    // DATA_READY (PA3) as a push-pull output, deasserted (low).
    GPIO_InitTypeDef g = {0};
    g.GPIO_Speed = GPIO_Speed_Very_High;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Pin   = LINK_DRDY_PIN;
    GPIO_Init(LINK_DRDY_PORT, &g);
    GPIO_ResetBits(LINK_DRDY_PORT, LINK_DRDY_PIN);

    // Software NSS held selected (SSM=1, SSI=0): the slave always-listens and shifts
    // purely on SCK. A hardware-NSS slave (SSM=0) only shifts while its NSS pin is
    // low, but PA4 is not AF-routed to SPI1_NSS here. PA4 is unused in this mode.
    link_spi_configure(SPI_Mode_Slave, SPI_NSS_Soft);
    SPI_NSSInternalSoftwareConfig(LINK_SPI, SPI_NSSInternalSoft_Reset); // internal NSS low = selected
}

void spi_link_slave_exchange(const uint8_t tx[SPI_LINK_SLOT],
                             uint8_t rx[SPI_LINK_SLOT])
{
    // The slave pre-loads MISO before each clock edge (link_xfer_slot_slave primes
    // byte 0 and refills ahead). tx must be fully staged on entry.
    link_xfer_slot_slave(tx, rx);
}

void spi_link_slave_set_drdy(int asserted)
{
    if (asserted) GPIO_SetBits(LINK_DRDY_PORT, LINK_DRDY_PIN);
    else          GPIO_ResetBits(LINK_DRDY_PORT, LINK_DRDY_PIN);
}

// --- Slave -> master telemetry return slot (staged onto MISO by the RXNE ISR) --
// Double-buffered: the publisher fills s_telem[next] then flips s_telem_cur; the ISR
// only reads s_telem[s_telem_cur], never a torn slot. The flip is a single-byte
// store, atomic w.r.t. the ISR since both run on the V5F core.
static volatile uint8_t s_telem[2][SPI_LINK_SLOT];
static volatile uint8_t s_telem_cur;     // which buffer the ISR reads
static volatile uint8_t s_telem_idx;     // next byte to send from the current slot
static volatile uint8_t s_telem_armed;   // 0 until the first publish

void spi_link_slave_set_telem(const uint8_t slot[SPI_LINK_SLOT])
{
    uint8_t next = (uint8_t)(s_telem_cur ^ 1u);
    for (uint32_t i = 0; i < SPI_LINK_SLOT; i++) s_telem[next][i] = slot[i];
    s_telem_cur = next;
    s_telem_armed = 1;
}

// ── Slave RX, interrupt-driven (stream-capable) ─────────────────────────────
// Lock-free SPSC ring: the RXNE ISR (single writer) pushes each clocked byte; the
// foreground spi_link_slave_rx_byte (single reader) pops. Power-of-two size for
// cheap masking. On overflow the ISR drops the byte (bumps a diag counter); the
// SOF-scanner downstream recovers alignment, so a drop costs one frame.
#define LINK_RX_RING_SZ   512u           // power of two
#define LINK_RX_RING_MASK (LINK_RX_RING_SZ - 1u)
static volatile uint8_t  s_rx_ring[LINK_RX_RING_SZ];
static volatile uint16_t s_rx_head;      // ISR writes
static volatile uint16_t s_rx_tail;      // foreground reads
volatile uint32_t spi_link_rx_overflows; // diag

void SPI1_IRQHandler(void) WCH_IRQ;

void spi_link_slave_init_irq(void)
{
    link_clocks_on();
    link_spi_pins_af();

    // DATA_READY (PA3) as a push-pull output, deasserted (reverse channel unused).
    GPIO_InitTypeDef g = {0};
    g.GPIO_Speed = GPIO_Speed_Very_High;
    g.GPIO_Mode  = GPIO_Mode_Out_PP;
    g.GPIO_Pin   = LINK_DRDY_PIN;
    GPIO_Init(LINK_DRDY_PORT, &g);
    GPIO_ResetBits(LINK_DRDY_PORT, LINK_DRDY_PIN);

    // Slave, software-NSS held selected: always-listen, shift purely on SCK. The
    // downstream SOF-scanner (spi_frame_stream) provides framing from the byte
    // stream, robust to misalignment, so no per-frame NSS framing is needed.
    link_spi_configure(SPI_Mode_Slave, SPI_NSS_Soft);
    SPI_NSSInternalSoftwareConfig(LINK_SPI, SPI_NSSInternalSoft_Reset);

    s_rx_head = 0;
    s_rx_tail = 0;
    spi_link_rx_overflows = 0;

    // Per-word RX interrupt. IRQn>31 are core-allocated; route SPI1_IRQn to V5F
    // before enabling (same as the USB ISRs).
    SPI_I2S_ITConfig(LINK_SPI, SPI_I2S_IT_RXNE, ENABLE);
    NVIC_SetAllocateIRQ(SPI1_IRQn, Core_ID_V5F);
    NVIC_EnableIRQ(SPI1_IRQn);
}

volatile uint32_t spi_link_isr_entries;  // diag: ISR entry count

void SPI1_IRQHandler(void)
{
    spi_link_isr_entries++;
    // RXNE: a 16-bit word was clocked in. Read DR (clears RXNE) and push both bytes
    // to the ring MSB-first, matching the master's word packing so the SOF-scanner
    // sees an MSB-first byte stream.
    if (LINK_SPI->STATR & SPI_STATR_RXNE) {
        uint16_t w = LINK_SPI->DATAR;
        uint8_t pair[2] = { (uint8_t)(w >> 8), (uint8_t)w };
        for (uint32_t k = 0; k < 2; k++) {
            uint16_t nh = (uint16_t)((s_rx_head + 1u) & LINK_RX_RING_MASK);
            if (nh != s_rx_tail) {
                s_rx_ring[s_rx_head] = pair[k];
                s_rx_head = nh;
            } else {
                spi_link_rx_overflows++; // ring full, drop (SOF-scan recovers)
            }
        }
    }
    // Stage the next telemetry word onto MISO for the return slot, wrapping so the
    // slot repeats continuously. Two bytes per word, MSB-first; s_telem_idx tracks
    // the byte position and advances by 2 (SPI_LINK_SLOT is even).
    if (s_telem_armed && (LINK_SPI->STATR & SPI_STATR_TXE)) {
        uint8_t hi = s_telem[s_telem_cur][s_telem_idx];
        uint8_t lo = s_telem[s_telem_cur][s_telem_idx + 1u];
        LINK_SPI->DATAR = (uint16_t)((hi << 8) | lo);
        s_telem_idx = (uint8_t)((s_telem_idx + 2u) % SPI_LINK_SLOT);
    }
}

int spi_link_slave_rx_byte(uint8_t *out)
{
    if (s_rx_tail == s_rx_head) return 0;       // empty
    *out = s_rx_ring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1u) & LINK_RX_RING_MASK);
    return 1;
}
