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
//  - Mode 3 (CPOL=1, CPHA=2edge), MSB-first, 8-bit, matching the ST7789 SPI2 setup
//    so both SPI blocks share one clocking convention. Master and slave must agree.
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

// Wait for `flag` to reach `want` on the master SPI, bounded by LINK_MASTER_WAIT_US.
// Returns 1 if the flag arrived, 0 on timeout (caller aborts + recovers). Wrap-safe.
static int link_master_wait(uint16_t flag, FlagStatus want)
{
    uint32_t t0 = timebase_v5f_us();
    while (SPI_I2S_GetFlagStatus(LINK_SPI, flag) != want) {
        if ((timebase_v5f_us() - t0) >= LINK_MASTER_WAIT_US) {
            spi_link_master_wedges++;
            return 0;
        }
    }
    return 1;
}

// SPI1 baud is f_PCLK/prescaler. Mode4 = /32. The SPI1 PB clock is ~100 MHz under
// the V5F profile, so /32 lands in the low-MHz range — a deliberately slow,
// signal-integrity-safe rate for bring-up. Raise once the wire is proven clean.
#define LINK_SPI_PRESCALER  SPI_BaudRatePrescaler_Mode4

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
    spi.SPI_DataSize          = SPI_DataSize_8b;
    spi.SPI_CPOL              = SPI_CPOL_High;   // Mode 3, matches ST7789 SPI2
    spi.SPI_CPHA              = SPI_CPHA_2Edge;
    spi.SPI_NSS               = nss;
    spi.SPI_BaudRatePrescaler = LINK_SPI_PRESCALER; // ignored in slave mode
    spi.SPI_FirstBit          = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial     = 7;               // unused (SW CRC); harmless default
    SPI_Init(LINK_SPI, &spi);
    SPI_Cmd(LINK_SPI, ENABLE);
}

// Blocking one-byte full-duplex shift for the slave side (unbounded: the slave
// blocks until the master clocks it, so no local timeout applies). Returns the byte
// clocked in.
static uint8_t link_xfer_byte(uint8_t out)
{
    while (SPI_I2S_GetFlagStatus(LINK_SPI, SPI_I2S_FLAG_TXE) == RESET) { }
    SPI_I2S_SendData(LINK_SPI, out);
    while (SPI_I2S_GetFlagStatus(LINK_SPI, SPI_I2S_FLAG_RXNE) == RESET) { }
    return (uint8_t)SPI_I2S_ReceiveData(LINK_SPI);
}

// Master slot transfer: the master controls the clock, so load-then-receive per
// byte is correct (SendData starts the clock; RXNE completes it). Bounded: on
// timeout it aborts the slot and returns 0 so the caller can recover.
static int link_xfer_slot_master(const uint8_t *tx, uint8_t *rx)
{
    for (uint32_t i = 0; i < SPI_LINK_SLOT; i++) {
        if (!link_master_wait(SPI_I2S_FLAG_TXE, SET)) return 0;
        SPI_I2S_SendData(LINK_SPI, tx ? tx[i] : 0x00u);
        if (!link_master_wait(SPI_I2S_FLAG_RXNE, SET)) return 0;
        uint8_t in = (uint8_t)SPI_I2S_ReceiveData(LINK_SPI);
        if (rx) rx[i] = in;
    }
    return 1;
}

// Slave slot transfer: the slave does not control the clock, so its TX byte must be
// in the data register before the master clocks it out on MISO. Loading after the
// clock starts (master-style) ships stale DR contents and the master sees a garbage
// echo. Pre-load byte 0, then per byte wait RXNE (read) and refill the next TX byte
// ahead of its clock.
static void link_xfer_slot_slave(const uint8_t *tx, uint8_t *rx)
{
    // Prime MISO with byte 0 before the master's first clock edge.
    while (SPI_I2S_GetFlagStatus(LINK_SPI, SPI_I2S_FLAG_TXE) == RESET) { }
    SPI_I2S_SendData(LINK_SPI, tx ? tx[0] : 0x00u);

    for (uint32_t i = 0; i < SPI_LINK_SLOT; i++) {
        // Refill the next outgoing byte once the shift register frees up, before
        // blocking on this byte's RXNE (TXE for byte i+1 comes up as byte i finishes).
        if (i + 1 < SPI_LINK_SLOT) {
            while (SPI_I2S_GetFlagStatus(LINK_SPI, SPI_I2S_FLAG_TXE) == RESET) { }
            SPI_I2S_SendData(LINK_SPI, tx ? tx[i + 1] : 0x00u);
        }
        while (SPI_I2S_GetFlagStatus(LINK_SPI, SPI_I2S_FLAG_RXNE) == RESET) { }
        uint8_t in = (uint8_t)SPI_I2S_ReceiveData(LINK_SPI);
        if (rx) rx[i] = in;
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
    if (!link_master_wait(SPI_I2S_FLAG_BSY, RESET)) {
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

    // Per-byte RX interrupt. IRQn>31 are core-allocated; route SPI1_IRQn to V5F
    // before enabling (same as the USB ISRs).
    SPI_I2S_ITConfig(LINK_SPI, SPI_I2S_IT_RXNE, ENABLE);
    NVIC_SetAllocateIRQ(SPI1_IRQn, Core_ID_V5F);
    NVIC_EnableIRQ(SPI1_IRQn);
}

volatile uint32_t spi_link_isr_entries;  // diag: ISR entry count

void SPI1_IRQHandler(void)
{
    spi_link_isr_entries++;
    // RXNE: a byte was clocked in. Read DR (clears RXNE) and push to the ring.
    if (SPI_I2S_GetFlagStatus(LINK_SPI, SPI_I2S_FLAG_RXNE) != RESET) {
        uint8_t b = (uint8_t)SPI_I2S_ReceiveData(LINK_SPI);
        uint16_t nh = (uint16_t)((s_rx_head + 1u) & LINK_RX_RING_MASK);
        if (nh != s_rx_tail) {
            s_rx_ring[s_rx_head] = b;
            s_rx_head = nh;
        } else {
            spi_link_rx_overflows++;     // ring full, drop (SOF-scan recovers)
        }
    }
    // Stage the next telemetry byte onto MISO for the return slot, wrapping so the
    // slot repeats continuously.
    if (s_telem_armed &&
        SPI_I2S_GetFlagStatus(LINK_SPI, SPI_I2S_FLAG_TXE) != RESET) {
        SPI_I2S_SendData(LINK_SPI, s_telem[s_telem_cur][s_telem_idx]);
        s_telem_idx = (uint8_t)((s_telem_idx + 1u) % SPI_LINK_SLOT);
    }
}

int spi_link_slave_rx_byte(uint8_t *out)
{
    if (s_rx_tail == s_rx_head) return 0;       // empty
    *out = s_rx_ring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1u) & LINK_RX_RING_MASK);
    return 1;
}
