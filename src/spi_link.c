// spi_link.c — board-to-board SPI1 link driver (polled, full-duplex). See
// spi_link.h. SPI1 on GPIOA AF5 (board.h LINK_*): SCK PA5, NSS PA4 (hardware),
// MISO PA6, MOSI PA7, DATA_READY PA3. (Pins chosen from what the nanoCH32H417
// actually breaks out — the A4..A7 group is four adjacent header pins with real
// hardware NSS; see board.h for the full rationale.)
//
// DESIGN NOTES
//  - POLLED, not DMA. The hot-path 8 kHz sustained design wants a free-running
//    DMA ring, but the SPI1 DMAMUX TX/RX request-source IDs are NOT enumerated in
//    the vendored SDK (grep DMA_Request* / SPI1 in vendor/wch/Peripheral/inc —
//    nothing). Guessing a hardware request number is the classic "DMA silently
//    never fires" bug. So this uses TXE/RXNE polling — well-understood flags — and
//    the DMA path can drop in later once the request IDs are confirmed on the bench.
//  - Integrity is the SOFTWARE CRC in spi_frame.c (already host-tested, KAT
//    0x29B1), NOT the SPI hardware CRC engine. The HW CRC is a separate
//    bench-verify item (plan Q3); the slot already carries its own CRC16, so the
//    link layer just moves bytes and the frame codec validates them.
//  - Mode: CPOL=1/CPHA=2edge (Mode 3), MSB-first, 8-bit — matches the ST7789 SPI2
//    setup so both SPI blocks on the board use one clocking convention. Master
//    and slave MUST agree; both are set here.
//  - CS: the master drives PA4 as a plain GPIO (one pulse per slot) so a bit-slip
//    can't propagate past a single frame (plan §2 resync). The slave uses PA4 as
//    the hardware NSS input (real SPI1 HW-NSS) for automatic per-frame re-sync.
#include "spi_link.h"
#include "board.h"
#include "ch32h417_conf.h"
#include "ch32h417_port.h"   // WCH_IRQ attribute for SPI1_IRQHandler
#include "timebase_v5f.h"    // timebase_v5f_us() — wrap-safe µs budget for waits

// Per-byte wait budget for the MASTER. In master mode every flag (TXE/RXNE/BSY)
// is driven by the master's OWN clock, so a healthy slot completes in microseconds;
// a flag that never comes means the SPI block stopped clocking (e.g. MODF auto-
// cleared SPE/MSTR — see spi_link_master_recover). 2 ms is ~1000× the real per-byte
// time at the bring-up prescaler, so it never trips in normal operation but bounds a
// genuine wedge to a finite stall instead of a permanent hang (the gate-4b "frozen
// heartbeat, counter 0x00" failure). Diagnostics live in spi_link_master_wedges.
#define LINK_MASTER_WAIT_US  2000u

// DIAG: incremented each time a master flag-wait times out (a wedge was caught and
// the slot aborted). Stays 0 on a healthy link. extern-visible for the oracle/LED.
volatile uint32_t spi_link_master_wedges;

// Wait for `flag` to reach `want` on the master SPI, bounded by LINK_MASTER_WAIT_US.
// Returns 1 if the flag arrived, 0 on timeout (caller aborts + recovers). Uses the
// wrap-safe TIM9 µs counter so it is correct across the 32-bit wrap.
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

// SPI1 baud is f_PCLK/prescaler. Mode4 = /32. Under the V5F clock profile the
// SPI1 PB clock is on the order of ~100 MHz, so /32 lands in the low-MHz range —
// a deliberately SLOW, signal-integrity-safe rate for first bring-up. Raise the
// prescaler toward the 25 MHz target (plan §4) once the wire is proven clean.
#define LINK_SPI_PRESCALER  SPI_BaudRatePrescaler_Mode4

// ── shared pin/clock bring-up ───────────────────────────────────────────────
static void link_clocks_on(void)
{
    RCC_HB2PeriphClockCmd(LINK_GPIO_RCC_HB2, ENABLE); // AFIO + GPIOA
    RCC_HB2PeriphClockCmd(LINK_SPI_RCC_HB2, ENABLE);  // SPI1 (HB2 bus)
}

// Configure SCK/MISO/MOSI as AF5. Direction of each pad is fixed by SPI role, but
// on this IP the AF mux handles drive direction, so master and slave configure
// the same three pads as AF push-pull (SCK/MOSI) / AF input (MISO). The slave's
// MISO is the only driven output but AF_PP on both is the EVT convention.
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

// Blocking one-byte full-duplex shift for the SLAVE side (unbounded: the slave is
// SUPPOSED to block until the master clocks it — there is no local timeout that
// makes sense). Returns the byte clocked in. TXE-wait, send, RXNE-wait, receive.
static uint8_t link_xfer_byte(uint8_t out)
{
    while (SPI_I2S_GetFlagStatus(LINK_SPI, SPI_I2S_FLAG_TXE) == RESET) { }
    SPI_I2S_SendData(LINK_SPI, out);
    while (SPI_I2S_GetFlagStatus(LINK_SPI, SPI_I2S_FLAG_RXNE) == RESET) { }
    return (uint8_t)SPI_I2S_ReceiveData(LINK_SPI);
}

// MASTER slot transfer: the master controls the clock, so load-then-receive per
// byte is correct (SendData starts the clock; RXNE completes it). BOUNDED — every
// wait has a finite budget; on timeout (the block stopped clocking) it aborts the
// slot and returns 0 so the caller can recover instead of hanging forever.
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

// SLAVE slot transfer: the slave does NOT control the clock, so its TX byte must
// already be in the data register BEFORE the master clocks it out on MISO. The
// master-style "wait TXE, then SendData" loads the byte one clock too late and
// the slave ships stale DR contents on MISO (master sees a garbage echo). Fix:
// PRE-LOAD byte 0, then per byte wait RXNE (read), and refill the NEXT TX byte
// ahead of its clock.
static void link_xfer_slot_slave(const uint8_t *tx, uint8_t *rx)
{
    // Prime MISO with byte 0 before the master's first clock edge.
    while (SPI_I2S_GetFlagStatus(LINK_SPI, SPI_I2S_FLAG_TXE) == RESET) { }
    SPI_I2S_SendData(LINK_SPI, tx ? tx[0] : 0x00u);

    for (uint32_t i = 0; i < SPI_LINK_SLOT; i++) {
        // Refill the NEXT outgoing byte as soon as the shift register frees up,
        // so it's staged before the master clocks it (do this BEFORE blocking on
        // this byte's RXNE — TXE for byte i+1 comes up as byte i finishes).
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

    // CS (PA4) as a plain push-pull GPIO, idle high. PA4 *is* the SPI1 hardware
    // NSS pin, but the master drives it as a software-managed GPIO (one pulse per
    // slot) — the slave uses PA4 as its HW-NSS input, so the master's CS edge
    // gives the slave automatic per-frame select + bit re-alignment.
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
// silently demoting the master to a disabled slave: SendData then writes DR but
// nothing shifts and every flag-wait times out forever. Re-asserting CS high, then
// disabling + re-enabling the block (and re-forcing internal NSS high so it stays
// master) restores clocking for the next slot. Cheap and idempotent — safe to call
// after any aborted slot.
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
        // A flag-wait timed out mid-slot: the block stopped clocking. Abort this
        // slot and recover so the next one (and the relay heartbeat) can proceed
        // instead of spinning forever. The dropped slot is fine — the descriptor
        // blob is re-sent periodically and the SOF-scanner re-aligns downstream.
        spi_link_master_recover();
        return;
    }
    // Drain the final RX and wait for the shift register to empty before raising CS,
    // so the last bit fully clocks out. Bounded: if BSY never clears the block has
    // wedged — recover rather than hang.
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

    // BRING-UP: software NSS held SELECTED (SSM=1, SSI=0). A hardware-NSS slave
    // (SSM=0) only shifts while its NSS *pin* is driven low, but PA4 was never
    // AF-routed to SPI1_NSS here, so it sat permanently deselected and never
    // clocked a byte. Software-NSS-selected makes the slave always-listen and
    // shift purely on SCK — the standard bring-up mode. (Hardware-NSS per-frame
    // resync is a later refinement; once bytes flow we can AF-config PA4 and
    // switch back to SPI_NSS_Hard.) The PA4 wire is unused in this mode.
    link_spi_configure(SPI_Mode_Slave, SPI_NSS_Soft);
    SPI_NSSInternalSoftwareConfig(LINK_SPI, SPI_NSSInternalSoft_Reset); // internal NSS low = selected
}

void spi_link_slave_exchange(const uint8_t tx[SPI_LINK_SLOT],
                             uint8_t rx[SPI_LINK_SLOT])
{
    // Slave clocks off the master and must PRE-LOAD MISO before each clock edge —
    // link_xfer_slot_slave primes byte 0 and refills ahead. tx must be fully
    // staged on entry (it is). Loading TX after the clock starts (master-style)
    // ships stale DR bytes on MISO and the master sees a garbage echo.
    link_xfer_slot_slave(tx, rx);
}

void spi_link_slave_set_drdy(int asserted)
{
    if (asserted) GPIO_SetBits(LINK_DRDY_PORT, LINK_DRDY_PIN);
    else          GPIO_ResetBits(LINK_DRDY_PORT, LINK_DRDY_PIN);
}

// --- Slave -> master telemetry return slot (staged onto MISO by the RXNE ISR) --
// Double-buffered: the foreground writes s_telem[next] then flips s_telem_cur.
// The ISR only ever reads s_telem[s_telem_cur], so it never sees a torn slot.
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
// Lock-free SPSC ring: the SPI1 RXNE ISR (single writer) pushes each clocked byte;
// the foreground spi_link_slave_rx_byte (single reader) pops. Power-of-two size for
// cheap masking. On overflow the ISR drops the byte (bumps a diag counter) — the
// SOF-scanner downstream recovers alignment, so a dropped byte just costs one frame.
#define LINK_RX_RING_SZ   512u           // power of two
#define LINK_RX_RING_MASK (LINK_RX_RING_SZ - 1u)
static volatile uint8_t  s_rx_ring[LINK_RX_RING_SZ];
static volatile uint16_t s_rx_head;      // ISR writes
static volatile uint16_t s_rx_tail;      // foreground reads
volatile uint32_t spi_link_rx_overflows; // diag (extern-visible if needed)

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

    // Slave, software-NSS held selected: always-listen, shift purely on SCK. No
    // per-frame NSS framing needed — the downstream SOF-scanner (spi_frame_stream)
    // provides framing from the byte stream, which is robust to any misalignment.
    link_spi_configure(SPI_Mode_Slave, SPI_NSS_Soft);
    SPI_NSSInternalSoftwareConfig(LINK_SPI, SPI_NSSInternalSoft_Reset);

    s_rx_head = 0;
    s_rx_tail = 0;
    spi_link_rx_overflows = 0;

    // Per-byte RX interrupt. IRQn>31 are core-allocated; the ISR runs on V5F, so
    // route SPI1_IRQn to V5F before enabling (same as the USB ISRs).
    SPI_I2S_ITConfig(LINK_SPI, SPI_I2S_IT_RXNE, ENABLE);
    NVIC_SetAllocateIRQ(SPI1_IRQn, Core_ID_V5F);
    NVIC_EnableIRQ(SPI1_IRQn);
}

volatile uint32_t spi_link_isr_entries;  // DIAG: ISR entry count (proves IRQ fires)

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
            spi_link_rx_overflows++;     // ring full — drop (SOF-scan recovers)
        }
    }
    // Stage the next telemetry byte onto MISO for the master's return slot. TXE is
    // up whenever the shift register can accept a byte; we feed the current slot,
    // wrapping so the slot repeats continuously. Cheap: one store per clocked byte.
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
