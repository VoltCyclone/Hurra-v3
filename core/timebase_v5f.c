// core/timebase_v5f.c — 1 kHz millisecond timebase for the V5F core.
//
// Mirrors core/timebase.c (the V3F TIM3 tick) but uses TIM4 so the two cores
// never contend for the same timer instance. The merge module's release
// scheduling (usb_merge_drain_icc -> click/kb release deadlines) needs millis()
// on V5F; this provides it.
//
// Why TIM4 (not TIM3, not SysTick):
//   * TIM3 is the V3F timebase. Although the two cores are separate, keeping a
//     distinct timer per core avoids any shared-register confusion and matches
//     the "TIM4 on V5F" directive.
//   * The vendored Debug/debug.c implements Delay_Us/Delay_Ms by *polling*
//     SysTick (set CMP, busy-wait, disable). Taking SysTick for a 1 ms IRQ
//     would fight those polled delays (desc_capture.c / led_blink_forever use
//     them). A spare general-purpose timer keeps the vendor delay API intact.
//   * TIM4 lives on the HB1 bus and is unused elsewhere in the V5F image. Its
//     IRQ vector (TIM4_IRQHandler) is wired in core/startup_v5f.S.
//
// Timer clock domain (V5F):
//   TIM4 sits on the HB1/HCLK bus. RCC_GetClocksFreq().HCLK_Frequency is that
//   bus clock, queried directly rather than trusting the passed-in core_hz,
//   because on V5F SystemCoreClock == Core_Frequency == the pre-FPRE AHB clock
//   (system_ch32h417.c: SystemCoreClock = tmp3), whereas the HB1 peripheral
//   clock is HCLK_Frequency = tmp3 >> FPRE. Using HCLK_Frequency makes the 1 ms
//   period exact regardless of the FPRE divider. The core_hz argument is kept
//   for API symmetry with timebase_init() and used only as a fallback.

#include "ch32h417_port.h"   // device + StdPeriph (RCC/TIM/NVIC) + core_riscv
#include "timebase_v5f.h"

static volatile uint32_t g_ms;

// ── Free-running 1 MHz microsecond counter (TIM9) ────────────────────────────
// Provides a microsecond timestamp for the gesture capture ring (per-report
// motion-residual timing) and general µs delays. Read via timebase_v5f_us().
//
// Why TIM9 (32-bit) and not TIM5 (16-bit):
//   * TIM9..TIM12 on this part are 32-bit counters (CNT_32 / ATRLR_32 union
//     members); TIM2..TIM7 (incl. TIM5) are 16-bit. A 16-bit 1 MHz counter
//     wraps every 65.536 ms, which would make humanize.c's 32-bit interval
//     subtraction (dt = ts_us - last_ts_us) wrap INCORRECTLY across the 16-bit
//     boundary in 32-bit arithmetic. A 32-bit 1 MHz counter wraps only every
//     ~71.6 minutes, so dt is always single-wrap-safe in plain uint32 math.
//   * This lets the counter be fully free-running with NO interrupt — CNT_32
//     is read directly. (A 16-bit timer would have required an overflow IRQ to
//     software-extend to 32 bits; the 32-bit timer avoids that entirely.)
//
// Clock domain: TIM9 is on the HB2 bus, which (like HB1/TIM4) is fed from HCLK.
// HCLK is prescaled to a 1 MHz tick, matching the TIM4 millis path, then
// set the 32-bit auto-reload to its max so the counter free-runs to full range.
//
// Register access: TIM_TimeBaseInit() writes only the 16-bit ATRLR member of
// the period union, so after the StdPeriph init we set TIM9->ATRLR_32 directly
// to 0xFFFFFFFF, and read the counter via TIM9->CNT_32. TIM9's reset counter
// mode is up-counting (CTLR1 = 0), which is what we want; TIM_TimeBaseInit does
// not touch CTLR1 for TIM9 (it only does for TIM1/2/3/4/5/8), so the default
// up-count + the SWEVGR immediate PSC reload it issues are sufficient.

static void timebase_v5f_us_init(uint32_t core_hz)
{
    uint32_t timer_hz;
    RCC_ClocksTypeDef clk;
    RCC_GetClocksFreq(&clk);
    timer_hz = clk.HCLK_Frequency;
    if (timer_hz == 0) timer_hz = core_hz;
    if (timer_hz == 0) timer_hz = 200000000u;          // safe fallback

    uint32_t psc = (timer_hz / 1000000u);              // -> 1 MHz tick (1 µs)
    if (psc == 0) psc = 1;
    psc -= 1u;

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_TIM9, ENABLE);

    TIM_TimeBaseInitTypeDef tb;
    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler         = (uint16_t)psc;          // -> 1 MHz tick
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_Period            = 0xFFFFu;                // overwritten below (32-bit)
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM9, &tb);

    // Widen the auto-reload to the full 32-bit range (TIM_TimeBaseInit only set
    // the low 16 bits). Free-running, no update interrupt: we only read CNT.
    TIM9->ATRLR_32 = 0xFFFFFFFFu;
    TIM9->CNT_32   = 0u;

    TIM_Cmd(TIM9, ENABLE);
}

uint32_t timebase_v5f_us(void) { return TIM9->CNT_32; }

// V5F-local blocking delays built on the free-running 1 MHz TIM9 counter.
//
// WHY THESE EXIST: the vendor Delay_Us/Delay_Ms (debug.c) spin on the SHARED
// SysTick0->ISR status register (bit0=V3F's SysTick0, bit1=V5F's SysTick1 — one
// physical register, see core_riscv.h). When BOTH cores call Delay_* at once
// (e.g. the instant the PC enumerates the cloned device and V3F gets busy), V3F's
// `SysTick0->ISR &= ~(1<<0)` read-modify-write races V5F's bit-1 wait, and V5F's
// Delay_Us spins FOREVER — wedging the relay loop in usbhs_transact (the real
// "hang on PC plug-in / no reports" bug). TIM9 is V5F-private and only ever READ
// here (no shared status register, no RMW), so it cannot be raced by V3F.
// These wrap correctly across the 32-bit TIM9 counter (unsigned subtraction).
// BENCH DIAG / SAFETY: the wedge witnessed on the bench is V5F frozen at the
// USBHS transfer-wait stamp (0x40A3), whose body is this delay. If TIM9 ever
// stops counting, the original `while (CNT - start < us)` spins FOREVER (the
// subtraction stays 0 < us) — wedging the relay/enum. We now bound the spin with
// an instruction-count fallback so a stalled TIM9 can NEVER hang V5F, and we
// publish a liveness witness so the oracle can confirm whether TIM9 actually
// stalled (cnt_seen_equal>0 with a huge fallback_hits => TIM9 frozen).
volatile uint32_t tim9_dbg_fallback_hits;   // times the fallback bound fired
volatile uint32_t tim9_dbg_last_cnt;        // last TIM9 CNT observed
volatile uint32_t tim9_dbg_max_spin;        // worst-case fallback spin count seen

void timebase_v5f_delay_us(uint32_t us)
{
    uint32_t start = TIM9->CNT_32;
    // Fallback ceiling: at 400 MHz core, this loop is a few instructions/iter, so
    // ~4000 iters ≈ at least 1 µs of real time even if TIM9 is dead. Scale by us,
    // with generous headroom (×8000) so a LIVE TIM9 always wins the race first and
    // timing stays accurate; the bound only matters when TIM9 is frozen.
    uint32_t guard = 0;
    uint32_t guard_max = (us > 524287u ? 524287u : us) * 8000u + 8000u;
    while ((uint32_t)(TIM9->CNT_32 - start) < us) {
        if (++guard >= guard_max) {
            tim9_dbg_fallback_hits++;
            if (guard > tim9_dbg_max_spin) tim9_dbg_max_spin = guard;
            break;
        }
    }
    tim9_dbg_last_cnt = TIM9->CNT_32;
}

void timebase_v5f_delay_ms(uint32_t ms)
{
    while (ms--) timebase_v5f_delay_us(1000u);
}

void timebase_v5f_init(uint32_t core_hz)
{
    // Determine the HB1 (TIM4) input clock. Prefer the measured HCLK from the
    // RCC driver; fall back to the caller-supplied core_hz, then a safe default.
    uint32_t timer_hz;
    RCC_ClocksTypeDef clk;
    RCC_GetClocksFreq(&clk);
    timer_hz = clk.HCLK_Frequency;
    if (timer_hz == 0) timer_hz = core_hz;
    if (timer_hz == 0) timer_hz = 200000000u;          // safe fallback

    // Prescale to a 1 MHz tick, then count 1000 ticks per 1 ms update event.
    uint32_t psc = (timer_hz / 1000000u);
    if (psc == 0) psc = 1;                             // guard for very low clocks
    psc -= 1u;

    RCC_HB1PeriphClockCmd(RCC_HB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef tb;
    TIM_TimeBaseStructInit(&tb);
    tb.TIM_Prescaler         = (uint16_t)psc;          // -> 1 MHz timer tick
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_Period            = 1000u - 1u;             // 1000 ticks @1MHz = 1 ms
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &tb);

    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);        // drop the init-generated UEV
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    // DUAL-CORE IRQ ROUTING: TIM4_IRQn (>31) is core-allocated via NVIC->IALLOCR,
    // default Core_ID_V3F. This millis IRQ runs on V5F and is the ONLY periodic
    // wake source for the relay loop's wfi (the USBHS host is polled, the USBFS
    // device only IRQs on transfers). Without routing it to V5F, the IRQ is
    // delivered to V3F and V5F's wfi never wakes once the USB bus goes idle —
    // the relay loop freezes and stops polling for new reports. Same bug class as
    // the USBFS IRQ-alloc and TIM2 cross-core fixes.
    NVIC_SetAllocateIRQ(TIM4_IRQn, Core_ID_V5F);
    NVIC_EnableIRQ(TIM4_IRQn);
    TIM_Cmd(TIM4, ENABLE);

    // Also start the free-running 1 MHz µs counter (TIM9) used for gesture
    // capture timestamps. No IRQ — read via timebase_v5f_us().
    timebase_v5f_us_init(core_hz);
}

uint32_t millis(void) { return g_ms; }

// TIM4 update ISR — handler name matches the V5F vector table in startup_v5f.S.
void TIM4_IRQHandler(void) WCH_IRQ;
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        g_ms++;
    }
}
