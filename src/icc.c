#include "ch32h417_port.h"
#include "icc.h"
#include <string.h>

// Single instance, pinned into shared SRAM (0x20178000) by both linker scripts.
// 0x20178000 lands in V3F's DTCM. A core can write the other core's DTCM and that
// core reads its own DTCM back fine, but a core reading the other core's DTCM gets
// stale data. Consequences:
//   * V3F->V5F (injection): V5F reading g_icc.v3f_to_v5f returns garbage, so this
//     direction goes through the IPC MSG hardware mailbox (IPC->MSG[0..3], MMIO at
//     0xE000D000, coherent across both cores). g_icc.v3f_to_v5f is a V3F-local
//     producer FIFO that icc_pump_to_v5f() drains into the mailbox; V5F never
//     reads it.
//   * V5F->V3F (telemetry): a return ring would hit the same stale-read wall and
//     the 4 IPC MSG regs are taken by the injection mailbox, so bulk telemetry is
//     not carried. V5F liveness/stage rides the coherent IPC CH1 status bits
//     (icc_telem_* at the bottom).
icc_shared_t g_icc __attribute__((section(".shared_data")));

static inline void mem_fence(void) { __asm volatile("fence" ::: "memory"); }

// --- V3F->V5F IPC MSG mailbox (single 8-byte slot, seq/ack handshake) --------
// MSG[0] = record bytes [0..3], MSG[1] = record bytes [4..7] (all V3F->V5F record
// tags fit in 8 bytes: keyboard = tag+mod+6keys is the largest). MSG[2] = publish
// seq (V3F bumps it after writing the slot — the "new data" signal). MSG[3] = ack
// (V5F sets it to the seq it consumed). Mailbox is free when MSG[3] == last seq
// V3F published. seq/ack are plain monotonic counters (wrap is harmless).
static uint32_t s_v3f_pub_seq;   // V3F: last seq published to the mailbox
static uint32_t s_v5f_seen_seq;  // V5F: last seq consumed from the mailbox

// Configure IPC channel 0 routing on the V3F (master) core. The IPC unit is
// core-private (0xE000D000); the CTLR routing (TxCID/RxCID/AutoEN) must be
// programmed or IPC_SetFlagStatus on Bit0 never raises the cross-core IRQ on V5F
// and the wfi/doorbell wake path degrades to poll latency. CTLR is configured on
// V3F only; V5F enables its NVIC + IT (main_v5f.c). Doorbell direction: V3F sets
// Bit0, V5F clears it.
static void icc_ipc_config_v3f(void)
{
    IPC_InitTypeDef ipc = {0};
    ipc.IPC_CH  = IPC_CH0;
    ipc.TxCID   = IPC_TxCID1;   // V3F->V5F
    ipc.RxCID   = IPC_RxCID0;
    ipc.TxIER   = ENABLE;
    ipc.RxIER   = ENABLE;
    ipc.AutoEN  = ENABLE;

    IPC_DeInit();
    /* Warm-reset safety: IPC_DeInit() only zeroes STS, which may leave sticky
     * channel-status bits set from a prior boot. Clear all of them before
     * IPC_Init so the first doorbell/telemetry write starts from a clean slate. */
    IPC->CLR = 0xFFFFFFFFu;
    IPC_Init(&ipc);
    IPC_CH0_Lock();
}

void icc_init_v3f(void)
{
    for (uint32_t i = 0; i < ICC_RING_SLOTS; i++) {
        g_icc.v3f_to_v5f.slot[i].tag = ICC_TAG_NONE;
    }
    g_icc.v3f_to_v5f.head = g_icc.v3f_to_v5f.tail = 0;
    mem_fence();
    g_icc.magic = ICC_MAGIC;
    mem_fence();

    icc_ipc_config_v3f();   // program IPC CH0 routing before the doorbell is used

    // Init the IPC MSG mailbox: clear seq + ack so the first publish is seq=1 and
    // V5F (seeing seq!=0) consumes it. Mailbox starts free.
    s_v3f_pub_seq = 0;
    IPC->MSG[2] = 0;        // publish seq
    IPC->MSG[3] = 0;        // ack (== seq -> free)
}

void icc_init_v5f(void)
{
    while (g_icc.magic != ICC_MAGIC) { __asm volatile("nop"); }
    mem_fence();
    // Sync our seen-seq to the mailbox's current publish seq so we don't replay a
    // stale record. icc_init_v3f cleared MSG[2]=0 before the wake, so this is 0.
    s_v5f_seen_seq = IPC->MSG[2];
}

static bool ring_push(icc_ring_t *r, const icc_record_t *rec)
{
    uint32_t head = r->head;
    uint32_t next = (head + 1) & ICC_RING_MASK;
    mem_fence();                            // observe the consumer's latest tail
    if (next == (r->tail & ICC_RING_MASK)) return false;   // full
    r->slot[head & ICC_RING_MASK] = *rec;
    mem_fence();                            // release: slot write before head publish
    r->head = next;
    return true;
}

static bool ring_pop(icc_ring_t *r, icc_record_t *out)
{
    uint32_t tail = r->tail;
    if ((tail & ICC_RING_MASK) == (r->head & ICC_RING_MASK)) return false; // empty
    mem_fence();                            // acquire: read slot only after observing head
    *out = r->slot[tail & ICC_RING_MASK];
    mem_fence();                            // slot read completes before freeing the slot
    r->tail = (tail + 1) & ICC_RING_MASK;
    return true;
}

// --- V3F side ---------------------------------------------------------------
// Enqueue into the V3F-local producer FIFO (V3F's own DTCM, so coherent).
// icc_pump_to_v5f() later moves records from this FIFO into the IPC mailbox.
bool icc_send_to_v5f(const icc_record_t *r)  { return ring_push(&g_icc.v3f_to_v5f, r); }

// Move at most one queued record from the V3F-local FIFO into the IPC MSG mailbox
// if the mailbox is free (V5F has acked the last publish). Returns true if a
// record was pushed. The mailbox is a single 8-byte slot; the 256-deep local FIFO
// absorbs bursts while V5F drains one record per loop.
bool icc_pump_to_v5f(void)
{
    // Mailbox busy until V5F's ack (MSG[3]) catches up to our last publish seq.
    if (IPC->MSG[3] != s_v3f_pub_seq) return false;

    icc_record_t rec;
    if (!ring_pop(&g_icc.v3f_to_v5f, &rec)) return false;   // FIFO empty

    // Pack the first 8 record bytes (tag + b[0..6]) into MSG[0..1]. All V3F->V5F
    // tags fit in 8 bytes (keyboard tag+mod+6keys = 8; click/kb-release tag+key+
    // u32 = 6; set_baud tag+u32 = 5; inject_mouse tag+5 = 6).
    uint8_t buf[8];
    buf[0] = rec.tag;
    memcpy(&buf[1], rec.b, 7);
    uint32_t w0, w1;
    memcpy(&w0, &buf[0], 4);
    memcpy(&w1, &buf[4], 4);
    IPC->MSG[0] = w0;
    IPC->MSG[1] = w1;
    mem_fence();                    // publish payload before bumping the seq
    IPC->MSG[2] = ++s_v3f_pub_seq;  // signal "new record" to V5F

    // Ring the doorbell after MSG[2] is published so the wakeup is meaningful: V5F
    // wakes exactly when a record is visible. Ringing at FIFO-push time would wake
    // V5F before this pump loaded the mailbox, so it would see an unchanged seq and
    // return to wfi, deferring the record to the next 1 ms TIM4 poll. The
    // single-slot mailbox rate-limits this to one doorbell per record.
    icc_ring_doorbell_v5f();
    return true;
}

// V5F->V3F bulk telemetry (report/drop counts, status flags) is not carried. A
// lock-free SRAM ring would need both cores to read the shared head+tail, but
// cross-core DTCM reads are stale on this part, and the 4 IPC MSG registers are
// consumed by the injection mailbox. V5F liveness rides the coherent IPC CH1
// stage telemetry below (icc_telem_*).

// --- V5F side ---------------------------------------------------------------
// Read one record from the IPC mailbox if V3F published a new seq since the last
// call. Never touches g_icc.v3f_to_v5f (V3F's DTCM, stale on V5F). Returns false
// when no new record is pending.
bool icc_recv_from_v3f(icc_record_t *out)
{
    uint32_t seq = IPC->MSG[2];
    if (seq == s_v5f_seen_seq) return false;   // nothing new
    mem_fence();                                // acquire payload after seeing seq
    uint32_t w0 = IPC->MSG[0];
    uint32_t w1 = IPC->MSG[1];
    uint8_t buf[8];
    memcpy(&buf[0], &w0, 4);
    memcpy(&buf[4], &w1, 4);
    memset(out, 0, sizeof(*out));               // zero b[7..14] we don't transmit
    out->tag = buf[0];
    memcpy(out->b, &buf[1], 7);
    s_v5f_seen_seq = seq;
    mem_fence();
    IPC->MSG[3] = seq;                          // ack: mailbox free for next record
    return true;
}

void icc_ring_doorbell_v5f(void)
{
    IPC_SetFlagStatus(IPC_CH0, IPC_CH_Sta_Bit0);
}

// --- V5F->V3F coherent stage telemetry (IPC CH1 status bits 8..15) -----------
// CH1's byte is a single-writer V5F->V3F register: [15:14]=heartbeat seq,
// [13:8]=stage code. V5F is the only writer (race-free, no lock); V3F only reads.
// Peripheral-bus MMIO (0xE000D000), coherent across cores. IPC->SET sets bits,
// IPC->CLR clears, IPC->STS reads. Each call rewrites the whole CH1 byte.
#define ICC_TLM_CH1_SHIFT   8u                 // CH1 status bits start at bit 8
#define ICC_TLM_CH1_MASK    (0xFFu << ICC_TLM_CH1_SHIFT)

void icc_telem_stage_v5f(uint8_t code)
{
    static uint8_t s_seq;                       // V5F-local rolling heartbeat (0..3)
    uint8_t val = (uint8_t)(((++s_seq & 0x3u) << 6) | (code & 0x3Fu));
    // Clear all CH1 bits, then set the new byte. SET/CLR are write-1-to-action,
    // so this is two MMIO stores with no RMW.
    IPC->CLR = ICC_TLM_CH1_MASK;
    IPC->SET = ((uint32_t)val << ICC_TLM_CH1_SHIFT);
}

uint8_t icc_telem_read_v3f(void)
{
    return (uint8_t)((IPC->STS >> ICC_TLM_CH1_SHIFT) & 0xFFu);
}

// IPC_CH0 doorbell ISR. Its only job is to wake the core from wfi; the mailbox
// read happens in the foreground (usb_merge_drain_icc -> icc_recv_from_v3f).
//
// The channel runs AutoEN=ENABLE, so the Bit0 interrupt re-asserts from the
// MSG-mailbox-pending condition: clearing the flag alone does not deassert it and
// the IRQ re-fires immediately, starving a foreground that is slow to drain (e.g.
// the ~80 ms two-board descriptor send). The ISR disables its own IT bit after
// clearing; the foreground re-arms it (icc_ipc_rearm_v5f) once drained. One ISR
// entry per doorbell regardless of foreground duration.
void IPC_CH0_Handler(void) WCH_IRQ;
void IPC_CH0_Handler(void)
{
    IPC_ClearFlagStatus(IPC_CH0, IPC_CH_Sta_Bit0);
    IPC_ITConfig(IPC_CH0, IPC_CH_Sta_Bit0, DISABLE);  // re-armed by the foreground
}

// Re-arm the IPC_CH0 doorbell after the foreground has drained the mailbox. Safe to
// call unconditionally each foreground pass (idempotent ENABLE).
void icc_ipc_rearm_v5f(void)
{
    IPC_ITConfig(IPC_CH0, IPC_CH_Sta_Bit0, ENABLE);
}

// --- V5F->V3F reverse status channel (IPC status bits [16:31], CH2+CH3) ------
// Distinct from the CH1 [8:15] stage telemetry above. Single-writer (V5F);
// V3F only reads. Coherent peripheral MMIO — never shared SRAM.
#define ICC_ST_SHIFT   16u
#define ICC_ST_MASK    (0xFFFFu << ICC_ST_SHIFT)

void icc_status_pump_v5f(const display_status_t *st)
{
    static uint8_t s_sel;            // rotation index
    static uint8_t s_seq;            // rolling heartbeat
    static uint8_t s_last_state = 0xFF;
    uint8_t sel;
    if (st->state != s_last_state) { // state changes jump the queue
        s_last_state = st->state;
        sel = ICC_ST_SEL_STATE;
    } else {
        sel = s_sel;
    }
    s_sel = (uint8_t)((s_sel + 1) % ICC_ST_SEL__COUNT);  // always advance the rotation
    uint16_t word = icc_status_pack(sel, ++s_seq, st);
    IPC->CLR = ICC_ST_MASK;
    IPC->SET = ((uint32_t)word << ICC_ST_SHIFT);
}

bool icc_status_poll_v3f(display_status_t *acc)
{
    static uint8_t s_last_seq = 0xFF;
    uint16_t word = (uint16_t)((IPC->STS >> ICC_ST_SHIFT) & 0xFFFFu);
    uint8_t seq = icc_status_unpack(word, acc);
    bool advanced = (seq != s_last_seq);
    s_last_seq = seq;
    return advanced;
}

uint16_t icc_status_read_raw_v3f(void)
{
    return (uint16_t)((IPC->STS >> ICC_ST_SHIFT) & 0xFFFFu);
}
