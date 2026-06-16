#include "ch32h417_port.h"
#include "icc.h"
#include <string.h>

// Single instance, pinned into shared SRAM (0x20178000) by both linker scripts.
// IMPORTANT (bench-proven 2026-06-13): 0x20178000 lands in V3F's DTCM. A core
// can WRITE the other core's DTCM and that core reads its OWN DTCM back fine, but
// a core READING the other core's DTCM gets STALE/garbage data. Consequences:
//   * V3F->V5F (injection): V5F reading g_icc.v3f_to_v5f from V3F's DTCM returns
//     garbage — V5F's ring_pop saw head!=tail forever and spun. So this direction
//     is NOT carried in SRAM; it goes through the IPC MSG hardware mailbox
//     (IPC->MSG[0..3], MMIO at 0xE000D000, proven coherent across both cores).
//     g_icc.v3f_to_v5f is now a V3F-LOCAL producer FIFO that icc_pump_to_v5f()
//     drains into the mailbox; V5F never reads it.
//   * V5F->V3F (telemetry): a return ring would hit the SAME stale-read wall (V3F
//     reading V5F's writes), and the 4 IPC MSG regs are taken by the injection
//     mailbox, so bulk telemetry is not carried at all. The v5f_to_v3f ring was
//     removed (reclaiming ~4 KB of shared SRAM per image). V5F liveness/stage
//     instead rides the coherent IPC CH1 status bits (icc_telem_* at the bottom).
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
// core-private (0xE000D000); the CTLR routing (TxCID/RxCID/AutoEN) MUST be
// programmed or IPC_SetFlagStatus on Bit0 never raises the cross-core IRQ on
// V5F — the wfi/doorbell wake path silently degrades to poll latency.
//
// CID values + DeInit/Lock order are copied verbatim from the WCH EVT IPC
// example (EXAM/CPU/IPC, Common/hardware.c) for the same V3F<->V5F channel 0.
// The reference configures CTLR on V3F only; V5F just enables its NVIC + IT
// (done in main_v5f.c). Doorbell bit direction (V3F sets Bit0, V5F clears it)
// is our convention, not the example's ping-pong (verified on the bench).
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

    // Init the V3F->V5F IPC MSG mailbox: clear seq + ack so the first publish is
    // seq=1 and V5F (seeing seq!=0) consumes it. Mailbox starts free.
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
// icc_send_to_v5f: enqueue into the V3F-LOCAL producer FIFO (V3F's own DTCM, so
// this read/modify/write is coherent). icc_pump_to_v5f() later moves records from
// this FIFO into the coherent IPC mailbox as it drains.
bool icc_send_to_v5f(const icc_record_t *r)  { return ring_push(&g_icc.v3f_to_v5f, r); }

// icc_pump_to_v5f: move at most one queued record from the V3F-local FIFO into the
// IPC MSG mailbox, if the mailbox is free (V5F has acked the last publish). Call
// this from the V3F main loop every iteration. Returns true if a record was
// pushed into the mailbox. The mailbox is a single 8-byte slot; the local FIFO
// (256 deep) absorbs bursts while V5F drains one record per loop.
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

    // Wake V5F NOW that the record is actually in the coherent mailbox. The
    // doorbell used to be rung by the producers (kmbox_cmd_inject_*) at FIFO-push
    // time — but that is BEFORE this pump loads MSG[0..2], so V5F woke, saw an
    // unchanged seq in icc_recv_from_v3f(), and went straight back to wfi; the
    // record was then only picked up on the next TIM4 1 ms poll (up to ~1 ms of
    // added injection latency, plus one spurious wakeup per pushed record during
    // a UART burst). Ringing here — after MSG[2] is published — makes every
    // wakeup meaningful: V5F wakes exactly when a record is visible, and the
    // single-slot mailbox rate-limits this to one doorbell per record.
    icc_ring_doorbell_v5f();
    return true;
}

// V5F->V3F bulk telemetry (report/drop counts, status flags) is DROPPED — not
// carried at all. A lock-free SRAM ring would need BOTH cores to read the shared
// head+tail, but cross-core DTCM reads are stale on this part (only the producer's
// remote WRITE lands; the consumer can only read its OWN DTCM), and the 4 IPC MSG
// registers are fully consumed by the V3F->V5F injection mailbox. The send/recv
// API and the v5f_to_v3f ring were removed (dead no-ops). V5F liveness instead
// rides the coherent IPC CH1 stage telemetry below (icc_telem_*), which is all
// the V3F diagnostic UART actually consumes.

// --- V5F side ---------------------------------------------------------------
// icc_recv_from_v3f: read one record from the IPC mailbox if V3F published a new
// seq since we last looked. NEVER touches g_icc.v3f_to_v5f (that lives in V3F's
// DTCM and reads back stale on V5F — the whole reason for the mailbox). Returns
// false when no new record is pending.
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
// CH1 owns STS bits [8..15] (bit = CH*8 + n = 8 + n). We use that byte as a
// single-writer V5F->V3F register: [15:14]=heartbeat seq, [13:8]=stage code.
// V5F is the ONLY writer (sole-writer => race-free, no lock); V3F only reads.
// These are peripheral-bus MMIO (0xE000D000), coherent across cores — unlike the
// 0x2017xxxx shared-SRAM writes that stall V5F. IPC->SET sets bits, IPC->CLR
// clears them, IPC->STS reads them back (see ch32h417_ipc.c). We write the whole
// CH1 byte each call: clear the 8 CH1 bits, then set the new value's bits.
#define ICC_TLM_CH1_SHIFT   8u                 // CH1 status bits start at bit 8
#define ICC_TLM_CH1_MASK    (0xFFu << ICC_TLM_CH1_SHIFT)

void icc_telem_stage_v5f(uint8_t code)
{
    static uint8_t s_seq;                       // V5F-local rolling heartbeat (0..3)
    uint8_t val = (uint8_t)(((++s_seq & 0x3u) << 6) | (code & 0x3Fu));
    // Single 32-bit clear of all CH1 bits, then a single set of the new byte.
    // SET/CLR are write-1-to-action registers, so this is two MMIO stores, no RMW
    // race (and V5F is the only writer of CH1 regardless).
    IPC->CLR = ICC_TLM_CH1_MASK;
    IPC->SET = ((uint32_t)val << ICC_TLM_CH1_SHIFT);
}

uint8_t icc_telem_read_v3f(void)
{
    return (uint8_t)((IPC->STS >> ICC_TLM_CH1_SHIFT) & 0xFFu);
}

void IPC_CH0_Handler(void) WCH_IRQ;
void IPC_CH0_Handler(void)
{
    if (IPC_GetITStatus(IPC_CH0, IPC_CH_Sta_Bit0)) {
        IPC_ClearFlagStatus(IPC_CH0, IPC_CH_Sta_Bit0);
    }
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
