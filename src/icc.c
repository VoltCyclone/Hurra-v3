#include "ch32h417_port.h"
#include "icc.h"

// Single instance, pinned into shared SRAM (0x20178000) by both linker scripts.
__attribute__((section(".shared_data")))
icc_shared_t g_icc;

static inline void mem_fence(void) { __asm volatile("fence" ::: "memory"); }

// Configure IPC channel 0 routing on the V3F (master) core. The IPC unit is
// core-private (0xE000D000); the CTLR routing (TxCID/RxCID/AutoEN) MUST be
// programmed or IPC_SetFlagStatus on Bit0 never raises the cross-core IRQ on
// V5F — the wfi/doorbell wake path silently degrades to poll latency.
//
// CID values + DeInit/Lock order are copied verbatim from the WCH EVT IPC
// example (EXAM/CPU/IPC, Common/hardware.c) for the same V3F<->V5F channel 0.
// The reference configures CTLR on V3F only; V5F just enables its NVIC + IT
// (done in main_v5f.c). Doorbell bit direction (V3F sets Bit0, V5F clears it)
// is our convention, not the example's ping-pong — verify Bit0 delivery on the
// bench (see CLAUDE.md ISR-name gotcha).
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
        g_icc.v5f_to_v3f.slot[i].tag = ICC_TAG_NONE;
    }
    g_icc.v3f_to_v5f.head = g_icc.v3f_to_v5f.tail = 0;
    g_icc.v5f_to_v3f.head = g_icc.v5f_to_v3f.tail = 0;
    mem_fence();
    g_icc.magic = ICC_MAGIC;
    mem_fence();

    icc_ipc_config_v3f();   // program IPC CH0 routing before the doorbell is used
}

void icc_init_v5f(void)
{
    while (g_icc.magic != ICC_MAGIC) { __asm volatile("nop"); }
    mem_fence();
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

bool icc_send_to_v5f(const icc_record_t *r)  { return ring_push(&g_icc.v3f_to_v5f, r); }
bool icc_send_to_v3f(const icc_record_t *r)  { return ring_push(&g_icc.v5f_to_v3f, r); }
bool icc_recv_from_v3f(icc_record_t *out)    { return ring_pop(&g_icc.v3f_to_v5f, out); }
bool icc_recv_from_v5f(icc_record_t *out)    { return ring_pop(&g_icc.v5f_to_v3f, out); }

void icc_ring_doorbell_v5f(void)
{
    IPC_SetFlagStatus(IPC_CH0, IPC_CH_Sta_Bit0);
}

void IPC_CH0_Handler(void) WCH_IRQ;
void IPC_CH0_Handler(void)
{
    if (IPC_GetITStatus(IPC_CH0, IPC_CH_Sta_Bit0)) {
        IPC_ClearFlagStatus(IPC_CH0, IPC_CH_Sta_Bit0);
    }
}
