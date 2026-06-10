#include "ch32h417_port.h"
#include "icc.h"

// Single instance, pinned into shared SRAM (0x20178000) by both linker scripts.
__attribute__((section(".shared_data")))
icc_shared_t g_icc;

static inline void mem_fence(void) { __asm volatile("fence" ::: "memory"); }

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
    if (next == (r->tail & ICC_RING_MASK)) return false;   // full
    r->slot[head & ICC_RING_MASK] = *rec;
    mem_fence();                                           // record before head
    r->head = next;
    return true;
}

static bool ring_pop(icc_ring_t *r, icc_record_t *out)
{
    uint32_t tail = r->tail;
    if ((tail & ICC_RING_MASK) == (r->head & ICC_RING_MASK)) return false; // empty
    *out = r->slot[tail & ICC_RING_MASK];
    mem_fence();                                           // record before tail
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

void IPC_CH0_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void IPC_CH0_Handler(void)
{
    if (IPC_GetITStatus(IPC_CH0, IPC_CH_Sta_Bit0)) {
        IPC_ClearFlagStatus(IPC_CH0, IPC_CH_Sta_Bit0);
    }
}
