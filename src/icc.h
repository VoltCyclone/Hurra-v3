#pragma once
#include <stdint.h>
#include <stdbool.h>

// Inter-core channel. Two single-producer/single-consumer rings in shared SRAM.
// V3F->V5F: injection commands. V5F->V3F: telemetry. Lock-free; HSEM only for
// the one-time init rendezvous. IPC channel 0 is the V3F->V5F doorbell.

#define ICC_RECORD_BYTES   16
#define ICC_RING_SLOTS     256          // power of two
#define ICC_RING_MASK      (ICC_RING_SLOTS - 1)

// Record tags (tag in byte 0).
enum {
    ICC_TAG_NONE = 0,
    // V3F -> V5F (inject commands)
    ICC_TAG_INJECT_MOUSE,
    ICC_TAG_INJECT_KEYBOARD,
    ICC_TAG_CLICK_RELEASE,
    ICC_TAG_KB_RELEASE,
    ICC_TAG_SET_BAUD,
    ICC_TAG_SET_HUMAN_LEVEL,
    ICC_TAG_PHYS_MASK,
    // V5F -> V3F (telemetry)
    ICC_TAG_TELEM_COUNTS,
    ICC_TAG_TELEM_STATUS,
    // Phase-2 smoke test
    ICC_TAG_PING,
    ICC_TAG_PONG,
};

typedef struct {
    uint8_t tag;
    uint8_t b[ICC_RECORD_BYTES - 1];    // payload, tag-specific
} icc_record_t;

typedef struct {
    volatile uint32_t head;             // producer writes
    volatile uint32_t tail;             // consumer writes
    icc_record_t slot[ICC_RING_SLOTS];
} icc_ring_t;

// The shared block lives at a fixed address in both images' .shared section.
typedef struct {
    volatile uint32_t magic;            // set by V3F at init
    icc_ring_t v3f_to_v5f;
    icc_ring_t v5f_to_v3f;
} icc_shared_t;

#define ICC_MAGIC  0x48563343u          // 'HV3C'

void icc_init_v3f(void);   // master: zero rings, set magic, then signal via HSEM
void icc_init_v5f(void);   // slave: wait for magic via HSEM rendezvous

// Producer side (returns false if ring full).
bool icc_send_to_v5f(const icc_record_t *r);   // call from V3F (enqueue to local FIFO)
bool icc_send_to_v3f(const icc_record_t *r);   // call from V5F

// V3F: drain one queued V3F->V5F record from the local FIFO into the coherent IPC
// MSG mailbox if it is free. Call every V3F main-loop iteration. Returns true if a
// record was moved into the mailbox. (The V3F->V5F SRAM ring is not readable by
// V5F — it lives in V3F's DTCM — so records cross via the IPC hardware mailbox.)
bool icc_pump_to_v5f(void);                    // call from V3F

// Consumer side (returns false if empty).
bool icc_recv_from_v3f(icc_record_t *out);     // call from V5F
bool icc_recv_from_v5f(icc_record_t *out);     // call from V3F

// Doorbell: V3F rings V5F after enqueue so V5F can wfi when idle.
void icc_ring_doorbell_v5f(void);              // V3F side
// V5F's IPC_CH0_Handler clears the doorbell; defined in icc.c.

// --- Bench debug: V5F boot-stage marker -------------------------------------
// A single volatile word at a FIXED shared-SRAM address (well past g_icc, which
// ends ~0x2017A014, and below the V3F stack) that V5F overwrites as it advances
// through bring-up. Read it live with `wlink dump 0x2017F000 4` to see exactly
// where V5F is, without depending on the icc_shared_t layout (it's a raw
// address, not a struct member).
#define DBG_STAGE_ADDR   0x2017F000u
enum {                                         // V5F boot stages (monotonic)
    DBG_V5F_BOOT          = 0x51,              // entered main, pre-rendezvous
    // Fine-grained early-init markers (0x60..) to bisect a pre-rendezvous hang.
    DBG_V5F_TIMEBASE      = 0x60,              // after timebase_v5f_init
    DBG_V5F_HUMANIZE      = 0x61,              // after humanize_init
    DBG_V5F_MERGE_INIT    = 0x62,              // after usb_merge_init
    DBG_V5F_LED_INIT      = 0x63,              // after led_init
    DBG_V5F_ICC_MAGIC     = 0x64,              // after icc_init_v5f (magic seen)
    DBG_V5F_HSEM_DONE     = 0x65,              // after HSEM take/release
    // Pre-USBHS window (between ICC_READY and usb_host_init) — bisects a hang that
    // freezes V5F before it ever reaches usb_host_init()'s first marker (0x70).
    DBG_V5F_PRE_AFIO      = 0x66,              // about to enable AFIO|GPIOB clock
    DBG_V5F_PRE_SWJ       = 0x67,              // AFIO|GPIOB on; about to disable SWJ
    DBG_V5F_PRE_HOSTINIT  = 0x68,              // SWJ disabled; about to call usb_host_init
    DBG_V5F_ICC_READY     = 0x52,              // ICC rendezvous done (IPC IRQ armed)
    DBG_V5F_HOST_INIT     = 0x53,              // USBHS host init done, entering host-wait
    DBG_V5F_HOST_WAITING  = 0x54,              // spinning in while(!device_connected)
    DBG_V5F_DEV_CONNECTED = 0x55,              // host saw a device, leaving host-wait
    DBG_V5F_DESC_OK       = 0x56,              // descriptors captured
    DBG_V5F_DEV_INIT      = 0x57,              // USBFS device init done (cloning to PC)
    DBG_V5F_RELAY         = 0x58,              // reached the relay loop (telemetry flows)
    // Trap marker: V5F's HardFault_Handler ORs 0x80 over the low mcause nibble so
    // a CPU trap surfaces as a single UART line (V3F prints it) instead of only a
    // PC3 blink. e.g. 0x82 = illegal-instruction, 0x85 = load-fault, 0x87 = store-
    // fault, 0x81 = instr-access-fault, 0x80 = instr-addr-misaligned, 0x8B=ecall-M.
    DBG_V5F_TRAP_BASE     = 0x80,              // 0x80 | (mcause & 0x0F)
};
static inline void dbg_stage(uint32_t s) { *(volatile uint32_t *)DBG_STAGE_ADDR = s; }

// --- Bench debug: live USBHS host port-register snapshot ---------------------
// V5F stamps its USBHS host port registers here each host-wait iteration so V3F
// can print them over UART. This is the evidence for "device attached but the
// port CONNECT bit never sets" — it shows whether the PHY sees ANY bus activity
// (PORT_STATUS line state / speed bits / connect-change edge) while V5F waits.
// Reading USBHSH registers in firmware (V5F) is safe; it's reading them over SWD
// (wlink) that wedges the debug module — hence the firmware-side stamp.
// Layout (words): [0]=valid magic 0x55485250 'USBHSP' [1]=CFG [2]=PORT_CFG
// [3]=PORT_STATUS [4]=PORT_STATUS_CHG [5]=PORT_CTRL [6]=snapshot counter.
#define DBG_USBHS_REGS_ADDR  0x2017F010u
#define DBG_USBHS_REGS_MAGIC 0x55485250u
static inline void dbg_usbhs_regs(uint32_t cfg, uint32_t port_cfg,
                                  uint32_t port_status, uint32_t port_status_chg,
                                  uint32_t port_ctrl, uint32_t counter)
{
    volatile uint32_t *p = (volatile uint32_t *)DBG_USBHS_REGS_ADDR;
    p[1] = cfg; p[2] = port_cfg; p[3] = port_status;
    p[4] = port_status_chg; p[5] = port_ctrl; p[6] = counter;
    p[0] = DBG_USBHS_REGS_MAGIC;   // publish last so V3F never reads a torn snapshot
}
