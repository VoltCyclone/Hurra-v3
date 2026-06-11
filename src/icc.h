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
bool icc_send_to_v5f(const icc_record_t *r);   // call from V3F
bool icc_send_to_v3f(const icc_record_t *r);   // call from V5F

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
    DBG_V5F_ICC_READY     = 0x52,              // ICC rendezvous done
    DBG_V5F_HOST_INIT     = 0x53,              // USBHS host init done, entering host-wait
    DBG_V5F_HOST_WAITING  = 0x54,              // spinning in while(!device_connected)
    DBG_V5F_DEV_CONNECTED = 0x55,              // host saw a device, leaving host-wait
    DBG_V5F_DESC_OK       = 0x56,              // descriptors captured
    DBG_V5F_DEV_INIT      = 0x57,              // USBFS device init done (cloning to PC)
    DBG_V5F_RELAY         = 0x58,              // reached the relay loop (telemetry flows)
};
static inline void dbg_stage(uint32_t s) { *(volatile uint32_t *)DBG_STAGE_ADDR = s; }
