#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "display.h"

// Inter-core channel. One single-producer/single-consumer FIFO (V3F->V5F
// injection commands), drained into the coherent IPC MSG mailbox by
// icc_pump_to_v5f (the SRAM ring is V3F-DTCM-local and unreadable by V5F). IPC
// channel 0 is the V3F->V5F doorbell. No V5F->V3F ring; V5F status rides the IPC
// CH1 stage telemetry (icc_telem_*).

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
    ICC_TAG_DEV_TEMP,     // device V3F -> V5F: device-board temp in b[0] (int8)
    // No V5F->V3F telemetry tags: that direction rides the IPC CH1 stage
    // telemetry (icc_telem_* below), not this FIFO.
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
// `magic` must stay first: V3F reads it raw at 0x20178000 (main_v3f.c) as the
// "V5F is up" handshake. Only V3F->V5F is carried, as a V3F-local producer FIFO
// that icc_pump_to_v5f drains into the IPC mailbox; a V5F->V3F ring would read
// back stale across the DTCM boundary.
typedef struct {
    volatile uint32_t magic;            // set by V3F at init
    icc_ring_t v3f_to_v5f;
} icc_shared_t;

#define ICC_MAGIC  0x48563343u          // 'HV3C'

void icc_init_v3f(void);   // master: zero rings, set magic, then signal via HSEM
void icc_init_v5f(void);   // slave: wait for magic via HSEM rendezvous

// Producer side (returns false if ring full). V3F->V5F only.
bool icc_send_to_v5f(const icc_record_t *r);   // call from V3F (enqueue to local FIFO)

// V3F: drain one queued V3F->V5F record from the local FIFO into the coherent IPC
// MSG mailbox if it is free. Call every V3F main-loop iteration. Returns true if a
// record was moved. The SRAM ring lives in V3F's DTCM (unreadable by V5F), so
// records cross via the IPC hardware mailbox.
bool icc_pump_to_v5f(void);                    // call from V3F

// Consumer side (returns false if empty). V5F reads the IPC mailbox.
bool icc_recv_from_v3f(icc_record_t *out);     // call from V5F

// Doorbell: V3F rings V5F after enqueue so V5F can wfi when idle.
void icc_ring_doorbell_v5f(void);              // V3F side
// V5F's IPC_CH0_Handler clears the doorbell and disables its own IT bit (storm-proof
// under AutoEN); the V5F foreground re-arms it after draining the mailbox.
void icc_ipc_rearm_v5f(void);                  // V5F side, call after draining

// --- V5F->V3F coherent stage telemetry (IPC CH1 status bits) -----------------
// V5F writing 0x2017xxxx is a cross-core store into V3F-side SRAM that
// intermittently stalls the V5F core (no-trap wedge); see icc.c top comment. The
// IPC status bits are peripheral-bus MMIO (0xE000D000), coherent across cores and
// single-writer here (only V5F writes CH1, only V3F reads) — a lock-free SPSC
// mailbox.
//
// Encoding: CH1 owns STS bits [8..15] — a 2-bit rolling heartbeat seq in [15:14]
// and a 6-bit stage/wedge code in [13:8]. A changing seq means V5F is alive; a
// frozen seq with a stuck code names where V5F wedged. icc_telem_stage_v5f sets
// the new byte and clears the stale bits; icc_telem_read_v3f returns the raw
// 8-bit value (seq<<6 | code).
void    icc_telem_stage_v5f(uint8_t code);     // V5F: publish a 6-bit stage code
uint8_t icc_telem_read_v3f(void);              // V3F: read seq<<6 | code

// Relay-loop stage codes (6-bit, distinct from the boot DBG_V5F_* SRAM markers).
enum {
    TLM_RLY_TOP     = 0x01,   // top of relay loop iteration
    TLM_RLY_DRAIN   = 0x02,   // after usb_merge_drain_icc
    TLM_RLY_DEVPOLL = 0x03,   // after usb_device_poll
    TLM_RLY_INPOLL  = 0x04,   // after host interrupt-IN poll
    TLM_RLY_MERGE   = 0x05,   // after usb_merge_report
    TLM_RLY_SEND    = 0x06,   // after usb_device_send_report
    TLM_RLY_OUT     = 0x07,   // after device-OUT loop
    TLM_RLY_PENDING = 0x08,   // after usb_merge_send_pending
    TLM_RLY_BOTTOM  = 0x09,   // end of iteration (pre-wfi)
};

// --- V5F->V3F reverse status channel (IPC status bits [16:31], CH2+CH3) -------
// Single-writer (V5F) coherent MMIO status, time-multiplexed; distinct from the
// CH1 [8:15] stage telemetry above. 16-bit word: [15:14]=seq, [13:10]=field
// selector, [9:0]=payload. V3F polls and reassembles into a display_status_t.
enum {                          // field selectors
    ICC_ST_SEL_STATE = 0,       // payload[2:0] = disp_state_t
    ICC_ST_SEL_VID_HI,          // payload[7:0] = vid >> 8
    ICC_ST_SEL_VID_LO,          // payload[7:0] = vid & 0xFF
    ICC_ST_SEL_PID_HI,          // payload[7:0] = pid >> 8
    ICC_ST_SEL_PID_LO,          // payload[7:0] = pid & 0xFF
    ICC_ST_SEL_RPS,             // payload[9:0] = reports_per_sec (clamped 0..1023)
    ICC_ST_SEL_DROPS,           // payload[9:0] = drops (clamped 0..1023)
    ICC_ST_SEL_PROBE,           // payload[7:4] = probe[3:0], payload[3:0] = gotmask[3:0]
    ICC_ST_SEL_WEDGE,           // payload[9:0] = wedge (clamped 0..1023)
    ICC_ST_SEL_SPEEDS,          // payload[3:2]=cap_speed, payload[1:0]=dev_speed
    ICC_ST_SEL_DEV,             // payload[9]=dev_link, payload[8]=dev_enum, payload[7:0]=dev_temp_c
    ICC_ST_SEL_HUMAN,           // payload[9:8]=warmth(0..2), payload[7:0]=replay_pct(0..100)
    ICC_ST_SEL__COUNT
};

// Pure (host-testable): pack one field of `st` into a 16-bit word with seq.
uint16_t icc_status_pack(uint8_t sel, uint8_t seq, const display_status_t *st);
// Pure: decode `word`, merging the carried field into `acc`. Returns the 2-bit seq.
uint8_t  icc_status_unpack(uint16_t word, display_status_t *acc);

// V5F: publish the next field in rotation (call on a throttle); publishes STATE
// immediately when `st->state` differs from the last published state.
void icc_status_pump_v5f(const display_status_t *st);
// V3F: read the current reverse word and merge into `acc`. Returns true if the
// heartbeat seq advanced since the last call (V5F alive and publishing).
bool icc_status_poll_v3f(display_status_t *acc);
// V3F: read the raw 16-bit reverse status word (IPC->STS bits [16:31]) without
// decoding it. Used by the V5F_STAGE_DIAG dump in main_v3f.c to avoid pulling
// IPC MMIO headers into that file.
uint16_t icc_status_read_raw_v3f(void);

// --- Bench debug: V5F boot-stage marker -------------------------------------
// A single volatile word at a fixed shared-SRAM address (past g_icc, below the
// V3F stack) that V5F overwrites as it advances through bring-up. Read it live
// with `wlink dump 0x2017F000 4` independent of the icc_shared_t layout.
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
    // Pre-USBHS window (between ICC_READY and usb_host_init's first marker 0x70).
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
    // Trap marker: HardFault_Handler ORs 0x80 over the low mcause nibble so a CPU
    // trap surfaces as a UART line instead of only a PC3 blink. e.g. 0x82 =
    // illegal-instruction, 0x85 = load-fault, 0x87 = store-fault, 0x81 =
    // instr-access-fault, 0x80 = instr-addr-misaligned, 0x8B = ecall-M.
    DBG_V5F_TRAP_BASE     = 0x80,              // 0x80 | (mcause & 0x0F)
};
static inline void dbg_stage(uint32_t s) { *(volatile uint32_t *)DBG_STAGE_ADDR = s; }
