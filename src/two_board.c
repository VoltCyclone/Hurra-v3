// two_board.c — two-board role loops (host = SPI master + USBHS capture; device =
// SPI slave + USB clone). See two_board.h and AGENTS.md.
//
// This is HARDWARE/MMIO bench scaffold: its gate is the bench, not a host unit
// test. The pure pieces it leans on (spi_frame codec, desc_xfer, usb_hs_desc,
// synth_mouse data) ARE host-tested.
#include "two_board.h"

#include "icc.h"            // dbg_stage() / DBG_V5F_* — UART-readable stage oracle
#include "led.h"
#include "spi_link.h"
#include "spi_frame.h"
#include "spi_frame_stream.h"   // IRQ-slave byte stream -> aligned frame slots
#include "synth_mouse.h"
#include "desc_xfer.h"      // descriptor blob chunk/reassemble codec (step 4b)
#include "desc_capture.h"   // capture_descriptors() — real host capture (step 4c)
#include "usb_host.h"       // usb_host_* — real USBHS host (step 4c)
#include "usb_device.h"
#include "usb_merge.h"
#include "timebase_v5f.h"
#include <string.h>

volatile int8_t g_tb_dev_temp_c;   // device-board temp from ICC_TAG_DEV_TEMP

/* Per-EP poll mapping for the real host capture (Board B). Mirrors main_v5f.c's
 * ep_mapping_t: each interrupt-IN endpoint is paced by its descriptor bInterval so
 * we don't flood the bus with NAKs. */
typedef struct {
    uint8_t  host_slot;
    uint8_t  dev_ep_num;
    uint16_t maxpkt;
    uint8_t  iface_protocol;   // 1=keyboard, 2=mouse (for usb_merge on Board A)
    uint32_t interval_us;
    uint32_t next_poll_us;
} tb_ep_map_t;

/* Send a whole serialized descriptor blob over the SPI link as a run of
 * TWO_BOARD_TYPE_DESC chunks (one SPI slot each, SEQ continuous from *seq). The
 * master clocks each slot; the return slot is ignored (restart-on-gap needs no
 * ACK). Used by Board B before/between mouse frames so a freshly-reset Board A can
 * always catch a full pass. */
// Inter-chunk pace (µs). The polled SPI slave needs time between slots to unpack
// the frame, run desc_xfer_accept, and re-enter spi_link_slave_exchange before the
// master's next CS edge — a back-to-back 325-slot burst out-runs it and the slave
// catches almost nothing (gate-4b 0xE0). Pacing each chunk to the rate the link is
// PROVEN at (cf. the 8ms mouse cadence / 2ms selftest) keeps the slave aligned.
// ~250µs × 325 chunks ≈ 80ms per blob — a one-time enumeration cost, well inside
// the 500ms periodic-resend cycle.
#define DESC_CHUNK_PACE_US  250u
static void send_descriptor_blob(const uint8_t *blob, uint16_t total, uint8_t *seq)
{
    uint16_t nchunks = desc_xfer_chunk_count(total);
    for (uint16_t i = 0; i < nchunks; i++) {
        uint8_t chunk[DESC_XFER_PAYLOAD_MAX];
        uint8_t clen = desc_xfer_pack_chunk(blob, total, i, chunk);
        uint8_t tx[SPI_LINK_SLOT];
        uint8_t rx[SPI_LINK_SLOT];
        if (spi_frame_pack(tx, TWO_BOARD_TYPE_DESC, (*seq)++, chunk, clen)
            == SPI_FRAME_OK) {
            spi_link_master_exchange(tx, rx);
            timebase_v5f_delay_us(DESC_CHUNK_PACE_US);
        }
        // DEFENSE IN DEPTH: the blob send is ~80 ms (325 chunks × ~250 µs). Keep the
        // IPC mailbox drained throughout so the V3F->V5F doorbell is acked + re-armed
        // and cannot back up while we're busy here. The ISR is already storm-proof
        // (it self-disables under AutoEN), but draining here keeps injection latency
        // low and the doorbell live. Cheap: a no-op when nothing is pending.
        usb_merge_drain_icc();
    }
}

/* Forward one captured report over SPI as a TWO_BOARD_TYPE_MOUSE frame. Payload
 * schema (Board A unpacks this exact layout): [dev_ep, iface_protocol, report…]. */
static void send_report_frame(uint8_t dev_ep, uint8_t protocol,
                              const uint8_t *report, uint8_t rlen, uint8_t *seq)
{
    if (rlen > SPI_FRAME_MAX_PAYLOAD - 2) rlen = SPI_FRAME_MAX_PAYLOAD - 2;
    uint8_t payload[SPI_FRAME_MAX_PAYLOAD];
    payload[0] = dev_ep & 0x0F;
    payload[1] = protocol;
    memcpy(&payload[2], report, rlen);

    uint8_t tx[SPI_LINK_SLOT], rx[SPI_LINK_SLOT];
    if (spi_frame_pack(tx, TWO_BOARD_TYPE_MOUSE, (*seq)++, payload,
                       (uint8_t)(2 + rlen)) == SPI_FRAME_OK) {
        spi_link_master_exchange(tx, rx);
    }
}

/* ======================================================================== */
/* Board B (USB host) = SPI MASTER.                                          */
/*                                                                           */
/* Step 4c: captures the REAL device on the USBHS host port, ships its       */
/* descriptor blob to Board A over SPI, then polls the real interrupt-IN     */
/* endpoints and forwards each captured report as an SPI mouse frame. The     */
/* descriptor blob is re-sent periodically (restart-on-gap) so Board A can    */
/* enumerate after an attach. SWJ is already disabled by main_v5f's host      */
/* divert (USBHS host uses PB8/9). Build with TWO_BOARD_HOST_SYNTH to use the */
/* synthetic source instead (isolated link test, no real device).            */
/* ======================================================================== */
void two_board_host_run(void)
{
    dbg_stage(DBG_V5F_RELAY);   // 0x58: host-role loop reached (reuse RELAY marker)

    spi_link_master_init();

    static captured_descriptors_t desc;   // static: large
    uint8_t  seq = 0;
    uint32_t last_blink_ms = millis();
    uint32_t last_desc_ms  = millis();
    const uint32_t desc_period_ms = 500;  // periodic blob re-send (restart-on-gap)

#if defined(TWO_BOARD_HOST_SYNTH)
    /* Isolated link test: synthetic mouse, no real USB host. */
    synth_mouse_build_descriptors(&desc);
    const uint8_t  *blob       = (const uint8_t *)&desc;
    const uint16_t  blob_total = (uint16_t)sizeof desc;
    uint32_t tick = 0, last_send_ms = millis();
    const uint32_t send_period_ms = 8;

    send_descriptor_blob(blob, blob_total, &seq);
    uint32_t diag_ms = millis();
    for (;;) {
        uint32_t now = millis();
        if ((now - last_desc_ms) >= desc_period_ms) {
            last_desc_ms = now; send_descriptor_blob(blob, blob_total, &seq);
        }
        if ((now - last_send_ms) >= send_period_ms) {
            last_send_ms = now;
            uint8_t report[SYNTH_MOUSE_REPORT_LEN];
            synth_mouse_next_report(tick++, report);
            send_report_frame(SYNTH_MOUSE_IN_EP, SYNTH_MOUSE_IFACE_PROTO,
                              report, SYNTH_MOUSE_REPORT_LEN, &seq);
        }
        /* Oracle: 0x5A = master clocking cleanly; 0xEA = the SPI master had to recover
         * a wedged exchange (bounded-wait timeout). Distinguishes a healthy link from
         * a marginal one without a logic analyzer. */
        if ((now - diag_ms) >= 1000u) {
            diag_ms = now;
            dbg_stage(spi_link_master_wedges ? 0xEA : 0x5A);
        }
        if ((now - last_blink_ms) >= 250u) { last_blink_ms = now; led_toggle(); }
    }
#else
    /* ---- Real USBHS host capture (SWJ already disabled by main_v5f) ---- */
    usb_host_init();
    dbg_stage(DBG_V5F_HOST_INIT);
    usb_host_power_on();

    /* Wait for a device, then capture its descriptors. A failed capture re-arms
     * the wait (device yanked / mid-glitch). Same dynamic-attach loop as the
     * single-board host (main_v5f.c). */
    for (;;) {
        dbg_stage(DBG_V5F_HOST_WAITING);
        while (!usb_host_device_connected()) {
            usb_host_power_on();
            if ((millis() - last_blink_ms) >= 250u) { last_blink_ms = millis(); led_toggle(); }
            __asm volatile("wfi");
        }
        dbg_stage(DBG_V5F_DEV_CONNECTED);
        led_on();
        timebase_v5f_delay_ms(10);
        if (capture_descriptors(&desc)) break;
        dbg_stage(0x9F);                  // capture failed — re-arm
        led_off();
        timebase_v5f_delay_ms(200);
    }
    dbg_stage(DBG_V5F_DESC_OK);

    /* Stamp the captured device speed so Board A clones at the SAME speed. */
    desc.speed = usb_host_device_speed();

    /* SET_PROTOCOL(Report) for BOOT-subclass HID interfaces only (so a boot mouse
     * streams report-protocol data). Best-effort; a device may legally STALL. */
    for (uint8_t i = 0; i < desc.num_ifaces; i++) {
        if (desc.ifaces[i].iface_class != 0x03) continue;
        if (desc.ifaces[i].iface_subclass != 0x01) continue;
        usb_setup_t sp = { .bmRequestType = 0x21, .bRequest = 0x0B,
                           .wValue = 1, .wIndex = desc.ifaces[i].iface_num,
                           .wLength = 0 };
        (void)usb_host_control_transfer(desc.dev_addr, desc.ep0_maxpkt, &sp, NULL, 2000);
    }

    /* Build the interrupt-IN poll map (ported from main_v5f.c). */
    tb_ep_map_t ep_map[MAX_INTR_EPS];
    uint8_t num_eps = 0;
    for (uint8_t i = 0; i < desc.num_ifaces; i++) {
        if (desc.ifaces[i].interrupt_in_ep == 0) continue;
        if (num_eps >= MAX_INTR_EPS) break;
        uint8_t slot = num_eps;
        uint8_t ep = desc.ifaces[i].interrupt_in_ep & 0x0F;
        usb_host_interrupt_init(slot, desc.dev_addr, ep,
                                desc.ifaces[i].interrupt_in_maxpkt);
        ep_map[slot].host_slot      = slot;
        ep_map[slot].dev_ep_num     = ep;
        ep_map[slot].maxpkt         = desc.ifaces[i].interrupt_in_maxpkt;
        ep_map[slot].iface_protocol = desc.ifaces[i].iface_protocol;
        uint32_t iv = desc.ifaces[i].interrupt_in_interval;
        if (iv == 0) iv = 1;
        if (iv > 255) iv = 255;
        uint32_t us = iv * 1000u;          // FS framing: bInterval is in ms
        if (us > 300u) us -= 250u;         // small lead so we don't poll late
        ep_map[slot].interval_us  = us;
        ep_map[slot].next_poll_us = timebase_v5f_us();
        num_eps++;
    }

    /* Ship the captured descriptor blob to Board A up front. */
    const uint8_t  *blob       = (const uint8_t *)&desc;
    const uint16_t  blob_total = (uint16_t)sizeof desc;
    send_descriptor_blob(blob, blob_total, &seq);
    last_desc_ms = millis();
    dbg_stage(DBG_V5F_RELAY);             // 0x58: relaying real reports over SPI

    for (;;) {
        uint32_t now = millis();

        /* Periodic descriptor re-send so a reset/late Board A can (re)enumerate,
         * gated on DRDY (PA3): Board A drives it high once enumerated, low from boot
         * until then. Only re-send while low — once A is up it ignores DESC frames and
         * the ~80 ms blocking send would stall the interrupt-IN poll (relative motion
         * deltas then flush in one jump). Self-healing: if A resets, PA3 falls low and
         * re-sends resume. */
        if (!spi_link_master_drdy() && (now - last_desc_ms) >= desc_period_ms) {
            last_desc_ms = now;
            send_descriptor_blob(blob, blob_total, &seq);
        }

        /* Poll each interrupt-IN EP at its paced interval; forward fresh reports. */
        uint32_t now_us = timebase_v5f_us();
        for (uint8_t m = 0; m < num_eps; m++) {
            if ((int32_t)(now_us - ep_map[m].next_poll_us) < 0) continue;
            ep_map[m].next_poll_us = now_us + ep_map[m].interval_us;
            uint8_t *rpt = NULL;
            int ret = usb_host_interrupt_poll_zerocopy(ep_map[m].host_slot, &rpt,
                                                       ep_map[m].maxpkt);
            if (ret > 0 && rpt) {
                send_report_frame(ep_map[m].dev_ep_num, ep_map[m].iface_protocol,
                                  rpt, (uint8_t)ret, &seq);
            }
        }

        if ((now - last_blink_ms) >= 250u) { last_blink_ms = now; led_toggle(); }
    }
#endif // TWO_BOARD_HOST_SYNTH
}

/* ======================================================================== */
/* Board A (USB device) = SPI SLAVE.                                         */
/*                                                                           */
/* Enumerates a synthetic boot mouse on the USBHSD device port, then receives  */
/* SPI frames, validates them (CRC + SEQ via the codec), merges/injects, and   */
/* forwards to the PC. A cumulative drop counter (CRC/SEQ failures) mirrors the */
/* relay's s_drop_count; a frozen counter + dead cursor localizes the fault.   */
/* ======================================================================== */
void two_board_device_run(void)
{
    static captured_descriptors_t desc;   // static: large; lives for program life

#if defined(TWO_BOARD_DEVICE_LOCAL)
    /* BENCH GATE B1 (Makefile BOARD=device DEVICE_LOCAL=1): drive the USBHSD clone
     * from a LOCAL synthetic generator, NO SPI. Isolates the USBHSD bring-up from
     * the link. Build descriptors locally and init up front. */
    synth_mouse_build_descriptors(&desc);
    usb_merge_cache_endpoints(&desc);
    if (!usb_device_init(&desc)) {
        led_blink_forever(9, 80, 120);    // init refused — should not happen
    }
    dbg_stage(DBG_V5F_DEV_INIT);          // 0x57: USBHSD device init done

    uint32_t wait_blink = millis();
    while (!usb_device_is_configured()) {
        usb_device_poll();
        usb_merge_drain_icc();
        if ((millis() - wait_blink) >= 250u) { wait_blink = millis(); led_toggle(); }
    }
    dbg_stage(DBG_V5F_RELAY);             // 0x58: configured — forwarding
    led_on();

    uint32_t tick = 0, send_ms = millis(), hb_ms = millis();
    for (;;) {
        if ((millis() - send_ms) >= 8u) {
            send_ms = millis();
            uint8_t rpt[SYNTH_MOUSE_REPORT_LEN];
            synth_mouse_next_report(tick++, rpt);
            usb_merge_report(SYNTH_MOUSE_IFACE_PROTO, rpt, SYNTH_MOUSE_REPORT_LEN);
            usb_device_send_report(SYNTH_MOUSE_IN_EP & 0x0F, rpt, SYNTH_MOUSE_REPORT_LEN);
        }
        usb_merge_drain_icc();
        usb_device_poll();
        if ((millis() - hb_ms) >= 250u) { hb_ms = millis(); led_toggle(); }
    }
#else
    /* STEP 4b: descriptors are NOT built locally — they arrive over SPI from
     * Board B. The SPI slave RX is INTERRUPT-DRIVEN: the RXNE ISR captures every
     * clocked byte into a ring, and a software SOF-scanner (spi_frame_stream)
     * extracts aligned 32-byte slots from the byte stream — robust to the
     * continuous descriptor burst that the polled slave could not frame.
     * Phase 1: receive + reassemble the blob. Phase 2: enumerate. Phase 3: relay. */
    spi_link_slave_init_irq();
    static spi_frame_stream_t stream;
    spi_frame_stream_init(&stream);

    /* ---- Phase 1: receive the descriptor blob (TWO_BOARD_TYPE_DESC chunks) ---- */
    dbg_stage(0x56);   // pre-enum: waiting for the descriptor blob over SPI
    static desc_xfer_ctx_t xfer;
    desc_xfer_reset(&xfer);
    uint32_t hb_ms = millis();
    bool got_blob = false;
    while (!got_blob) {
        uint8_t byte, slot[SPI_LINK_SLOT];
        while (spi_link_slave_rx_byte(&byte)) {
            if (!spi_frame_stream_push(&stream, byte, slot)) continue;
            uint8_t type, seq, len;
            const uint8_t *payload;
            if (spi_frame_unpack(slot, &type, &seq, &payload, &len) == SPI_FRAME_OK &&
                type == TWO_BOARD_TYPE_DESC) {
                desc_xfer_result_t r = desc_xfer_accept(&xfer, payload, len);
                if (r == DESC_XFER_COMPLETE && xfer.total == sizeof desc) {
                    memcpy(&desc, xfer.buf, sizeof desc);
                    got_blob = true;
                    break;
                }
            }
        }
        usb_merge_drain_icc();   // keep the ICC mailbox drained while waiting
        if ((millis() - hb_ms) >= 125u) { hb_ms = millis(); led_toggle(); }
    }

    /* ---- Phase 2: enumerate the clone from the transferred descriptors ---- */
    if (!desc.valid) {
        led_blink_forever(7, 80, 120);    // blob arrived but not a valid capture
    }
    usb_merge_cache_endpoints(&desc);
    if (!usb_device_init(&desc)) {
        led_blink_forever(9, 80, 120);
    }
    dbg_stage(DBG_V5F_DEV_INIT);          // 0x57: device init done

    uint32_t wait_blink = millis();
    while (!usb_device_is_configured()) {
        usb_device_poll();
        usb_merge_drain_icc();
        if ((millis() - wait_blink) >= 250u) { wait_blink = millis(); led_toggle(); }
    }
    dbg_stage(DBG_V5F_RELAY);             // 0x58: configured — forwarding
    led_on();

    /* Signal "enumerated" to Board B by asserting DATA_READY (PA3); Board B gates its
     * periodic descriptor re-send on this so it stops re-blasting the blob once we no
     * longer need it. Held low from boot through Phase 1/2 by spi_link's slave init. */
    spi_link_slave_set_drdy(1);

    /* ---- Phase 3: relay mouse reports (descriptor re-sends still arrive but the
     * device is already enumerated, so they're ignored) ---- */
    uint8_t  prev_seq = 0;
    bool     have_prev = false;
    uint32_t drop_count = 0;
    hb_ms = millis();
    for (;;) {
        uint8_t byte, slot[SPI_LINK_SLOT];
        while (spi_link_slave_rx_byte(&byte)) {
            if (!spi_frame_stream_push(&stream, byte, slot)) continue;
            uint8_t type, seq, len;
            const uint8_t *payload;
            if (spi_frame_unpack(slot, &type, &seq, &payload, &len) != SPI_FRAME_OK) {
                drop_count++;
                continue;
            }
            if (have_prev) {
                uint8_t gap = spi_frame_seq_gap(prev_seq, seq);
                if (gap > 1) drop_count += (gap - 1);
            }
            prev_seq = seq;
            have_prev = true;

            if (type == TWO_BOARD_TYPE_MOUSE && len >= 3) {
                uint8_t  dev_ep   = payload[0];
                uint8_t  protocol = payload[1];
                uint8_t  rlen     = (uint8_t)(len - 2);
                uint8_t  rpt[SPI_FRAME_MAX_PAYLOAD];
                memcpy(rpt, &payload[2], rlen);
                usb_merge_report(protocol, rpt, rlen);
                usb_device_send_report(dev_ep, rpt, rlen);
            }
        }
        usb_merge_drain_icc();
        usb_device_poll();

        /* Publish device->host telemetry on the SPI return slot (~every 100 ms).
         * The IRQ slave cycles this slot onto MISO; the host SOF-scans it. */
        static uint32_t tlm_ms;
        static uint8_t  tlm_seq;
        if ((millis() - tlm_ms) >= 100u) {
            tlm_ms = millis();
            uint8_t pay[3] = {
                (uint8_t)(usb_device_is_configured() ? 1u : 0u),
                usb_device_active_speed(),
                (uint8_t)g_tb_dev_temp_c,
            };
            uint8_t slot[SPI_LINK_SLOT];
            if (spi_frame_pack(slot, TWO_BOARD_TYPE_TELEM, tlm_seq++, pay, 3)
                == SPI_FRAME_OK) {
                spi_link_slave_set_telem(slot);
            }
        }

        if ((millis() - hb_ms) >= 250u) {
            hb_ms = millis();
            led_toggle();
            (void)drop_count;
        }
    }
#endif // TWO_BOARD_DEVICE_LOCAL
}
