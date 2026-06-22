// two_board.c — two-board role loops (host = SPI master + USBHS capture; device =
// SPI slave + USB clone). See two_board.h and AGENTS.md. Bench-gated; the pure
// pieces it uses (spi_frame, desc_xfer, usb_hs_desc, synth_mouse) are host-tested.
#include "two_board.h"

#include "icc.h"            // dbg_stage() / DBG_V5F_* UART stage oracle
#include "display.h"
#include "led.h"
#include "spi_link.h"
#include "spi_frame.h"
#include "spi_frame_stream.h"
#include "synth_mouse.h"
#include "desc_xfer.h"
#include "desc_serialize.h"
#include "desc_capture.h"
#include "usb_host.h"
#include "usb_device.h"
#include "usb_merge.h"
#include "timebase_v5f.h"
#include "ws2812.h"
#include <string.h>

volatile int8_t g_tb_dev_temp_c;   // device-board temp from ICC_TAG_DEV_TEMP

/* Per-EP poll mapping for the real host capture (Board B). Each interrupt-IN
 * endpoint is paced by its descriptor bInterval to avoid flooding the bus with
 * NAKs. */
typedef struct {
    uint8_t  host_slot;
    uint8_t  dev_ep_num;
    uint16_t maxpkt;
    uint8_t  iface_protocol;   // 1=keyboard, 2=mouse (for usb_merge on Board A)
    uint32_t interval_us;
    uint32_t next_poll_us;
} tb_ep_map_t;

/* Send a serialized descriptor blob over the SPI link as a run of
 * TWO_BOARD_TYPE_DESC chunks (one slot each, SEQ continuous from *seq). The return
 * slot is ignored (restart-on-gap needs no ACK). */
// Inter-chunk pace (µs). The slave RX is interrupt-driven (RXNE ISR -> lock-free
// ring, spi_link.c), so the master no longer has to wait for the slave to unpack
// and re-arm between slots — the ring absorbs the burst. Per-chunk device work
// (ring drain + SOF-scan + CRC + desc_xfer_accept) is ~4-6µs at 400MHz, under one
// slot's ~5µs wire time at 50MHz. The whole blob is ~13 slots = ~416B, which the
// 2KB ring (LINK_RX_RING_SZ) holds ~5x over, so the pace is pure safety margin, not
// flow control. 5µs still leaves the device foreground time to service usb_device_poll
// between chunks. Bench-gated: if spi_link_rx_overflows rises off zero, raise this or
// LINK_RX_RING_SZ. Was 40µs (and 250µs in the polled-slave era) — both far above what
// the ring depth requires; the 40µs was sized before the RX ring absorbed the burst.
#define DESC_CHUNK_PACE_US  5u
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
        // Keep the IPC mailbox drained during the ~80 ms blob send so the V3F->V5F
        // doorbell stays acked/re-armed and injection latency stays low. No-op when
        // nothing is pending.
        usb_merge_drain_icc();
    }
}

/* Forward one captured report over SPI as a TWO_BOARD_TYPE_MOUSE frame. Payload
 * layout: [dev_ep, iface_protocol, report…]. rx_out receives the return slot so
 * the caller can feed it to host_absorb_return for device->host telemetry. */
static void send_report_frame(uint8_t dev_ep, uint8_t protocol,
                              const uint8_t *report, uint8_t rlen, uint8_t *seq,
                              uint8_t rx_out[SPI_LINK_SLOT])
{
    if (rlen > SPI_FRAME_MAX_PAYLOAD - 2) rlen = SPI_FRAME_MAX_PAYLOAD - 2;
    uint8_t payload[SPI_FRAME_MAX_PAYLOAD];
    payload[0] = dev_ep & 0x0F;
    payload[1] = protocol;
    memcpy(&payload[2], report, rlen);

    uint8_t tx[SPI_LINK_SLOT];
    if (spi_frame_pack(tx, TWO_BOARD_TYPE_MOUSE, (*seq)++, payload,
                       (uint8_t)(2 + rlen)) == SPI_FRAME_OK) {
        spi_link_master_exchange(tx, rx_out);
    } else if (rx_out) {
        memset(rx_out, 0, SPI_LINK_SLOT);
    }
}

/* Feed one received SPI return slot through the SOF-scanner; on a completed
 * TWO_BOARD_TYPE_TELEM frame, fold its fields into *disp and bump *fresh_ms when
 * the seq advances (device->host liveness heartbeat). */
static void host_absorb_return(spi_frame_stream_t *rxs, const uint8_t *rx,
                               display_status_t *disp, uint8_t *last_seq,
                               bool *have_seq, uint32_t *fresh_ms, uint32_t now)
{
    for (uint32_t i = 0; i < SPI_LINK_SLOT; i++) {
        uint8_t slot[SPI_LINK_SLOT];
        if (!spi_frame_stream_push(rxs, rx[i], slot)) continue;
        uint8_t type, seq, len; const uint8_t *pay;
        if (spi_frame_unpack(slot, &type, &seq, &pay, &len) != SPI_FRAME_OK) continue;
        if (type != TWO_BOARD_TYPE_TELEM || len < 3) continue;
        disp->dev_enum   = pay[0] ? 1u : 0u;
        disp->dev_speed  = pay[1];
        disp->dev_temp_c = (int8_t)pay[2];
        if (!*have_seq || seq != *last_seq) { *fresh_ms = now; }
        *last_seq = seq; *have_seq = true;
    }
}

/* ======================================================================== */
/* Board B (USB host) = SPI MASTER.                                          */
/*                                                                           */
/* Captures the device on the USBHS host port, ships its descriptor blob to  */
/* Board A over SPI, then polls the interrupt-IN endpoints and forwards each  */
/* captured report as an SPI mouse frame. The blob is re-sent periodically    */
/* (restart-on-gap) so Board A can enumerate after an attach. SWJ is disabled */
/* by main_v5f's host divert (USBHS host uses PB8/9). Build with              */
/* TWO_BOARD_HOST_SYNTH for an isolated link test with no real device.        */
/* ======================================================================== */
void two_board_host_run(void)
{
    dbg_stage(DBG_V5F_RELAY);   // 0x58: host-role loop reached

    spi_link_master_init();

    static captured_descriptors_t desc;   // static: large
    uint8_t  seq = 0;
    uint32_t last_blink_ms = millis();
    uint32_t last_desc_ms  = millis();
    const uint32_t desc_period_ms = 500;  // periodic blob re-send (restart-on-gap)

    // Descriptors ship in the compact serialized wire form (desc_serialize), not
    // as the 7136-byte struct: a real device populates only ~200-400 bytes, so this
    // cuts the blob ~25x and the per-attach transfer from ~80ms to a few ms. Built
    // once after capture (desc is immutable after) and reused for every re-send.
    static uint8_t desc_wire[DESC_WIRE_MAX_LEN];   // static: large

#if defined(TWO_BOARD_HOST_SYNTH)
    /* Isolated link test: synthetic mouse, no real USB host. */
    synth_mouse_build_descriptors(&desc);
    const uint8_t  *blob       = desc_wire;
    const uint16_t  blob_total = desc_serialize(&desc, desc_wire, sizeof desc_wire);
    uint32_t tick = 0, last_send_ms = millis();
    const uint32_t send_period_ms = 8;

    send_descriptor_blob(blob, blob_total, &seq);
    uint32_t diag_ms = millis();
    ws2812_init();
    for (;;) {
        uint32_t now = millis();
        if ((now - last_desc_ms) >= desc_period_ms) {
            last_desc_ms = now; send_descriptor_blob(blob, blob_total, &seq);
        }
        if ((now - last_send_ms) >= send_period_ms) {
            last_send_ms = now;
            uint8_t report[SYNTH_MOUSE_REPORT_LEN];
            uint8_t rx_scratch[SPI_LINK_SLOT];
            synth_mouse_next_report(tick++, report);
            send_report_frame(SYNTH_MOUSE_IN_EP, SYNTH_MOUSE_IFACE_PROTO,
                              report, SYNTH_MOUSE_REPORT_LEN, &seq, rx_scratch);
            ws2812_note_report(now);
        }
        /* Oracle: 0x5A = master clocking cleanly; 0xEA = a wedged exchange was
         * recovered (bounded-wait timeout). Distinguishes a healthy link from a
         * marginal one without a logic analyzer. */
        if ((now - diag_ms) >= 1000u) {
            diag_ms = now;
            dbg_stage(spi_link_master_wedges ? 0xEA : 0x5A);
        }
        if ((now - last_blink_ms) >= 250u) { last_blink_ms = now; led_toggle(); }
        ws2812_service(now, 1);   /* synth source is self-relaying */
    }
#else
    /* ---- Real USBHS host capture (SWJ already disabled by main_v5f) ---- */
    if (!usb_host_init())
        led_blink_forever(3, 80, 120);  // 3 pulses = USBHS 480M PLL never locked
    dbg_stage(DBG_V5F_HOST_INIT);
    usb_host_power_on();

    /* Wait for a device, then capture its descriptors. A failed capture re-arms
     * the wait (device yanked / mid-glitch). */
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
        dbg_stage(0x9F);                  // capture failed, re-arm
        led_off();
        timebase_v5f_delay_ms(200);
    }
    dbg_stage(DBG_V5F_DESC_OK);

    /* Stamp the captured device speed so Board A clones at the same speed. */
    desc.speed = usb_host_device_speed();

    /* Host TFT telemetry: fill our own display_status_t and pump it to V3F over
     * the IPC reverse status channel. */
    display_status_t disp = { .state = DISP_STATE_RELAYING };
    disp.vid = (uint16_t)(desc.device_desc[8]  | (desc.device_desc[9]  << 8));
    disp.pid = (uint16_t)(desc.device_desc[10] | (desc.device_desc[11] << 8));
    disp.cap_speed = desc.speed;
    uint32_t disp_ms = millis();
    uint32_t rep_count = 0, rep_ms = millis();

    /* Return-path (device->host) telemetry decode. */
    static spi_frame_stream_t rxs;
    spi_frame_stream_init(&rxs);
    uint8_t  last_telem_seq = 0;
    bool     have_telem_seq = false;
    uint32_t telem_fresh_ms = millis();   // last time the telem seq advanced
    uint32_t poll_ms = millis();          // last periodic poll-frame clock

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

    /* Build the interrupt-IN poll map. */
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
        if (us > 300u) us -= 250u;         // small lead to avoid polling late
        ep_map[slot].interval_us  = us;
        ep_map[slot].next_poll_us = timebase_v5f_us();
        num_eps++;
    }

    /* Serialize the captured descriptors to the compact wire form once (desc is
     * immutable after capture; speed was just stamped above so it lands in the wire
     * header) and ship it. Every periodic re-send reuses this buffer. */
    const uint8_t  *blob       = desc_wire;
    const uint16_t  blob_total = desc_serialize(&desc, desc_wire, sizeof desc_wire);
    send_descriptor_blob(blob, blob_total, &seq);
    last_desc_ms = millis();
    dbg_stage(DBG_V5F_RELAY);             // 0x58: relaying reports over SPI
    ws2812_init();

    for (;;) {
        uint32_t now = millis();

        /* Periodic descriptor re-send so a reset/late Board A can (re)enumerate,
         * gated on DRDY (PA3): Board A drives it high once enumerated, low until
         * then. Only re-send while low — once A is up the ~80 ms blocking send would
         * stall the interrupt-IN poll (relative-motion deltas flush in one jump). If
         * A resets, PA3 falls low and re-sends resume. */
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
                uint8_t rx[SPI_LINK_SLOT];
                send_report_frame(ep_map[m].dev_ep_num, ep_map[m].iface_protocol,
                                  rpt, (uint8_t)ret, &seq, rx);
                ws2812_note_report(now);
                host_absorb_return(&rxs, rx, &disp, &last_telem_seq,
                                   &have_telem_seq, &telem_fresh_ms, now);
                rep_count++;
            }
        }

        /* Idle poll: if no report was clocked recently, send an empty TELEM_REQ so
         * the device's MISO slot keeps flowing (else a still mouse would falsely
         * read LINK DOWN). */
        if ((now - poll_ms) >= 250u) {
            poll_ms = now;
            uint8_t tx[SPI_LINK_SLOT], rx[SPI_LINK_SLOT];
            if (spi_frame_pack(tx, TWO_BOARD_TYPE_TELEM_REQ, seq++, NULL, 0)
                == SPI_FRAME_OK) {
                spi_link_master_exchange(tx, rx);
                host_absorb_return(&rxs, rx, &disp, &last_telem_seq,
                                   &have_telem_seq, &telem_fresh_ms, now);
            }
        }

        /* Fill host-local fields + freshness, then pump one field to V3F. */
        if ((now - rep_ms) >= 1000u) {
            rep_ms = now;
            disp.reports_per_sec = (rep_count > 1023u) ? 1023u : (uint16_t)rep_count;
            rep_count = 0;
            disp.wedge = (uint16_t)(spi_link_master_wedges > 1023u
                                    ? 1023u : spi_link_master_wedges);
        }
        disp.dev_link = ((now - telem_fresh_ms) < 1000u) ? 1u : 0u;
        if ((now - disp_ms) >= 50u) {   // throttle the rotating field pump
            disp_ms = now;
            icc_status_pump_v5f(&disp);
        }

        if ((now - last_blink_ms) >= 250u) { last_blink_ms = now; led_toggle(); }
        /* Board A drives DRDY high once enumerated downstream = "relaying". */
        ws2812_service(now, spi_link_master_drdy());
    }
#endif // TWO_BOARD_HOST_SYNTH
}

/* ======================================================================== */
/* Board A (USB device) = SPI SLAVE.                                         */
/*                                                                           */
/* Enumerates the clone on the USBHSD device port, then receives SPI frames, */
/* validates them (CRC + SEQ), merges/injects, and forwards to the PC. A      */
/* cumulative drop counter (CRC/SEQ failures) plus a dead cursor localizes a  */
/* fault to a specific hop.                                                   */
/* ======================================================================== */
void two_board_device_run(void)
{
    static captured_descriptors_t desc;   // static: large; lives for program life

#if defined(TWO_BOARD_DEVICE_LOCAL)
    /* DEVICE_LOCAL: drive the USBHSD clone from a local synthetic generator, no
     * SPI, to isolate USBHSD bring-up from the link. */
    synth_mouse_build_descriptors(&desc);
    usb_merge_cache_endpoints(&desc);
    if (!usb_device_init(&desc)) {
        led_blink_forever(9, 80, 120);    // init refused
    }
    dbg_stage(DBG_V5F_DEV_INIT);          // 0x57: USBHSD device init done

    uint32_t wait_blink = millis();
    while (!usb_device_is_configured()) {
        usb_device_poll();
        usb_merge_drain_icc();
        if ((millis() - wait_blink) >= 250u) { wait_blink = millis(); led_toggle(); }
    }
    dbg_stage(DBG_V5F_RELAY);             // 0x58: configured, forwarding
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
    /* Descriptors arrive over SPI from Board B. The slave RX is interrupt-driven:
     * the RXNE ISR captures every clocked byte into a ring, and a software
     * SOF-scanner (spi_frame_stream) extracts aligned 32-byte slots, robust to the
     * continuous descriptor burst. Phase 1: receive + reassemble the blob. Phase 2:
     * enumerate. Phase 3: relay. */
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
                if (r == DESC_XFER_COMPLETE) {
                    // Reassembled blob is the compact wire form; deserialize it back
                    // into the struct. A malformed/truncated/wrong-version blob fails
                    // closed — drop it and keep waiting (host re-sends every 500ms).
                    if (desc_deserialize(xfer.buf, xfer.total, &desc)) {
                        got_blob = true;
                        break;
                    }
                    desc_xfer_reset(&xfer);
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
    dbg_stage(DBG_V5F_RELAY);             // 0x58: configured, forwarding
    led_on();

    /* Assert DATA_READY (PA3) to signal "enumerated"; Board B gates its periodic
     * descriptor re-send on this. Held low from boot through Phase 1/2 by the slave
     * init. */
    spi_link_slave_set_drdy(1);

    /* ---- Phase 3: relay mouse reports (descriptor re-sends still arrive but are
     * ignored once enumerated) ---- */
    uint8_t  prev_seq = 0;
    bool     have_prev = false;
    uint32_t drop_count = 0;
    hb_ms = millis();
    ws2812_init();
    for (;;) {
        uint32_t now = millis();
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
                ws2812_note_report(now);
            }
        }
        usb_merge_drain_icc();
        usb_device_poll();

        /* Publish device->host telemetry on the SPI return slot every ~100 ms. The
         * IRQ slave cycles this slot onto MISO; the host SOF-scans it. */
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
        /* Clone enumerated on the PC = "relaying"; else pulse red. */
        ws2812_service(now, usb_device_is_configured());
    }
#endif // TWO_BOARD_DEVICE_LOCAL
}
