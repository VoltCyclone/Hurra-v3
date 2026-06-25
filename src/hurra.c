// src/hurra.c — Hurra binary protocol parser (TinyFrame).
#include "TinyFrame.h"
#include "hurra.h"
#include "actions.h"
#include "kmbox.h"
#include "ch32h417_port.h"   // NVIC_SystemReset for the reboot command
#include <string.h>

extern uint32_t millis(void);

// ── Command TYPE byte allocation ────────────────────────────────────────────
enum {
    // 0x00–0x0F admin
    TYPE_PING        = 0x00,
    TYPE_VERSION     = 0x01,
    TYPE_STATS       = 0x02,
    TYPE_INIT        = 0x03,
    TYPE_REBOOT      = 0x04,
    TYPE_BAUD        = 0x05,
    TYPE_SCREEN      = 0x06,
    // 0x10–0x2F mouse
    TYPE_MOUSE_MOVE        = 0x10,
    TYPE_MOUSE_MOVE_SMOOTH = 0x11,
    TYPE_MOUSE_SILENT_MOVE = 0x12,
    TYPE_MOUSE_MO          = 0x13,
    TYPE_MOUSE_CLICK       = 0x14,
    TYPE_MOUSE_WHEEL       = 0x15,
    TYPE_MOUSE_GETPOS      = 0x16,
    TYPE_INVERT_X          = 0x17,
    TYPE_INVERT_Y          = 0x18,
    TYPE_SWAP_XY           = 0x19,
    TYPE_MOUSE_MOVE_DUR    = 0x1B,   // duration-stepped automove
    TYPE_MOUSE_MOVE_BEZIER = 0x1C,   // bezier move
    TYPE_BTN_LEFT          = 0x20,
    TYPE_BTN_RIGHT         = 0x21,
    TYPE_BTN_MIDDLE        = 0x22,
    TYPE_BTN_SIDE1         = 0x23,
    TYPE_BTN_SIDE2         = 0x24,
    // 0x40–0x4F keyboard
    TYPE_KB_DOWN        = 0x40,
    TYPE_KB_UP          = 0x41,
    TYPE_KB_PRESS       = 0x42,
    TYPE_KB_ISDOWN      = 0x43,
    TYPE_KB_MASK        = 0x44,
    TYPE_KB_STRING      = 0x45,
    TYPE_KB_MULTIDOWN   = 0x46,
    TYPE_KB_MULTIUP     = 0x47,
    TYPE_KB_MULTIPRESS  = 0x48,
    // 0x60–0x6F locks + catch
    TYPE_LOCK_ML  = 0x60,
    TYPE_LOCK_MR  = 0x61,
    TYPE_LOCK_MM  = 0x62,
    TYPE_LOCK_MS1 = 0x63,
    TYPE_LOCK_MS2 = 0x64,
    TYPE_LOCK_MX  = 0x65,
    TYPE_LOCK_MY  = 0x66,
    TYPE_CATCH_XY = 0x67,
    TYPE_PHYS_MASK = 0x68,   // physical-input mask / unmask_all
    // 0x70–0x73 reserved
    TYPE_CB_BUTTONS   = 0x74,
    TYPE_CB_AXES      = 0x75,
    TYPE_CB_KEYS      = 0x76,
    TYPE_CB_PHYS      = 0x77,   // physical-only telemetry enable
    // 0x80–0x8F on-change telemetry callbacks
    TYPE_TLM_AXIS    = 0x80,
    TYPE_TLM_BUTTONS = 0x81,
    // 0x82 reserved
    TYPE_TLM_KB      = 0x83,
    // 0x84–0x85 reserved
    TYPE_TLM_PHYS_AXIS    = 0x86,   // physical-only mouse motion (device→host)
    TYPE_TLM_PHYS_BUTTONS = 0x87,   // physical-only buttons (device→host)
    TYPE_TLM_PHYS_KB      = 0x88,   // physical-only keyboard (device→host)
};

#define HURRA_IDENTITY "kmbox: Hurra v1"

// ── TinyFrame instance ──────────────────────────────────────────────────────
static TinyFrame s_tf;
static hurra_tx_fn s_tx;

void TF_WriteImpl(TinyFrame *tf, const uint8_t *buf, uint32_t len)
{
    (void)tf;
    if (s_tx) s_tx(buf, (uint16_t)len);
}

// ── stats counters ──────────────────────────────────────────────────────────
static uint32_t s_rx_frames_ok;
static uint32_t s_head_crc_err;
static uint32_t s_payload_crc_err;
static uint32_t s_id_gap_total;
static uint32_t s_idle_resync;
static uint32_t s_payload_invalid;
static uint16_t s_tx_ring_high_water;
static uint32_t s_tx_ring_skip;
static uint8_t  s_last_rx_id;
static bool     s_have_last_id;

// ── deferred actions ─────────────────────────────────────────────────────────
static uint32_t s_reboot_at;
static uint32_t s_baud_pending;
static uint32_t s_baud_apply_at;

// ── callback state (on-change callbacks) ────────────────────────────────────
static uint8_t  s_cb_buttons, s_cb_axes, s_cb_keys;
static uint8_t  s_cb_phys;   // physical-only telemetry enable
static uint8_t  s_last_btn_emitted = 0;
static uint8_t  s_last_keys_emitted[6];

// ── screen ──────────────────────────────────────────────────────────────────
static int16_t s_screen_w, s_screen_h;

// ── catch_xy ────────────────────────────────────────────────────────────────
static struct {
    bool     active;
    uint32_t deadline;
    int32_t  accum_x, accum_y;
    uint8_t  reply_id;
} s_catch;

// ── helpers ─────────────────────────────────────────────────────────────────
static inline int16_t  rd_i16le(const uint8_t *p) { return (int16_t)(p[0] | (p[1] << 8)); }
static inline uint16_t rd_u16le(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t rd_u32le(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void send_reply(TF_Msg *req, const uint8_t *data, uint32_t len)
{
    TF_Msg r = *req;
    r.data = (uint8_t *)data;
    r.len  = (TF_LEN)len;
    r.is_response = true;
    TF_Respond(&s_tf, &r);
}

// Track frame-ID gap for stats; called by every listener.
static void track_id(uint8_t id)
{
    s_rx_frames_ok++;
    if (s_have_last_id) {
        uint8_t gap = (uint8_t)((id - s_last_rx_id - 1) & 0xFFu);
        s_id_gap_total += gap;
    }
    s_last_rx_id = id;
    s_have_last_id = true;
}

// ── admin listeners: PING / VERSION / STATS ──────────────────────────────────

static TF_Result l_ping(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len != 4) { s_payload_invalid++; return TF_STAY; }
    send_reply(msg, msg->data, 4);   // echo nonce
    return TF_STAY;
}

static TF_Result l_version(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    send_reply(msg, (const uint8_t *)HURRA_IDENTITY, sizeof(HURRA_IDENTITY) - 1);
    return TF_STAY;
}

static void pack_stats(uint8_t out[36])
{
    uint32_t uptime = millis();
    uint16_t ring_hw = s_tx_ring_high_water;
    s_tx_ring_high_water = 0;
    memcpy(&out[0],  &uptime,            4);
    memcpy(&out[4],  &s_rx_frames_ok,    4);
    memcpy(&out[8],  &s_head_crc_err,    4);
    memcpy(&out[12], &s_payload_crc_err, 4);
    memcpy(&out[16], &s_id_gap_total,    4);
    memcpy(&out[20], &s_idle_resync,     4);
    uint32_t over = kmbox_rx_drv_overrun();
    memcpy(&out[24], &over,              4);
    memcpy(&out[28], &s_tx_ring_skip,    4);
    memcpy(&out[32], &s_payload_invalid, 4);
    (void)ring_hw;  // reserved for a future 40-byte extension
}

static TF_Result l_stats(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    uint8_t buf[36];
    pack_stats(buf);
    send_reply(msg, buf, sizeof(buf));
    return TF_STAY;
}

// ── mouse listeners ───────────────────────────────────────────────────────────

static TF_Result l_mouse_move(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len != 4) { s_payload_invalid++; return TF_STAY; }
    int16_t dx = rd_i16le(&msg->data[0]);
    int16_t dy = rd_i16le(&msg->data[2]);
    act_move(dx, dy);
    return TF_STAY;
}

static TF_Result l_mouse_move_smooth(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len != 4) { s_payload_invalid++; return TF_STAY; }
    act_move(rd_i16le(&msg->data[0]), rd_i16le(&msg->data[2]));
    return TF_STAY;
}

static TF_Result l_mouse_silent(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len != 4) { s_payload_invalid++; return TF_STAY; }
    act_move(rd_i16le(&msg->data[0]), rd_i16le(&msg->data[2]));
    return TF_STAY;
}

static TF_Result l_mouse_mo(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len != 8) { s_payload_invalid++; return TF_STAY; }
    uint8_t buttons = msg->data[0];
    int16_t dx = rd_i16le(&msg->data[1]);
    int16_t dy = rd_i16le(&msg->data[3]);
    int8_t  wheel = (int8_t)msg->data[5];
    // pan/tilt (data[6], data[7]) accepted but dropped: no HID transport.
    act_button_set(buttons ^ g_buttons, 0);
    act_button_set(buttons, 1);
    act_move(dx, dy);
    if (wheel) act_wheel(wheel);
    return TF_STAY;
}

static TF_Result l_mouse_click(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len != 3) { s_payload_invalid++; return TF_STAY; }
    // payload: [button:u8, count:u8, delay_ms:u8]
    act_click(msg->data[0], msg->data[1], msg->data[2]);
    return TF_STAY;
}

static TF_Result l_mouse_wheel(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len != 1) { s_payload_invalid++; return TF_STAY; }
    act_wheel((int8_t)msg->data[0]);
    return TF_STAY;
}

// Automove: total (dx,dy) spread over dur_ms with a human velocity profile.
// Starts a motion program stepped from the poll loop; oneway, no reply.
// dur_ms==0 falls back to an immediate move.
static TF_Result l_mouse_move_dur(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len != 6) { s_payload_invalid++; return TF_STAY; }
    int16_t  dx  = rd_i16le(&msg->data[0]);
    int16_t  dy  = rd_i16le(&msg->data[2]);
    uint16_t dur = rd_u16le(&msg->data[4]);
    act_motion_move_dur(dx, dy, dur);
    return TF_STAY;
}

// Bezier: cubic curve from origin to (dx,dy) over dur_ms with control points
// (x1,y1),(x2,y2) relative to start. Oneway, no reply. Payload is 14 bytes:
// 7 little-endian int16 fields dx,dy,dur,x1,y1,x2,y2 — the host encoder must match.
#define BEZIER_PAYLOAD_LEN 14
static TF_Result l_mouse_move_bezier(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len != BEZIER_PAYLOAD_LEN) { s_payload_invalid++; return TF_STAY; }
    int16_t  dx  = rd_i16le(&msg->data[0]);
    int16_t  dy  = rd_i16le(&msg->data[2]);
    uint16_t dur = rd_u16le(&msg->data[4]);
    int16_t  x1  = rd_i16le(&msg->data[6]);
    int16_t  y1  = rd_i16le(&msg->data[8]);
    int16_t  x2  = rd_i16le(&msg->data[10]);
    int16_t  y2  = rd_i16le(&msg->data[12]);
    act_motion_bezier(dx, dy, dur, x1, y1, x2, y2);
    return TF_STAY;
}

static TF_Result l_mouse_getpos(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    int32_t x = g_pos_x, y = g_pos_y;
    if (x > INT16_MAX) x = INT16_MAX; else if (x < INT16_MIN) x = INT16_MIN;
    if (y > INT16_MAX) y = INT16_MAX; else if (y < INT16_MIN) y = INT16_MIN;
    uint8_t p[4] = {
        (uint8_t)x, (uint8_t)(x >> 8),
        (uint8_t)y, (uint8_t)(y >> 8),
    };
    send_reply(msg, p, sizeof(p));
    return TF_STAY;
}

// ── button listeners ──────────────────────────────────────────────────────────

static TF_Result button_listener(TinyFrame *tf, TF_Msg *msg, uint8_t mask)
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len == 0) {
        uint8_t v = (g_buttons & mask) ? 1 : 0;
        send_reply(msg, &v, 1);
        return TF_STAY;
    }
    if (msg->len != 1) { s_payload_invalid++; return TF_STAY; }
    act_button_set(mask, msg->data[0] ? 1 : 0);
    return TF_STAY;
}

#define MAKE_BTN(NAME, MASK) \
static TF_Result l_##NAME(TinyFrame *tf, TF_Msg *m) { return button_listener(tf, m, MASK); }

MAKE_BTN(btn_left,   0x01)
MAKE_BTN(btn_right,  0x02)
MAKE_BTN(btn_middle, 0x04)
MAKE_BTN(btn_side1,  0x08)
MAKE_BTN(btn_side2,  0x10)

static TF_Result invert_listener(TinyFrame *tf, TF_Msg *msg,
                                 bool (*get)(void), void (*set)(bool))
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len == 0) {
        uint8_t v = get() ? 1 : 0;
        send_reply(msg, &v, 1);
        return TF_STAY;
    }
    if (msg->len != 1) { s_payload_invalid++; return TF_STAY; }
    set(msg->data[0] != 0);
    return TF_STAY;
}

static TF_Result l_invert_x(TinyFrame *tf, TF_Msg *m)
{ return invert_listener(tf, m, act_get_invert_x, act_set_invert_x); }
static TF_Result l_invert_y(TinyFrame *tf, TF_Msg *m)
{ return invert_listener(tf, m, act_get_invert_y, act_set_invert_y); }
static TF_Result l_swap_xy(TinyFrame *tf, TF_Msg *m)
{ return invert_listener(tf, m, act_get_swap_xy, act_set_swap_xy); }

// ── keyboard listeners ────────────────────────────────────────────────────────

static TF_Result l_kb_down(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len != 1) { s_payload_invalid++; return TF_STAY; }
    act_kb_down(msg->data[0]);
    return TF_STAY;
}

static TF_Result l_kb_up(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len != 1) { s_payload_invalid++; return TF_STAY; }
    act_kb_up(msg->data[0]);
    return TF_STAY;
}

// xorshift32 jitter for KB_PRESS rand_ms
static uint32_t s_rng_state = 0xDEADBEEF;
static uint32_t rng_next(void)
{
    uint32_t x = s_rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return s_rng_state = x;
}

static TF_Result l_kb_press(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len != 3) { s_payload_invalid++; return TF_STAY; }
    uint32_t delay = msg->data[1] + (msg->data[2] ? (rng_next() % (msg->data[2] + 1u)) : 0);
    act_kb_press(msg->data[0], delay);
    return TF_STAY;
}

static TF_Result l_kb_isdown(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len != 1) { s_payload_invalid++; return TF_STAY; }
    uint8_t v = act_kb_isdown(msg->data[0]);
    send_reply(msg, &v, 1);
    return TF_STAY;
}

static TF_Result l_kb_mask(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len != 2) { s_payload_invalid++; return TF_STAY; }
    act_kb_mask(msg->data[0], msg->data[1]);
    return TF_STAY;
}

// ASCII → HID (lowercase letters, digits, space). Other chars dropped.
#define STRING_PRESS_MS 12
static uint8_t ascii_to_hid(char c)
{
    if (c >= 'a' && c <= 'z') return 0x04 + (c - 'a');
    if (c >= 'A' && c <= 'Z') return 0x04 + (c - 'A');
    if (c >= '1' && c <= '9') return 0x1E + (c - '1');
    if (c == '0') return 0x27;
    if (c == ' ') return 0x2C;
    return 0;
}

static TF_Result l_kb_string(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len > 240) { s_payload_invalid++; return TF_STAY; }
    for (uint16_t i = 0; i < msg->len; i++) {
        uint8_t hid = ascii_to_hid((char)msg->data[i]);
        if (hid) act_kb_press(hid, STRING_PRESS_MS);
    }
    return TF_STAY;
}

static TF_Result multikey_listener(TinyFrame *tf, TF_Msg *msg,
                                   void (*op)(uint8_t))
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len < 1 || msg->len > 6) { s_payload_invalid++; return TF_STAY; }
    for (uint16_t i = 0; i < msg->len; i++) op(msg->data[i]);
    return TF_STAY;
}

static void kb_press_default(uint8_t k) { act_kb_press(k, 80); }
static void kb_down_void(uint8_t k) { (void)act_kb_down(k); }

static TF_Result l_kb_multidown(TinyFrame *tf, TF_Msg *m)
{ return multikey_listener(tf, m, kb_down_void); }
static TF_Result l_kb_multiup(TinyFrame *tf, TF_Msg *m)
{ return multikey_listener(tf, m, act_kb_up); }
static TF_Result l_kb_multipress(TinyFrame *tf, TF_Msg *m)
{ return multikey_listener(tf, m, kb_press_default); }

// ── lock listeners + catch_xy ────────────────────────────────────────────────

static TF_Result lock_listener(TinyFrame *tf, TF_Msg *msg, uint8_t bit)
{
    (void)tf;
    track_id(msg->frame_id);
    extern uint16_t g_lock_mask;
    uint16_t bitmask = (uint16_t)(1u << bit);
    if (msg->len == 0) {
        uint8_t v = (g_lock_mask & bitmask) ? 1 : 0;
        send_reply(msg, &v, 1);
        return TF_STAY;
    }
    if (msg->len != 1) { s_payload_invalid++; return TF_STAY; }
    if (msg->data[0]) g_lock_mask |= bitmask;
    else              g_lock_mask &= ~bitmask;
    return TF_STAY;
}

#define MAKE_LOCK(NAME, BIT) \
static TF_Result l_##NAME(TinyFrame *tf, TF_Msg *m) { return lock_listener(tf, m, BIT); }

MAKE_LOCK(lock_ml,  0)
MAKE_LOCK(lock_mr,  1)
MAKE_LOCK(lock_mm,  2)
MAKE_LOCK(lock_ms1, 3)
MAKE_LOCK(lock_ms2, 4)
MAKE_LOCK(lock_mx,  5)
MAKE_LOCK(lock_my,  6)

static TF_Result l_catch_xy(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len != 2) { s_payload_invalid++; return TF_STAY; }
    uint16_t dur = rd_u16le(&msg->data[0]);
    if (dur > 1000) dur = 1000;

    // Re-entrant call: emit the prior result to its original requester first.
    if (s_catch.active) {
        uint8_t p[8];
        memcpy(&p[0], &s_catch.accum_x, 4);
        memcpy(&p[4], &s_catch.accum_y, 4);
        TF_Msg r;
        TF_ClearMsg(&r);
        r.type = TYPE_CATCH_XY;
        r.frame_id = s_catch.reply_id;
        r.is_response = true;
        r.data = p; r.len = 8;
        TF_Respond(&s_tf, &r);
    }
    s_catch.accum_x = 0;
    s_catch.accum_y = 0;
    s_catch.deadline = millis() + (uint32_t)dur;
    s_catch.reply_id = msg->frame_id;
    s_catch.active = true;
    return TF_STAY;
}

// mask / unmask_all. Payload 3 bytes: domain, code, enable.
//   domain 0 (mouse): code 0..6 = ml,mr,mm,ms1,ms2,mx,my; 7 = wheel
//   domain 1 (keyboard): code = HID keycode to mask/unmask
//   domain 0xFF: clear every active mask (unmask_all); code/enable ignored
// Oneway, no reply. Enforcement lives in the merge path.
static TF_Result l_phys_mask(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len != 3) { s_payload_invalid++; return TF_STAY; }
    uint8_t domain = msg->data[0];
    uint8_t code   = msg->data[1];
    bool    enable = msg->data[2] != 0;
    if (domain == 0xFF) {
        act_phys_unmask_all();
    } else if (domain == 0) {
        act_phys_mask_mouse(code, enable);
    } else if (domain == 1) {
        act_phys_mask_key(code, enable);
    } else {
        s_payload_invalid++;
    }
    return TF_STAY;
}

// ── admin listeners: INIT / REBOOT / BAUD / SCREEN ──────────────────────────

static TF_Result l_init(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    act_init();
    act_kb_init();
    s_rx_frames_ok = 0;  s_head_crc_err = 0; s_payload_crc_err = 0;
    s_id_gap_total = 0;  s_idle_resync  = 0; s_payload_invalid = 0;
    s_tx_ring_skip = 0;  s_tx_ring_high_water = 0;
    return TF_STAY;
}

static TF_Result l_reboot(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    uint8_t ok = 0;
    send_reply(msg, &ok, 1);
    s_reboot_at = millis() + 20;
    return TF_STAY;
}

static TF_Result l_baud(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len == 0) {
        uint32_t b = kmbox_current_baud();
        uint8_t p[4] = { (uint8_t)b, (uint8_t)(b >> 8), (uint8_t)(b >> 16), (uint8_t)(b >> 24) };
        send_reply(msg, p, 4);
        return TF_STAY;
    }
    if (msg->len != 4) { s_payload_invalid++; return TF_STAY; }
    uint32_t new_baud = rd_u32le(&msg->data[0]);
    uint8_t  ack[4]  = { (uint8_t)new_baud, (uint8_t)(new_baud >> 8),
                         (uint8_t)(new_baud >> 16), (uint8_t)(new_baud >> 24) };
    send_reply(msg, ack, 4);
    s_baud_pending  = new_baud;
    s_baud_apply_at = millis() + 20;
    return TF_STAY;
}

static TF_Result l_screen(TinyFrame *tf, TF_Msg *msg)
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len == 0) {
        uint8_t p[4] = {
            (uint8_t)s_screen_w, (uint8_t)(s_screen_w >> 8),
            (uint8_t)s_screen_h, (uint8_t)(s_screen_h >> 8),
        };
        send_reply(msg, p, 4);
        return TF_STAY;
    }
    if (msg->len != 4) { s_payload_invalid++; return TF_STAY; }
    s_screen_w = rd_i16le(&msg->data[0]);
    s_screen_h = rd_i16le(&msg->data[2]);
    return TF_STAY;
}

// ── callback configuration listeners ────────────────────────────────────────

static TF_Result cb_toggle_listener(TinyFrame *tf, TF_Msg *msg, uint8_t *flag)
{
    (void)tf;
    track_id(msg->frame_id);
    if (msg->len == 0) {
        send_reply(msg, flag, 1);
        return TF_STAY;
    }
    if (msg->len != 1) { s_payload_invalid++; return TF_STAY; }
    *flag = msg->data[0] ? 1 : 0;
    if (flag == &s_cb_keys) memset(s_last_keys_emitted, 0xFF, sizeof(s_last_keys_emitted));
    if (flag == &s_cb_buttons) s_last_btn_emitted = 0xFF;
    return TF_STAY;
}

static TF_Result l_cb_btn     (TinyFrame *tf, TF_Msg *m) { return cb_toggle_listener(tf, m, &s_cb_buttons); }
static TF_Result l_cb_axes    (TinyFrame *tf, TF_Msg *m) { return cb_toggle_listener(tf, m, &s_cb_axes);    }
static TF_Result l_cb_keys    (TinyFrame *tf, TF_Msg *m) { return cb_toggle_listener(tf, m, &s_cb_keys);    }
static TF_Result l_cb_phys    (TinyFrame *tf, TF_Msg *m) { return cb_toggle_listener(tf, m, &s_cb_phys);    }

// ── telemetry emit ──────────────────────────────────────────────────────────
// Skipped when the TX ring lacks room for the frame; only telemetry yields under
// backpressure, input listeners never do.
#define TLM_TX_SLACK 32

static void tlm_send(uint8_t type, const uint8_t *p, uint16_t n)
{
    if (kmbox_tx_room() < (n + TLM_TX_SLACK)) { s_tx_ring_skip++; return; }
    TF_Msg m;
    TF_ClearMsg(&m);
    m.type = type;
    m.data = (uint8_t *)p;
    m.len  = n;
    TF_Send(&s_tf, &m);
}

// ── public API ──────────────────────────────────────────────────────────────
void hurra_init(void)
{
    memset(&s_tf, 0, sizeof(s_tf));
    TF_InitStatic(&s_tf, TF_MASTER);
    s_tx = NULL;
    s_rx_frames_ok = s_head_crc_err = s_payload_crc_err = 0;
    s_id_gap_total = s_idle_resync = s_payload_invalid = 0;
    s_tx_ring_high_water = 0;
    s_tx_ring_skip = 0;
    s_have_last_id = false;
    s_reboot_at = 0;
    s_baud_pending = 0;
    s_baud_apply_at = 0;
    s_cb_buttons = s_cb_axes = s_cb_keys = 0;
    s_cb_phys = 0;
    memset(s_last_keys_emitted, 0, sizeof(s_last_keys_emitted));
    s_screen_w = s_screen_h = 0;
    memset(&s_catch, 0, sizeof(s_catch));
    TF_AddTypeListener(&s_tf, TYPE_PING,    l_ping);
    TF_AddTypeListener(&s_tf, TYPE_VERSION, l_version);
    TF_AddTypeListener(&s_tf, TYPE_STATS,   l_stats);
    TF_AddTypeListener(&s_tf, TYPE_MOUSE_MOVE,        l_mouse_move);
    TF_AddTypeListener(&s_tf, TYPE_MOUSE_MOVE_SMOOTH, l_mouse_move_smooth);
    TF_AddTypeListener(&s_tf, TYPE_MOUSE_SILENT_MOVE, l_mouse_silent);
    TF_AddTypeListener(&s_tf, TYPE_MOUSE_MO,          l_mouse_mo);
    TF_AddTypeListener(&s_tf, TYPE_MOUSE_CLICK,       l_mouse_click);
    TF_AddTypeListener(&s_tf, TYPE_MOUSE_WHEEL,       l_mouse_wheel);
    TF_AddTypeListener(&s_tf, TYPE_MOUSE_MOVE_DUR,    l_mouse_move_dur);
    TF_AddTypeListener(&s_tf, TYPE_MOUSE_MOVE_BEZIER, l_mouse_move_bezier);
    TF_AddTypeListener(&s_tf, TYPE_MOUSE_GETPOS,      l_mouse_getpos);
    TF_AddTypeListener(&s_tf, TYPE_BTN_LEFT,   l_btn_left);
    TF_AddTypeListener(&s_tf, TYPE_BTN_RIGHT,  l_btn_right);
    TF_AddTypeListener(&s_tf, TYPE_BTN_MIDDLE, l_btn_middle);
    TF_AddTypeListener(&s_tf, TYPE_BTN_SIDE1,  l_btn_side1);
    TF_AddTypeListener(&s_tf, TYPE_BTN_SIDE2,  l_btn_side2);
    TF_AddTypeListener(&s_tf, TYPE_INVERT_X,   l_invert_x);
    TF_AddTypeListener(&s_tf, TYPE_INVERT_Y,   l_invert_y);
    TF_AddTypeListener(&s_tf, TYPE_SWAP_XY,    l_swap_xy);
    TF_AddTypeListener(&s_tf, TYPE_KB_DOWN,       l_kb_down);
    TF_AddTypeListener(&s_tf, TYPE_KB_UP,         l_kb_up);
    TF_AddTypeListener(&s_tf, TYPE_KB_PRESS,      l_kb_press);
    TF_AddTypeListener(&s_tf, TYPE_KB_ISDOWN,     l_kb_isdown);
    TF_AddTypeListener(&s_tf, TYPE_KB_MASK,       l_kb_mask);
    TF_AddTypeListener(&s_tf, TYPE_KB_STRING,     l_kb_string);
    TF_AddTypeListener(&s_tf, TYPE_KB_MULTIDOWN,  l_kb_multidown);
    TF_AddTypeListener(&s_tf, TYPE_KB_MULTIUP,    l_kb_multiup);
    TF_AddTypeListener(&s_tf, TYPE_KB_MULTIPRESS, l_kb_multipress);
    TF_AddTypeListener(&s_tf, TYPE_LOCK_ML,  l_lock_ml);
    TF_AddTypeListener(&s_tf, TYPE_LOCK_MR,  l_lock_mr);
    TF_AddTypeListener(&s_tf, TYPE_LOCK_MM,  l_lock_mm);
    TF_AddTypeListener(&s_tf, TYPE_LOCK_MS1, l_lock_ms1);
    TF_AddTypeListener(&s_tf, TYPE_LOCK_MS2, l_lock_ms2);
    TF_AddTypeListener(&s_tf, TYPE_LOCK_MX,  l_lock_mx);
    TF_AddTypeListener(&s_tf, TYPE_LOCK_MY,  l_lock_my);
    TF_AddTypeListener(&s_tf, TYPE_CATCH_XY, l_catch_xy);
    TF_AddTypeListener(&s_tf, TYPE_PHYS_MASK, l_phys_mask);
    TF_AddTypeListener(&s_tf, TYPE_INIT,   l_init);
    TF_AddTypeListener(&s_tf, TYPE_REBOOT, l_reboot);
    TF_AddTypeListener(&s_tf, TYPE_BAUD,   l_baud);
    TF_AddTypeListener(&s_tf, TYPE_SCREEN, l_screen);
    TF_AddTypeListener(&s_tf, TYPE_CB_BUTTONS,   l_cb_btn);
    TF_AddTypeListener(&s_tf, TYPE_CB_AXES,      l_cb_axes);
    TF_AddTypeListener(&s_tf, TYPE_CB_KEYS,      l_cb_keys);
    TF_AddTypeListener(&s_tf, TYPE_CB_PHYS,      l_cb_phys);
}

void hurra_reset(void) { TF_ResetParser(&s_tf); }
void hurra_set_tx(hurra_tx_fn tx) { s_tx = tx; }

void hurra_feed_byte(uint8_t b) { TF_AcceptChar(&s_tf, b); }
void hurra_feed(const uint8_t *buf, uint16_t len) { TF_Accept(&s_tf, buf, (uint32_t)len); }

void hurra_tick(void)
{
    uint32_t now = millis();
    TF_Tick(&s_tf);

    if (s_catch.active && now >= s_catch.deadline) {
        uint8_t p[8];
        memcpy(&p[0], &s_catch.accum_x, 4);
        memcpy(&p[4], &s_catch.accum_y, 4);
        TF_Msg r;
        TF_ClearMsg(&r);
        r.type = TYPE_CATCH_XY;
        r.frame_id = s_catch.reply_id;
        r.is_response = true;
        r.data = p; r.len = 8;
        TF_Respond(&s_tf, &r);
        s_catch.active = false;
    }
    if (s_baud_apply_at && now >= s_baud_apply_at) {
        kmbox_set_baud(s_baud_pending);
        s_baud_apply_at = 0;
    }
    if (s_reboot_at && now >= s_reboot_at) {
        NVIC_SystemReset();
    }
}

void hurra_notify_buttons(uint8_t buttons)
{
    if (s_cb_buttons && buttons != s_last_btn_emitted) {
        s_last_btn_emitted = buttons;
        tlm_send(TYPE_TLM_BUTTONS, &buttons, 1);
    }
}

void hurra_notify_axes(int16_t dx, int16_t dy, int8_t scroll)
{
    if (s_catch.active) { s_catch.accum_x += dx; s_catch.accum_y += dy; }
    if (s_cb_axes) {
        uint8_t p[5] = {
            (uint8_t)dx, (uint8_t)(dx >> 8),
            (uint8_t)dy, (uint8_t)(dy >> 8),
            (uint8_t)scroll,
        };
        tlm_send(TYPE_TLM_AXIS, p, sizeof(p));
    }
}

void hurra_notify_keys(const uint8_t keys[6])
{
    if (s_cb_keys && memcmp(keys, s_last_keys_emitted, 6) != 0) {
        memcpy(s_last_keys_emitted, keys, 6);
        uint8_t p[7];
        p[0] = g_kb_modifier;
        memcpy(&p[1], keys, 6);
        tlm_send(TYPE_TLM_KB, p, sizeof(p));
    }
}

// ── physical-only telemetry (`monitor`) ─────────────────────────────────────
// Emitted by the merge path with pre-merge, pre-mask physical values when
// CB_PHYS is enabled, so a client can observe true input distinctly from merged
// state. Rate is bounded by the physical poll rate; yields to TX backpressure.
bool hurra_phys_enabled(void) { return s_cb_phys != 0; }

void hurra_notify_phys_axes(int16_t dx, int16_t dy, int8_t wheel)
{
    if (!s_cb_phys) return;
    uint8_t p[5] = {
        (uint8_t)dx, (uint8_t)(dx >> 8),
        (uint8_t)dy, (uint8_t)(dy >> 8),
        (uint8_t)wheel,
    };
    tlm_send(TYPE_TLM_PHYS_AXIS, p, sizeof(p));
}

void hurra_notify_phys_buttons(uint8_t buttons)
{
    if (!s_cb_phys) return;
    tlm_send(TYPE_TLM_PHYS_BUTTONS, &buttons, 1);
}

void hurra_notify_phys_keys(uint8_t modifier, const uint8_t keys[6])
{
    if (!s_cb_phys) return;
    uint8_t p[8];
    p[0] = modifier;
    p[1] = 0;                 // reserved, matches TLM_KB layout
    memcpy(&p[2], keys, 6);
    tlm_send(TYPE_TLM_PHYS_KB, p, sizeof(p));
}
