#include "ch32h417_port.h"
#include "kmbox_cmd.h"
#include "uart.h"
#include "icc.h"
#include "proto.h"
#include "actions.h"
#include <string.h>

static void cmd_tx(const uint8_t *buf, uint16_t len) { uart_tx_write(buf, len); }

static uint32_t s_inj_mouse_count;
static uint32_t s_inj_kbd_count;

void kmbox_cmd_init(void)
{
    act_init();
    proto_init();
    proto_set_tx(cmd_tx);
}

bool kmbox_cmd_rx_pending(void) { return uart_rx_available() > 0; }

void kmbox_cmd_poll(void)
{
    uint8_t buf[256];
    uint16_t n;
    while ((n = uart_rx_read(buf, sizeof buf)) > 0) {
        proto_feed(buf, n);
    }
    proto_tick();
    // Step any in-flight timed/curved auto-move (km.move with duration, bezier).
    // act_motion_move_dur/_bezier only arm g_motion; this per-iteration tick is the
    // sole thing that advances the trajectory and emits its per-tick deltas (via
    // act_move_raw -> kmbox_inject_mouse). proto_tick() handles only catch/baud, so
    // without this the motion would init but never move. Early-returns when no
    // program is in flight (g_motion.kind == MOTION_NONE), so calling it
    // unconditionally is free on the idle path.
    act_motion_tick();
    uart_tx_flush();
}

void kmbox_cmd_inject_mouse(int16_t dx, int16_t dy, uint8_t buttons, int8_t wheel)
{
    s_inj_mouse_count++;
    icc_record_t r = { .tag = ICC_TAG_INJECT_MOUSE };
    r.b[0] = (uint8_t)(dx);     r.b[1] = (uint8_t)((uint16_t)dx >> 8);
    r.b[2] = (uint8_t)(dy);     r.b[3] = (uint8_t)((uint16_t)dy >> 8);
    r.b[4] = buttons;
    r.b[5] = (uint8_t)wheel;
    (void)icc_send_to_v5f(&r);   // doorbell rung by icc_pump_to_v5f() (see icc.c)
}

void kmbox_cmd_inject_keyboard(uint8_t modifier, const uint8_t keys[6])
{
    s_inj_kbd_count++;
    icc_record_t r = { .tag = ICC_TAG_INJECT_KEYBOARD };
    r.b[0] = modifier;
    for (int i = 0; i < 6; i++) r.b[1 + i] = keys[i];
    (void)icc_send_to_v5f(&r);   // doorbell rung by icc_pump_to_v5f() (see icc.c)
}

void kmbox_cmd_schedule_click_release(uint8_t button_mask, uint32_t delay_ms)
{
    icc_record_t r = { .tag = ICC_TAG_CLICK_RELEASE };
    r.b[0] = button_mask;
    r.b[1] = (uint8_t)(delay_ms);       r.b[2] = (uint8_t)(delay_ms >> 8);
    r.b[3] = (uint8_t)(delay_ms >> 16); r.b[4] = (uint8_t)(delay_ms >> 24);
    (void)icc_send_to_v5f(&r);   // doorbell rung by icc_pump_to_v5f() (see icc.c)
}

void kmbox_cmd_schedule_kb_release(uint8_t key, uint32_t delay_ms)
{
    icc_record_t r = { .tag = ICC_TAG_KB_RELEASE };
    r.b[0] = key;
    r.b[1] = (uint8_t)(delay_ms);       r.b[2] = (uint8_t)(delay_ms >> 8);
    r.b[3] = (uint8_t)(delay_ms >> 16); r.b[4] = (uint8_t)(delay_ms >> 24);
    (void)icc_send_to_v5f(&r);   // doorbell rung by icc_pump_to_v5f() (see icc.c)
}

void kmbox_cmd_set_baud(uint32_t baud)
{
    uart_set_baud(baud);
    icc_record_t r = { .tag = ICC_TAG_SET_BAUD };
    r.b[0]=(uint8_t)baud; r.b[1]=(uint8_t)(baud>>8);
    r.b[2]=(uint8_t)(baud>>16); r.b[3]=(uint8_t)(baud>>24);
    (void)icc_send_to_v5f(&r);   // doorbell rung by icc_pump_to_v5f() (see icc.c)
}

uint32_t kmbox_cmd_inj_mouse_count(void) { return s_inj_mouse_count; }
uint32_t kmbox_cmd_inj_kbd_count(void)   { return s_inj_kbd_count; }
