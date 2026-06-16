#include "ch32h417_port.h"
#include "kmbox_cmd.h"
#include "uart.h"
#include "icc.h"
#include "proto.h"
#include "actions.h"
#include <string.h>

static void cmd_tx(const uint8_t *buf, uint16_t len) { uart_tx_write(buf, len); }

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
    uart_tx_flush();
}

void kmbox_cmd_inject_mouse(int16_t dx, int16_t dy, uint8_t buttons, int8_t wheel)
{
    icc_record_t r = { .tag = ICC_TAG_INJECT_MOUSE };
    r.b[0] = (uint8_t)(dx);     r.b[1] = (uint8_t)((uint16_t)dx >> 8);
    r.b[2] = (uint8_t)(dy);     r.b[3] = (uint8_t)((uint16_t)dy >> 8);
    r.b[4] = buttons;
    r.b[5] = (uint8_t)wheel;
    (void)icc_send_to_v5f(&r);   // doorbell is rung by icc_pump_to_v5f() once the
                                 // record reaches the coherent mailbox (see icc.c)
}

void kmbox_cmd_inject_keyboard(uint8_t modifier, const uint8_t keys[6])
{
    icc_record_t r = { .tag = ICC_TAG_INJECT_KEYBOARD };
    r.b[0] = modifier;
    for (int i = 0; i < 6; i++) r.b[1 + i] = keys[i];
    (void)icc_send_to_v5f(&r);   // doorbell is rung by icc_pump_to_v5f() once the
                                 // record reaches the coherent mailbox (see icc.c)
}

void kmbox_cmd_schedule_click_release(uint8_t button_mask, uint32_t delay_ms)
{
    icc_record_t r = { .tag = ICC_TAG_CLICK_RELEASE };
    r.b[0] = button_mask;
    r.b[1] = (uint8_t)(delay_ms);       r.b[2] = (uint8_t)(delay_ms >> 8);
    r.b[3] = (uint8_t)(delay_ms >> 16); r.b[4] = (uint8_t)(delay_ms >> 24);
    (void)icc_send_to_v5f(&r);   // doorbell is rung by icc_pump_to_v5f() once the
                                 // record reaches the coherent mailbox (see icc.c)
}

void kmbox_cmd_schedule_kb_release(uint8_t key, uint32_t delay_ms)
{
    icc_record_t r = { .tag = ICC_TAG_KB_RELEASE };
    r.b[0] = key;
    r.b[1] = (uint8_t)(delay_ms);       r.b[2] = (uint8_t)(delay_ms >> 8);
    r.b[3] = (uint8_t)(delay_ms >> 16); r.b[4] = (uint8_t)(delay_ms >> 24);
    (void)icc_send_to_v5f(&r);   // doorbell is rung by icc_pump_to_v5f() once the
                                 // record reaches the coherent mailbox (see icc.c)
}

void kmbox_cmd_set_baud(uint32_t baud)
{
    uart_set_baud(baud);
    icc_record_t r = { .tag = ICC_TAG_SET_BAUD };
    r.b[0]=(uint8_t)baud; r.b[1]=(uint8_t)(baud>>8);
    r.b[2]=(uint8_t)(baud>>16); r.b[3]=(uint8_t)(baud>>24);
    (void)icc_send_to_v5f(&r);   // doorbell is rung by icc_pump_to_v5f() once the
                                 // record reaches the coherent mailbox (see icc.c)
}

void kmbox_cmd_set_human_level(uint8_t level)
{
    icc_record_t r = { .tag = ICC_TAG_SET_HUMAN_LEVEL };
    r.b[0] = level;
    (void)icc_send_to_v5f(&r);   // doorbell is rung by icc_pump_to_v5f() once the
                                 // record reaches the coherent mailbox (see icc.c)
}
