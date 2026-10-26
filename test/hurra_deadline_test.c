#include <assert.h>
#include <stdint.h>
#include <string.h>
#include "../src/hurra.c"
static uint32_t now, baud_changes, reboots, writes;
uint32_t millis(void) { return now; }
void NVIC_SystemReset(void) { reboots++; }
uint32_t uart_current_baud(void) { return 921600; }
uint16_t uart_tx_room(void) { return 4095; }
uint32_t uart_overrun(void) { return 0; }
void kmbox_cmd_set_baud(uint32_t baud) { assert(baud == 115200); baud_changes++; }
void kmbox_cmd_inject_mouse(int16_t x, int16_t y, uint8_t b, int8_t w) {(void)x;(void)y;(void)b;(void)w;}
void kmbox_cmd_inject_keyboard(uint8_t m, const uint8_t k[6]) {(void)m;(void)k;}
void kmbox_cmd_schedule_click_release(uint8_t b, uint32_t d) {(void)b;(void)d;}
void kmbox_cmd_schedule_kb_release(uint8_t k, uint32_t d) {(void)k;(void)d;}
static void tx(const uint8_t *p, uint16_t n) {assert(p && n); writes++;}
int main(void) {
    hurra_init(); hurra_set_tx(tx); act_init();
    hurra_feed_byte(TF_SOF_BYTE);
    for (unsigned i = 0; i < 100; i++) hurra_tick();
    assert(s_tf.parser_timeout_ticks == 0);
    now = 1; hurra_tick(); assert(s_tf.parser_timeout_ticks == 1);
    now = 10000; hurra_feed_byte(TF_SOF_BYTE); hurra_tick();
    assert(s_tf.parser_timeout_ticks == 0); // prior idle is not charged to fresh input
    now += 4; hurra_tick(); assert(s_tf.parser_timeout_ticks == 4);
    now++; hurra_tick(); assert(s_tf.parser_timeout_ticks == 5);
    now = UINT32_MAX - 1u; hurra_reset(); hurra_feed_byte(TF_SOF_BYTE);
    now = 1; hurra_tick(); assert(s_tf.parser_timeout_ticks == 3);

    TF_Msg msg; TF_ClearMsg(&msg);
    uint8_t duration[] = {20,0}; msg.data = duration; msg.len = 2;
    now = UINT32_MAX - 9u; l_catch_xy(&s_tf, &msg); writes = 0;
    now = UINT32_MAX - 1u; hurra_tick(); assert(writes == 0);
    now = 10; hurra_tick(); assert(writes == 1 && !s_catch.active);

    uint8_t baud[] = {0x00,0xC2,0x01,0}; msg.data = baud; msg.len = 4;
    now = UINT32_MAX - 19u; l_baud(&s_tf, &msg); assert(s_baud_apply_at == 0);
    now = UINT32_MAX - 1u; hurra_tick(); assert(baud_changes == 0);
    now = 0; hurra_tick(); assert(baud_changes == 1);
    hurra_tick(); assert(baud_changes == 1);
    now = UINT32_MAX - 19u; l_reboot(&s_tf, &msg); assert(s_reboot_at == 0);
    now = UINT32_MAX - 1u; hurra_tick(); assert(reboots == 0);
    now = 0; hurra_tick(); assert(reboots == 1);
    hurra_tick(); assert(reboots == 1);
    return 0;
}
