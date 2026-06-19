// src/kmbox_cmd_v5f_stub.c — V5F-only stubs for the kmbox_cmd_* injection sinks.
//
// actions.c is linked into both images and references kmbox_cmd_inject_* via the
// kmbox.h shim. On V3F those live in kmbox_cmd.c, but that file is not linked on
// V5F (it pulls in the V3F-only protocol parser and UART transport). V5F drains
// injection from the ICC in the merge path and only uses actions.c for the
// physical-mask state, so these sinks are reachable but never called. Empty
// stubs resolve the link while keeping a single shared actions.c across cores.

#include <stdint.h>

void kmbox_cmd_inject_mouse(int16_t dx, int16_t dy, uint8_t buttons, int8_t wheel)
{
    (void)dx; (void)dy; (void)buttons; (void)wheel;
}

void kmbox_cmd_inject_keyboard(uint8_t modifier, const uint8_t keys[6])
{
    (void)modifier; (void)keys;
}

void kmbox_cmd_schedule_click_release(uint8_t button_mask, uint32_t delay_ms)
{
    (void)button_mask; (void)delay_ms;
}

void kmbox_cmd_schedule_kb_release(uint8_t key, uint32_t delay_ms)
{
    (void)key; (void)delay_ms;
}
