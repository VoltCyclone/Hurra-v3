// src/kmbox_cmd_v5f_stub.c — V5F-only stubs for the kmbox_cmd_* injection sinks.
//
// Why this file exists:
//   actions.c is linked into BOTH images. It includes kmbox.h, whose shim maps
//   kmbox_inject_* -> kmbox_cmd_inject_* (the ICC encoders). On V3F those live
//   in kmbox_cmd.c. On V5F kmbox_cmd.c is NOT linked (it pulls in the protocol
//   parser + UART transport, which are V3F-only), so the four kmbox_cmd_*
//   symbols would be undefined at link time.
//
//   On V5F, however, actions.c's act_move / act_click / act_kb_* path is never
//   exercised: V5F runs the *merge*, which drains injection directly from the
//   ICC (usb_merge_drain_icc) and never goes through actions.c's command sinks.
//   V5F only needs actions.c for the physical-mask STATE (g_phys_mask) and the
//   act_phys_* query helpers, which do not touch kmbox_cmd_*. The four sinks are
//   therefore reachable in the object file but never called on V5F.
//
//   Providing empty stubs here resolves the link minimally and safely (the
//   symbols exist but are unreachable on V5F). This is cleaner than splitting
//   actions.c, and keeps a single shared actions.c across both cores.

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
