#pragma once
// Shim: v2's actions.c calls kmbox_inject_*/kmbox_schedule_*; on v3 (V3F core)
// those route into the inter-core channel via kmbox_cmd_*. (The HID-merge half
// of v2's kmbox lives on V5F as usb_merge.c.)
#include "kmbox_cmd.h"
#define kmbox_inject_mouse           kmbox_cmd_inject_mouse
#define kmbox_inject_keyboard        kmbox_cmd_inject_keyboard
#define kmbox_schedule_click_release kmbox_cmd_schedule_click_release
#define kmbox_schedule_kb_release    kmbox_cmd_schedule_kb_release
