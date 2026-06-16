// icc_status.c — pure (host-testable) pack/unpack for the V5F->V3F reverse
// status channel. NO MMIO, NO WCH headers — includes only icc.h (stdint +
// display.h). The MMIO wrappers that USE these live in icc.c.
#include "icc.h"

uint16_t icc_status_pack(uint8_t sel, uint8_t seq, const display_status_t *st)
{
    uint16_t payload = 0;
    switch (sel) {
        case ICC_ST_SEL_STATE:  payload = (uint16_t)(st->state & 0x07u); break;
        case ICC_ST_SEL_VID_HI: payload = (uint16_t)((st->vid >> 8) & 0xFFu); break;
        case ICC_ST_SEL_VID_LO: payload = (uint16_t)(st->vid & 0xFFu); break;
        case ICC_ST_SEL_PID_HI: payload = (uint16_t)((st->pid >> 8) & 0xFFu); break;
        case ICC_ST_SEL_PID_LO: payload = (uint16_t)(st->pid & 0xFFu); break;
        case ICC_ST_SEL_RPS:    payload = (st->reports_per_sec > 1023u)
                                          ? 1023u : st->reports_per_sec; break;
        default: break;
    }
    return (uint16_t)(((seq & 0x3u) << 14) | ((sel & 0xFu) << 10) | (payload & 0x3FFu));
}

uint8_t icc_status_unpack(uint16_t word, display_status_t *acc)
{
    uint8_t  seq = (uint8_t)((word >> 14) & 0x3u);
    uint8_t  sel = (uint8_t)((word >> 10) & 0xFu);
    uint16_t payload = (uint16_t)(word & 0x3FFu);
    switch (sel) {
        case ICC_ST_SEL_STATE:  acc->state = (uint8_t)(payload & 0x07u); break;
        case ICC_ST_SEL_VID_HI: acc->vid = (uint16_t)((acc->vid & 0x00FFu) | (payload << 8)); break;
        case ICC_ST_SEL_VID_LO: acc->vid = (uint16_t)((acc->vid & 0xFF00u) | (payload & 0xFFu)); break;
        case ICC_ST_SEL_PID_HI: acc->pid = (uint16_t)((acc->pid & 0x00FFu) | (payload << 8)); break;
        case ICC_ST_SEL_PID_LO: acc->pid = (uint16_t)((acc->pid & 0xFF00u) | (payload & 0xFFu)); break;
        case ICC_ST_SEL_RPS:    acc->reports_per_sec = payload; break;
        default: break;
    }
    return seq;
}
