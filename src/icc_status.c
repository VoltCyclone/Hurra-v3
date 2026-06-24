// icc_status.c — pure (host-testable) pack/unpack for the V5F->V3F reverse
// status channel. No MMIO, no WCH headers; includes only icc.h. The MMIO wrappers
// that use these live in icc.c.
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
        case ICC_ST_SEL_DROPS:  payload = (st->drops > 1023u) ? 1023u : st->drops; break;
        case ICC_ST_SEL_PROBE:  payload = (uint16_t)(((st->probe & 0xFu) << 4) | (st->gotmask & 0xFu)); break;
        case ICC_ST_SEL_WEDGE:  payload = (st->wedge > 1023u) ? 1023u : st->wedge; break;
        case ICC_ST_SEL_SPEEDS: payload = (uint16_t)(((st->cap_speed & 0x3u) << 2)
                                                     | (st->dev_speed & 0x3u)); break;
        case ICC_ST_SEL_DEV:    payload = (uint16_t)(((st->dev_link & 0x1u) << 9)
                                                     | ((st->dev_enum & 0x1u) << 8)
                                                     | ((uint8_t)st->dev_temp_c)); break;
        case ICC_ST_SEL_HUMAN: {
            uint16_t rp = (st->human_replay_pct > 100u) ? 100u : st->human_replay_pct;
            payload = (uint16_t)(((st->human_warmth & 0x3u) << 8) | rp);
            break;
        }
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
        case ICC_ST_SEL_VID_HI: acc->vid = (uint16_t)((acc->vid & 0x00FFu) | ((uint32_t)payload << 8)); break;
        case ICC_ST_SEL_VID_LO: acc->vid = (uint16_t)((acc->vid & 0xFF00u) | (payload & 0xFFu)); break;
        case ICC_ST_SEL_PID_HI: acc->pid = (uint16_t)((acc->pid & 0x00FFu) | ((uint32_t)payload << 8)); break;
        case ICC_ST_SEL_PID_LO: acc->pid = (uint16_t)((acc->pid & 0xFF00u) | (payload & 0xFFu)); break;
        case ICC_ST_SEL_RPS:    acc->reports_per_sec = payload; break;
        case ICC_ST_SEL_DROPS:  acc->drops = payload; break;
        case ICC_ST_SEL_PROBE:
            acc->probe   = (uint8_t)((payload >> 4) & 0xFu);
            acc->gotmask = (uint8_t)(payload & 0xFu);
            acc->zerolen = (uint8_t)(acc->probe & 0x1u);
            break;
        case ICC_ST_SEL_WEDGE:  acc->wedge = payload; break;
        case ICC_ST_SEL_SPEEDS:
            acc->cap_speed = (uint8_t)((payload >> 2) & 0x3u);
            acc->dev_speed = (uint8_t)(payload & 0x3u);
            break;
        case ICC_ST_SEL_DEV:
            acc->dev_link  = (uint8_t)((payload >> 9) & 0x1u);
            acc->dev_enum  = (uint8_t)((payload >> 8) & 0x1u);
            acc->dev_temp_c = (int8_t)(int16_t)(payload & 0xFFu);
            break;
        case ICC_ST_SEL_HUMAN:
            acc->human_warmth     = (uint8_t)((payload >> 8) & 0x3u);
            acc->human_replay_pct = (uint8_t)(payload & 0xFFu);
            break;
        default: break;
    }
    return seq;
}
