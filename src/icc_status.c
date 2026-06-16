// icc_status.c — pure (host-testable) pack/unpack for the V5F->V3F reverse
// status channel. NO MMIO, NO WCH headers — includes only icc.h (stdint +
// display.h). The MMIO wrappers that USE these live in icc.c.
#include "icc.h"

// 6-bit name charset. Index = 6-bit value; entry = ASCII char.
// 0=space, 1..26=A..Z, 27..36=0..9, 37='-', 38='.', 39='+', 40=':', 41='/',
// 42..63=space (padding; unmapped decode to space).
static const char NAME_CHARSET[64] = {
    ' ','A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R',
    'S','T','U','V','W','X','Y','Z','0','1','2','3','4','5','6','7','8','9',
    '-','.','+',':','/',
    ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '
};

char icc_name_from6(uint8_t v) { return NAME_CHARSET[v & 0x3F]; }

uint8_t icc_name_to6(char c) {
    if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
    if (c >= 'A' && c <= 'Z') return (uint8_t)(1 + (c - 'A'));
    if (c >= '0' && c <= '9') return (uint8_t)(27 + (c - '0'));
    switch (c) {
        case '-': return 37; case '.': return 38; case '+': return 39;
        case ':': return 40; case '/': return 41;
        default:  return 0;   // space / unmapped
    }
}

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
        case ICC_ST_SEL_NAME: {
            uint8_t pos = (uint8_t)((payload >> 6) & 0xFu);
            acc->name[pos] = icc_name_from6((uint8_t)(payload & 0x3Fu));
            acc->name[16] = '\0';   // always keep terminated (name[17])
            break;
        }
        default: break;
    }
    return seq;
}
