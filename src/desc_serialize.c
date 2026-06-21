// desc_serialize.c — compact, versioned (de)serialization of
// captured_descriptors_t. See desc_serialize.h. Pure, MMIO-free, host-tested.
//
// Wire layout (little-endian, fixed section order). Every variable-length array is
// emitted as its companion length field followed by exactly that many bytes — never
// the fixed-array max — so the zero-filled tails that dominate the struct are never
// sent. The deserializer zeroes the output first, so those tails come back zeroed
// and a whole-struct memcmp round-trips byte-for-byte.
//
//   Header (12 B): MAGIC0 MAGIC1 VERSION FLAGS ep0_maxpkt dev_addr speed
//                  config_string_idx langid(u16) ms_os_vendor_code num_ifaces
//   DEVICE : u8 len, bytes        CONFIG: u16 len, bytes
//   LANGID : u8 len, bytes        BOS   : u16 len, bytes
//   MS_OS  : u8 len, bytes
//   IFACES : num_ifaces records (count from header); each mirrors captured_iface_t
//            scalars then has_hid_desc(u8), hid_report_desc_len(u16)+bytes
//   STRINGS: num_strings(u8), then per string: string_index(u8),
//            string_desc_len(u8)+bytes
#include "desc_serialize.h"
#include <string.h>

// ── write cursor ────────────────────────────────────────────────────────────
typedef struct { uint8_t *p; uint16_t cap; uint16_t off; bool ok; } wcur_t;

static void w_u8(wcur_t *w, uint8_t v)
{
    if (!w->ok || (uint32_t)w->off + 1u > w->cap) { w->ok = false; return; }
    w->p[w->off++] = v;
}
static void w_u16(wcur_t *w, uint16_t v)
{
    w_u8(w, (uint8_t)v);
    w_u8(w, (uint8_t)(v >> 8));
}
static void w_blob(wcur_t *w, const uint8_t *src, uint16_t n)
{
    if (!w->ok || (uint32_t)w->off + n > w->cap) { w->ok = false; return; }
    if (n) memcpy(&w->p[w->off], src, n);
    w->off += n;
}

uint16_t desc_serialize(const captured_descriptors_t *in, uint8_t *out, uint16_t out_cap)
{
    if (!in || !out) return 0;
    wcur_t w = { out, out_cap, 0, true };

    // Header
    w_u8(&w, DESC_WIRE_MAGIC0);
    w_u8(&w, DESC_WIRE_MAGIC1);
    w_u8(&w, DESC_WIRE_VERSION);
    w_u8(&w, (uint8_t)(in->valid ? 0x01u : 0x00u));
    w_u8(&w, in->ep0_maxpkt);
    w_u8(&w, in->dev_addr);
    w_u8(&w, in->speed);
    w_u8(&w, in->config_string_idx);
    w_u16(&w, in->langid);
    w_u8(&w, in->ms_os_vendor_code);

    uint8_t nif = in->num_ifaces; if (nif > MAX_INTERFACES) nif = MAX_INTERFACES;
    w_u8(&w, nif);

    // DEVICE
    { uint8_t n = in->device_desc_len;
      if (n > sizeof in->device_desc) n = (uint8_t)sizeof in->device_desc;
      w_u8(&w, n); w_blob(&w, in->device_desc, n); }
    // CONFIG
    { uint16_t n = in->config_desc_len; if (n > MAX_CONFIG_DESC_SIZE) n = MAX_CONFIG_DESC_SIZE;
      w_u16(&w, n); w_blob(&w, in->config_desc, n); }
    // LANGID
    { uint8_t n = in->langid_desc_len; if (n > MAX_LANGID_DESC_SIZE) n = MAX_LANGID_DESC_SIZE;
      w_u8(&w, n); w_blob(&w, in->langid_desc, n); }
    // BOS
    { uint16_t n = in->bos_desc_len; if (n > MAX_BOS_DESC_SIZE) n = MAX_BOS_DESC_SIZE;
      w_u16(&w, n); w_blob(&w, in->bos_desc, n); }
    // MS_OS
    { uint8_t n = in->ms_os_desc_len; if (n > MS_OS_1_0_STRING_SIZE) n = MS_OS_1_0_STRING_SIZE;
      w_u8(&w, n); w_blob(&w, in->ms_os_desc, n); }

    // IFACES
    for (uint8_t i = 0; i < nif; i++) {
        const captured_iface_t *f = &in->ifaces[i];
        w_u8(&w, f->iface_num);
        w_u8(&w, f->iface_class);
        w_u8(&w, f->iface_subclass);
        w_u8(&w, f->iface_protocol);
        w_u8(&w, f->iface_string_idx);
        w_u8(&w, f->interrupt_in_ep);
        w_u16(&w, f->interrupt_in_maxpkt);
        w_u8(&w, f->interrupt_in_interval);
        w_u8(&w, f->interrupt_out_ep);
        w_u16(&w, f->interrupt_out_maxpkt);
        w_u8(&w, f->interrupt_out_interval);
        w_u8(&w, (uint8_t)(f->has_hid_desc ? 1u : 0u));
        uint16_t hn = f->hid_report_desc_len;
        if (hn > MAX_HID_REPORT_DESC_SIZE) hn = MAX_HID_REPORT_DESC_SIZE;
        w_u16(&w, hn); w_blob(&w, f->hid_report_desc, hn);
    }

    // STRINGS
    uint8_t ns = in->num_strings; if (ns > MAX_STRINGS) ns = MAX_STRINGS;
    w_u8(&w, ns);
    for (uint8_t i = 0; i < ns; i++) {
        w_u8(&w, in->string_index[i]);
        uint8_t sn = in->string_desc_len[i]; if (sn > MAX_STRING_DESC_SIZE) sn = MAX_STRING_DESC_SIZE;
        w_u8(&w, sn); w_blob(&w, in->string_desc[i], sn);
    }

    return w.ok ? w.off : 0u;
}

// ── read cursor ─────────────────────────────────────────────────────────────
typedef struct { const uint8_t *p; uint16_t len; uint16_t off; bool ok; } rcur_t;

static uint8_t r_u8(rcur_t *r)
{
    if (!r->ok || (uint32_t)r->off + 1u > r->len) { r->ok = false; return 0; }
    return r->p[r->off++];
}
static uint16_t r_u16(rcur_t *r)
{
    uint16_t lo = r_u8(r);
    uint16_t hi = r_u8(r);
    return (uint16_t)(lo | (hi << 8));
}
// Copy n bytes into dst[0..cap-1]. If n > cap OR n > remaining input, fail closed.
static void r_blob(rcur_t *r, uint8_t *dst, uint16_t n, uint16_t cap)
{
    if (!r->ok || n > cap || (uint32_t)r->off + n > r->len) { r->ok = false; return; }
    if (n) memcpy(dst, &r->p[r->off], n);
    r->off += n;
}

bool desc_deserialize(const uint8_t *in, uint16_t len, captured_descriptors_t *out)
{
    if (!in || !out) return false;
    memset(out, 0, sizeof *out);     // tails come back zeroed -> byte-exact round-trip
    rcur_t r = { in, len, 0, true };

    if (r_u8(&r) != DESC_WIRE_MAGIC0)  return false;
    if (r_u8(&r) != DESC_WIRE_MAGIC1)  return false;
    if (r_u8(&r) != DESC_WIRE_VERSION) return false;   // unknown version -> fail closed
    uint8_t flags = r_u8(&r);
    out->ep0_maxpkt        = r_u8(&r);
    out->dev_addr          = r_u8(&r);
    out->speed             = r_u8(&r);
    out->config_string_idx = r_u8(&r);
    out->langid            = r_u16(&r);
    out->ms_os_vendor_code = r_u8(&r);
    uint8_t nif            = r_u8(&r);
    if (!r.ok) { memset(out, 0, sizeof *out); return false; }
    if (nif > MAX_INTERFACES) { memset(out, 0, sizeof *out); return false; }

    // DEVICE
    { uint8_t n = r_u8(&r); r_blob(&r, out->device_desc, n, (uint16_t)sizeof out->device_desc);
      out->device_desc_len = n; }
    // CONFIG
    { uint16_t n = r_u16(&r); r_blob(&r, out->config_desc, n, MAX_CONFIG_DESC_SIZE);
      out->config_desc_len = n; }
    // LANGID
    { uint8_t n = r_u8(&r); r_blob(&r, out->langid_desc, n, MAX_LANGID_DESC_SIZE);
      out->langid_desc_len = n; }
    // BOS
    { uint16_t n = r_u16(&r); r_blob(&r, out->bos_desc, n, MAX_BOS_DESC_SIZE);
      out->bos_desc_len = n; }
    // MS_OS
    { uint8_t n = r_u8(&r); r_blob(&r, out->ms_os_desc, n, MS_OS_1_0_STRING_SIZE);
      out->ms_os_desc_len = n; }

    // IFACES
    for (uint8_t i = 0; i < nif; i++) {
        captured_iface_t *f = &out->ifaces[i];
        f->iface_num              = r_u8(&r);
        f->iface_class            = r_u8(&r);
        f->iface_subclass         = r_u8(&r);
        f->iface_protocol         = r_u8(&r);
        f->iface_string_idx       = r_u8(&r);
        f->interrupt_in_ep        = r_u8(&r);
        f->interrupt_in_maxpkt    = r_u16(&r);
        f->interrupt_in_interval  = r_u8(&r);
        f->interrupt_out_ep       = r_u8(&r);
        f->interrupt_out_maxpkt   = r_u16(&r);
        f->interrupt_out_interval = r_u8(&r);
        f->has_hid_desc           = r_u8(&r) ? true : false;
        uint16_t hn = r_u16(&r);
        r_blob(&r, f->hid_report_desc, hn, MAX_HID_REPORT_DESC_SIZE);
        f->hid_report_desc_len = hn;
    }
    out->num_ifaces = nif;

    // STRINGS
    uint8_t ns = r_u8(&r);
    if (!r.ok || ns > MAX_STRINGS) { memset(out, 0, sizeof *out); return false; }
    for (uint8_t i = 0; i < ns; i++) {
        out->string_index[i] = r_u8(&r);
        uint8_t sn = r_u8(&r);
        r_blob(&r, out->string_desc[i], sn, MAX_STRING_DESC_SIZE);
        out->string_desc_len[i] = sn;
    }
    out->num_strings = ns;

    if (!r.ok) { memset(out, 0, sizeof *out); return false; }  // any shortfall -> fail closed
    out->valid = (flags & 0x01u) ? true : false;
    return true;
}
