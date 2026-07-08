// Host unit test for descriptor truncation-consistency patching in
// src/desc_capture.c. Pure host build (no MMIO): the real capture_descriptors()
// is compiled in via #include "desc_capture.c" and driven by a scripted mock
// usb_host_control_transfer() that models a device whose declared descriptor
// lengths exceed our capture caps. Built/run via `make test`.
//
// What it proves (the ported v2 "descriptor truncation-consistency" fix):
//   * config descriptor wTotalLength is patched down to the actually-stored
//     length when the device advertised more than MAX_CONFIG_DESC_SIZE;
//   * the HID descriptor's wDescriptorLength inside the config blob is patched
//     to the actually-captured report-descriptor length (patch_hid_report_desc_len);
//   * a BOS wTotalLength is patched when the BOS was truncated;
//   * a string descriptor's bLength is clamped to the captured length;
//   * a failed HID report-descriptor fetch clears has_hid_desc AND zeroes the
//     length (don't replay an invalid descriptor).
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "usb_host.h"      // usb_setup_t + USB_* constants (pure header, no MMIO)
#include "desc_capture.h"

#define CHECK(cond) do { if (!(cond)) { \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); assert(0); } } while (0)

// ── Scripted device model ────────────────────────────────────────────────────
// A composite HID device (mouse iface 0 + keyboard iface 1) that lies about its
// descriptor sizes so every truncation path is exercised in one enumeration.
#define DEV_CONFIG_TOTAL   600u   // > MAX_CONFIG_DESC_SIZE (512): config truncates
#define DEV_BOS_TOTAL      700u   // > MAX_BOS_DESC_SIZE (256/512): BOS truncates
#define IFACE0_HID_DECL   1024u   // declared wDescriptorLength (> cap): report truncates
#define IFACE0_HID_ACTUAL  300u   // bytes the device actually returns for the report
#define IFACE1_HID_DECL    200u   // declared; its report FETCH fails (returns < 0)
#define STR1_DECL_BLEN      40u   // string bLength the device claims (> captured len)
#define STR1_ACTUAL_LEN     12u   // bytes actually returned for string index 1

// Meaningful config descriptor prefix (59 bytes); the device pads the rest with
// zeros up to the requested (clamped) length. Offsets referenced by assertions:
//   [2..3]   config wTotalLength
//   [25..26] iface0 HID wDescriptorLength (offset 18 HID desc + 7)
//   [50..51] iface1 HID wDescriptorLength (offset 43 HID desc + 7)
static const uint8_t k_config_prefix[59] = {
    // config header (9)
    9, USB_DESC_CONFIGURATION, (DEV_CONFIG_TOTAL & 0xFF), (DEV_CONFIG_TOTAL >> 8),
    2, 1, 0, 0x80, 50,
    // iface 0: HID mouse, 1 EP (9)
    9, USB_DESC_INTERFACE, 0, 0, 1, 3, 1, 2, 0,
    // iface 0 HID desc (9): wDescriptorLength = IFACE0_HID_DECL
    9, USB_DESC_HID, 0x11, 0x01, 0, 1, USB_DESC_HID_REPORT,
        (IFACE0_HID_DECL & 0xFF), (IFACE0_HID_DECL >> 8),
    // iface 0 interrupt IN EP (7)
    7, USB_DESC_ENDPOINT, 0x81, 0x03, 4, 0, 1,
    // iface 1: HID keyboard, 1 EP (9)
    9, USB_DESC_INTERFACE, 1, 0, 1, 3, 1, 1, 0,
    // iface 1 HID desc (9): wDescriptorLength = IFACE1_HID_DECL
    9, USB_DESC_HID, 0x11, 0x01, 0, 1, USB_DESC_HID_REPORT,
        (IFACE1_HID_DECL & 0xFF), (IFACE1_HID_DECL >> 8),
    // iface 1 interrupt IN EP (7)
    7, USB_DESC_ENDPOINT, 0x82, 0x03, 8, 0, 10,
};

// Set by the short-transfer regression scenario in main(): the config
// GET_DESCRIPTOR then advertises a wTotalLength <= cap (so config_truncated is
// false) but returns fewer bytes than requested, exercising the short-transfer
// patch branch that mirrors the BOS handling.
#define SHORT_CFG_ADVERTISED  59u   // <= MAX_CONFIG_DESC_SIZE
#define SHORT_CFG_RETURNED    40u   // device actually returns fewer bytes
static int g_short_cfg = 0;

// Cap-independent copy of MAX_CONFIG_DESC_SIZE for the mock's clamp (mirrors what
// the caller requests). We read the real macro so the mock tracks the header.
static int mock_control_transfer(uint8_t addr, uint8_t maxpkt,
    const usb_setup_t *s, uint8_t *data, uint32_t timeout_ms);

// ── usb_host stubs the capture path calls (single TU; defined before the
//    #include of the code under test) ───────────────────────────────────────
int usb_host_control_transfer(uint8_t addr, uint8_t maxpkt,
    const usb_setup_t *setup, uint8_t *data, uint32_t timeout_ms)
{
    return mock_control_transfer(addr, maxpkt, setup, data, timeout_ms);
}
void     delay(uint32_t msec)               { (void)msec; }
bool     usb_host_device_connected(void)    { return true; }
uint8_t  usb_host_device_speed(void)        { return USB_SPEED_HIGH; }
void     usb_host_power_on(void)            { }
void     usb_host_port_reset(void)          { }

// Pull in the real code under test (capture_descriptors, parse_config_descriptor,
// and — after the fix is ported — patch_hid_report_desc_len).
#include "desc_capture.c"

// ── Mock implementation ──────────────────────────────────────────────────────
static int mock_control_transfer(uint8_t addr, uint8_t maxpkt,
    const usb_setup_t *s, uint8_t *data, uint32_t timeout_ms)
{
    (void)addr; (void)maxpkt; (void)timeout_ms;
    const uint8_t  type  = (uint8_t)(s->wValue >> 8);
    const uint8_t  index = (uint8_t)(s->wValue & 0xFF);
    const uint16_t wlen  = s->wLength;

    // Class request: HID SET_IDLE (0x21 / 0x0A). Also SET_ADDRESS / SET_CONFIG.
    if (s->bmRequestType == 0x21) return 0;                 // SET_IDLE ok
    if (s->bRequest == USB_REQ_SET_ADDRESS) return 0;
    if (s->bRequest == USB_REQ_SET_CONFIG)  return 0;

    if (s->bRequest != USB_REQ_GET_DESCRIPTOR) return -1;

    switch (type) {
    case USB_DESC_DEVICE: {
        uint8_t dd[18] = {0};
        dd[0] = 18; dd[1] = USB_DESC_DEVICE; dd[7] = 64;    // bMaxPacketSize0
        dd[14] = 2;  // iManufacturer -> malformed (too short), must be rejected
        dd[15] = 1;  // iProduct      -> valid string, stored
        dd[16] = 3;  // iSerialNumber -> malformed (wrong type), must be rejected
        uint16_t n = (wlen < 18) ? 8 : 18;
        memcpy(data, dd, n);
        return (int)n;
    }
    case USB_DESC_CONFIGURATION: {
        if (wlen <= 9) {                                     // header probe
            memcpy(data, k_config_prefix, 9);
            if (g_short_cfg) {                               // advertise <= cap
                data[2] = (SHORT_CFG_ADVERTISED & 0xFF);
                data[3] = (SHORT_CFG_ADVERTISED >> 8);
            }
            return 9;
        }
        if (g_short_cfg) {
            // Not truncated (advertised <= cap) but a short transfer: the body
            // still claims SHORT_CFG_ADVERTISED yet only SHORT_CFG_RETURNED bytes
            // arrive, so capture must patch wTotalLength down to what we stored.
            memset(data, 0, wlen);
            memcpy(data, k_config_prefix, SHORT_CFG_RETURNED);
            data[2] = (SHORT_CFG_ADVERTISED & 0xFF);
            data[3] = (SHORT_CFG_ADVERTISED >> 8);
            return (int)SHORT_CFG_RETURNED;
        }
        // Full fetch: caller clamps wlen to MAX_CONFIG_DESC_SIZE. Return exactly
        // that many bytes (prefix + zero pad) to simulate a device larger than cap.
        uint16_t n = wlen;
        memset(data, 0, n);
        memcpy(data, k_config_prefix, sizeof k_config_prefix);
        return (int)n;
    }
    case USB_DESC_STRING: {
        if (index == 0) {                                    // LANGID table
            data[0] = 4; data[1] = USB_DESC_STRING; data[2] = 0x09; data[3] = 0x04;
            return 4;
        }
        if (index == 0xEE) return -1;                        // no MS OS 1.0
        if (index == 1) {                                    // iProduct string
            memset(data, 0, wlen);
            data[0] = STR1_DECL_BLEN;                        // lies: claims 40
            data[1] = USB_DESC_STRING;
            for (uint8_t i = 2; i < STR1_ACTUAL_LEN; i++) data[i] = (uint8_t)('A' + i);
            return (int)STR1_ACTUAL_LEN;                     // actually returns 12
        }
        if (index == 2) {                                    // iManufacturer: too short
            data[0] = 1;                                     // 1-byte reply, no type byte
            return 1;                                        // ret < 2 -> must be rejected
        }
        if (index == 3) {                                    // iSerial: wrong descriptor type
            memset(data, 0, wlen);
            data[0] = 8; data[1] = 0x22;                     // bDescriptorType != STRING(3)
            return 8;                                        // wrong type -> must be rejected
        }
        return -1;
    }
    case USB_DESC_BOS: {
        if (wlen == 5) {                                     // header probe
            data[0] = 5; data[1] = USB_DESC_BOS;
            data[2] = (DEV_BOS_TOTAL & 0xFF); data[3] = (DEV_BOS_TOTAL >> 8);
            data[4] = 1;
            return 5;
        }
        uint16_t n = wlen;                                   // clamped to cap by caller
        memset(data, 0, n);
        data[0] = 5; data[1] = USB_DESC_BOS;
        data[2] = (DEV_BOS_TOTAL & 0xFF); data[3] = (DEV_BOS_TOTAL >> 8);
        data[4] = 1;
        return (int)n;
    }
    case USB_DESC_HID_REPORT: {                              // wIndex = iface_num
        if (s->wIndex == 0) {                                // iface0: succeeds (300)
            memset(data, 0xAB, IFACE0_HID_ACTUAL);
            return (int)IFACE0_HID_ACTUAL;
        }
        return -1;                                           // iface1: fetch fails
    }
    default:
        return -1;
    }
}

// ── Isolated unit test for patch_hid_report_desc_len() ───────────────────────
// Exercises the byte-patcher directly (no USB path) on a hand-built config blob,
// including the "wrong interface -> no change" and malformed-descriptor cases.
static void test_patch_helper_isolated(void)
{
    captured_descriptors_t d;
    memset(&d, 0, sizeof d);

    // config header(9) + interface #3 (9) + HID desc(9) declaring 0xBEEF.
    static const uint8_t blob[27] = {
        9, USB_DESC_CONFIGURATION, 27, 0, 1, 1, 0, 0x80, 50,
        9, USB_DESC_INTERFACE, 3, 0, 0, 3, 1, 2, 0,
        9, USB_DESC_HID, 0x11, 0x01, 0, 1, USB_DESC_HID_REPORT, 0xEF, 0xBE,
    };
    memcpy(d.config_desc, blob, sizeof blob);
    d.config_desc_len = sizeof blob;
    // wDescriptorLength lives at config offset 18(HID desc)+7 = 25,26.
    CHECK((d.config_desc[25] | (d.config_desc[26] << 8)) == 0xBEEF);

    // Matching interface -> patched to the actual length.
    patch_hid_report_desc_len(&d, 3, 0x0123);
    CHECK(d.config_desc[25] == 0x23 && d.config_desc[26] == 0x01);

    // Non-existent interface -> left unchanged.
    patch_hid_report_desc_len(&d, 7, 0x4444);
    CHECK((d.config_desc[25] | (d.config_desc[26] << 8)) == 0x0123);

    // Malformed trailing descriptor (bLength = 0) must not loop/overrun: shrink
    // config_desc_len so the walk hits the zero-length guard and bails.
    d.config_desc[9] = 9; d.config_desc_len = 27;
    d.config_desc[18] = 0;                 // corrupt the HID desc bLength
    patch_hid_report_desc_len(&d, 3, 0x5678);   // must return without touching OOB
    // iface #3's HID entry is now unreachable, so the value stays as last set.
    CHECK((d.config_desc[25] | (d.config_desc[26] << 8)) == 0x0123);
}

int main(void)
{
    test_patch_helper_isolated();

    captured_descriptors_t d;
    bool ok = capture_descriptors(&d);
    CHECK(ok);

    // ── config wTotalLength patched to the actually-stored length ────────────
    uint16_t cfg_wtotal = (uint16_t)(d.config_desc[2] | (d.config_desc[3] << 8));
    CHECK(d.config_desc_len == MAX_CONFIG_DESC_SIZE);   // truncated to cap
    CHECK(cfg_wtotal == d.config_desc_len);             // <-- the core assertion
    CHECK(cfg_wtotal != DEV_CONFIG_TOTAL);              // no longer the device's lie

    // ── interfaces ────────────────────────────────────────────────────────────
    CHECK(d.num_ifaces == 2);
    // iface0: report fetch succeeded, truncated to actual; config HID
    // wDescriptorLength patched to match the captured length.
    CHECK(d.ifaces[0].has_hid_desc == true);
    CHECK(d.ifaces[0].hid_report_desc_len == IFACE0_HID_ACTUAL);
    uint16_t hid0_wdesc = (uint16_t)(d.config_desc[25] | (d.config_desc[26] << 8));
    CHECK(hid0_wdesc == d.ifaces[0].hid_report_desc_len);
    CHECK(hid0_wdesc != IFACE0_HID_DECL);

    // iface1: report fetch failed -> don't replay an invalid descriptor.
    CHECK(d.ifaces[1].has_hid_desc == false);
    CHECK(d.ifaces[1].hid_report_desc_len == 0);

    // ── BOS wTotalLength patched to the stored length ────────────────────────
    uint16_t bos_wtotal = (uint16_t)(d.bos_desc[2] | (d.bos_desc[3] << 8));
    CHECK(d.bos_desc_len == MAX_BOS_DESC_SIZE);
    CHECK(bos_wtotal == d.bos_desc_len);
    CHECK(bos_wtotal != DEV_BOS_TOTAL);

    // ── string bLength clamped; malformed strings rejected ───────────────────
    // iManufacturer(2) returns 1 byte and iSerial(3) returns a wrong-type
    // descriptor; both must be rejected so only the valid iProduct(1) is stored.
    // (Pre-fix, `if (ret > 0)` stored all three -> num_strings would be 3.)
    CHECK(d.num_strings == 1);
    CHECK(d.string_index[0] == 1);
    CHECK(d.string_desc_len[0] == STR1_ACTUAL_LEN);
    CHECK(d.string_desc[0][0] == STR1_ACTUAL_LEN);      // was STR1_DECL_BLEN (40)

    // ── config short-transfer (not truncated): wTotalLength patched down ──────
    // A device advertising <= cap but returning fewer bytes must have its
    // replayed config wTotalLength patched to the stored length (mirrors BOS).
    // Pre-fix (patch only on config_truncated), config_desc[2..3] stays at the
    // advertised 59 while only 40 bytes were stored -> the clone host over-reads.
    g_short_cfg = 1;
    captured_descriptors_t d2;
    CHECK(capture_descriptors(&d2));
    CHECK(d2.config_desc_len == SHORT_CFG_RETURNED);
    uint16_t cfg2 = (uint16_t)(d2.config_desc[2] | (d2.config_desc[3] << 8));
    CHECK(cfg2 == d2.config_desc_len);                  // patched to 40, not left at 59
    g_short_cfg = 0;

    printf("desc_capture_test: all assertions passed\n");
    return 0;
}
