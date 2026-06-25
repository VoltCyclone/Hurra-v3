// Host unit test for the descriptor serializer (src/desc_serialize.c). Pure, no
// MMIO — built/run via `make test`. Verifies the compact wire form round-trips a
// populated captured_descriptors_t byte-for-byte and fails closed on bad input.
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "desc_serialize.h"

#define CHECK(cond) do { if (!(cond)) { \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); assert(0); } } while (0)

// Fill a representative populated struct: composite keyboard+mouse, a few strings,
// HID report descriptors, langid. Tails left zeroed (memset first) so a whole-struct
// memcmp after round-trip is meaningful.
static void build_populated(captured_descriptors_t *d)
{
    memset(d, 0, sizeof *d);

    d->device_desc_len = 18;
    for (uint8_t i = 0; i < 18; i++) d->device_desc[i] = (uint8_t)(0x10u + i);
    d->device_desc[8]  = 0x6Bu; d->device_desc[9]  = 0x1Du; // VID 0x1D6B
    d->device_desc[10] = 0x04u; d->device_desc[11] = 0x00u; // PID 0x0004

    d->config_desc_len = 59;
    for (uint16_t i = 0; i < 59; i++) d->config_desc[i] = (uint8_t)(i * 3u + 1u);
    d->config_string_idx = 4;

    d->num_ifaces = 2;
    // iface 0: boot keyboard
    d->ifaces[0].iface_num = 0; d->ifaces[0].iface_class = 0x03;
    d->ifaces[0].iface_subclass = 0x01; d->ifaces[0].iface_protocol = 0x01;
    d->ifaces[0].iface_string_idx = 0;
    d->ifaces[0].interrupt_in_ep = 0x81; d->ifaces[0].interrupt_in_maxpkt = 8;
    d->ifaces[0].interrupt_in_interval = 10;
    d->ifaces[0].has_hid_desc = true;
    d->ifaces[0].hid_report_desc_len = 63;
    for (uint16_t i = 0; i < 63; i++) d->ifaces[0].hid_report_desc[i] = (uint8_t)(i + 5u);
    // iface 1: boot mouse
    d->ifaces[1].iface_num = 1; d->ifaces[1].iface_class = 0x03;
    d->ifaces[1].iface_subclass = 0x01; d->ifaces[1].iface_protocol = 0x02;
    d->ifaces[1].iface_string_idx = 0;
    d->ifaces[1].interrupt_in_ep = 0x82; d->ifaces[1].interrupt_in_maxpkt = 4;
    d->ifaces[1].interrupt_in_interval = 10;
    d->ifaces[1].has_hid_desc = true;
    d->ifaces[1].hid_report_desc_len = 52;
    for (uint16_t i = 0; i < 52; i++) d->ifaces[1].hid_report_desc[i] = (uint8_t)(i * 2u + 9u);

    d->num_strings = 3;
    const char *strs[3] = { "Acme Inc", "Acme Combo", "SN-12345" };
    uint8_t idx[3] = { 1, 2, 3 };
    for (uint8_t s = 0; s < 3; s++) {
        d->string_index[s] = idx[s];
        uint8_t n = (uint8_t)strlen(strs[s]);
        d->string_desc_len[s] = n;
        memcpy(d->string_desc[s], strs[s], n);
    }

    d->langid_desc_len = 4;
    d->langid_desc[0] = 0x04; d->langid_desc[1] = 0x03;
    d->langid_desc[2] = 0x09; d->langid_desc[3] = 0x04;
    d->langid = 0x0409;

    // bos / ms_os left empty
    d->ep0_maxpkt = 64;
    d->dev_addr   = 5;
    d->speed      = 2;
    d->valid      = true;
}

int main(void)
{
    // 1. Round-trip equality (the headline test).
    {
        captured_descriptors_t src;
        build_populated(&src);

        uint8_t wire[DESC_WIRE_MAX_LEN];
        uint16_t n = desc_serialize(&src, wire, sizeof wire);
        CHECK(n > 0);
        CHECK(n < (uint16_t)sizeof(captured_descriptors_t));  // it actually shrank

        captured_descriptors_t dst;
        CHECK(desc_deserialize(wire, n, &dst));
        CHECK(memcmp(&src, &dst, sizeof src) == 0);           // byte-for-byte
        printf("  round-trip: %u bytes (struct is %u) -> %.1fx smaller\n",
               (unsigned)n, (unsigned)sizeof(captured_descriptors_t),
               (double)sizeof(captured_descriptors_t) / (double)n);
    }

    // 2. Empty/minimal struct round-trips, valid=false preserved.
    {
        captured_descriptors_t src;
        memset(&src, 0, sizeof src);
        src.valid = false;

        uint8_t wire[DESC_WIRE_MAX_LEN];
        uint16_t n = desc_serialize(&src, wire, sizeof wire);
        CHECK(n > 0);

        captured_descriptors_t dst;
        CHECK(desc_deserialize(wire, n, &dst));
        CHECK(memcmp(&src, &dst, sizeof src) == 0);
        CHECK(dst.valid == false);
    }

    // 3. Truncation rejection — every short prefix must fail closed.
    {
        captured_descriptors_t src;
        build_populated(&src);
        uint8_t wire[DESC_WIRE_MAX_LEN];
        uint16_t n = desc_serialize(&src, wire, sizeof wire);
        CHECK(n > 0);
        for (uint16_t k = 0; k < n; k++) {
            captured_descriptors_t dst;
            CHECK(desc_deserialize(wire, k, &dst) == false);
            CHECK(dst.valid == false);
        }
        // The exact length must still succeed.
        captured_descriptors_t ok;
        CHECK(desc_deserialize(wire, n, &ok));
    }

    // 4. Bad magic / version.
    {
        captured_descriptors_t src;
        build_populated(&src);
        uint8_t wire[DESC_WIRE_MAX_LEN];
        uint16_t n = desc_serialize(&src, wire, sizeof wire);
        captured_descriptors_t dst;

        uint8_t save = wire[0]; wire[0] ^= 0xFFu;
        CHECK(desc_deserialize(wire, n, &dst) == false);
        wire[0] = save;

        save = wire[1]; wire[1] ^= 0xFFu;
        CHECK(desc_deserialize(wire, n, &dst) == false);
        wire[1] = save;

        save = wire[2]; wire[2] = 0x02u;  // unknown version
        CHECK(desc_deserialize(wire, n, &dst) == false);
        wire[2] = save;

        // restored buffer parses again
        CHECK(desc_deserialize(wire, n, &dst));
    }

    // 5. Overflow rejection — a CONFIG length claiming more than the array max.
    //    Hand-craft a minimal valid header, then a CONFIG length over the cap.
    {
        // header(12) + device(1 len=0) + config(2 len) ...
        uint8_t wire[64];
        uint16_t o = 0;
        wire[o++] = DESC_WIRE_MAGIC0;
        wire[o++] = DESC_WIRE_MAGIC1;
        wire[o++] = DESC_WIRE_VERSION;
        wire[o++] = 0x01;             // flags valid
        wire[o++] = 64;               // ep0_maxpkt
        wire[o++] = 1;                // dev_addr
        wire[o++] = 2;                // speed
        wire[o++] = 0;                // config_string_idx
        wire[o++] = 0x09; wire[o++] = 0x04;  // langid
        wire[o++] = 0;                // ms_os_vendor_code
        wire[o++] = 0;                // num_ifaces
        wire[o++] = 0;                // DEVICE len=0
        // CONFIG len = MAX_CONFIG_DESC_SIZE + 1 (over cap)
        uint16_t bad = (uint16_t)(MAX_CONFIG_DESC_SIZE + 1);
        wire[o++] = (uint8_t)bad; wire[o++] = (uint8_t)(bad >> 8);
        // (no actual config bytes follow; r_blob must reject on cap before reading)

        captured_descriptors_t dst;
        CHECK(desc_deserialize(wire, o, &dst) == false);
        CHECK(dst.valid == false);
    }

    // 6. num_ifaces over MAX_INTERFACES rejected.
    {
        uint8_t wire[16];
        uint16_t o = 0;
        wire[o++] = DESC_WIRE_MAGIC0;
        wire[o++] = DESC_WIRE_MAGIC1;
        wire[o++] = DESC_WIRE_VERSION;
        wire[o++] = 0x01;
        wire[o++] = 64; wire[o++] = 1; wire[o++] = 2; wire[o++] = 0;
        wire[o++] = 0x09; wire[o++] = 0x04;
        wire[o++] = 0;
        wire[o++] = (uint8_t)(MAX_INTERFACES + 1);  // num_ifaces over max

        captured_descriptors_t dst;
        CHECK(desc_deserialize(wire, o, &dst) == false);
    }

    // 7. Serializer capacity guard — tiny buffer returns 0, no OOB write (ASan).
    {
        captured_descriptors_t src;
        build_populated(&src);
        uint8_t tiny[4];
        CHECK(desc_serialize(&src, tiny, sizeof tiny) == 0);
        // even a header-sized-minus-one buffer fails closed
        uint8_t hdr_minus[11];
        CHECK(desc_serialize(&src, hdr_minus, sizeof hdr_minus) == 0);
    }

    // 8. NULL args.
    {
        captured_descriptors_t src; build_populated(&src);
        uint8_t wire[DESC_WIRE_MAX_LEN];
        CHECK(desc_serialize(NULL, wire, sizeof wire) == 0);
        CHECK(desc_serialize(&src, NULL, sizeof wire) == 0);
        captured_descriptors_t dst;
        CHECK(desc_deserialize(NULL, 10, &dst) == false);
        CHECK(desc_deserialize(wire, 10, NULL) == false);
    }

    printf("desc_serialize: all passed\n");
    return 0;
}
