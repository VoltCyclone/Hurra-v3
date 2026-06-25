// desc_serialize.h — compact, versioned serialization of captured_descriptors_t.
// Replaces the whole-struct memcpy blob on the SPI link with a length-prefixed
// form that carries only the bytes a real device actually populated. Pure,
// MMIO-free, host-tested (test/desc_serialize_test.c, in `make test`).
//
// The transport (desc_xfer) and framing (spi_frame) are unchanged — they already
// carry an arbitrary-length byte blob; this only changes *what bytes* we hand them.
#ifndef DESC_SERIALIZE_H
#define DESC_SERIALIZE_H

#include <stdint.h>
#include <stdbool.h>
#include "desc_capture.h"

#define DESC_WIRE_MAGIC0   0x44u   // 'D'
#define DESC_WIRE_MAGIC1   0x53u   // 'S'
#define DESC_WIRE_VERSION  0x01u

// Worst-case serialized size is bounded by the struct size (every field at its
// fixed-array max plus length prefixes still sums to less than the padded struct),
// so a buffer sized to the struct is always large enough.
#define DESC_WIRE_MAX_LEN  ((uint16_t)sizeof(captured_descriptors_t))

// Serialize *in into out[0..out_cap-1]. Returns bytes written, or 0 on failure
// (NULL args, or out_cap too small at any step — fail closed, never truncate).
uint16_t desc_serialize(const captured_descriptors_t *in,
                        uint8_t *out, uint16_t out_cap);

// Deserialize in[0..len-1] back into *out. Zeroes *out first, then fills present
// fields. Returns true on success; false (and *out left zeroed, valid=false) on
// any malformed/truncated/over-long input. Every copy is bounded against the
// fixed-array max so a corrupt length can never overflow *out.
bool desc_deserialize(const uint8_t *in, uint16_t len,
                     captured_descriptors_t *out);

#endif // DESC_SERIALIZE_H
