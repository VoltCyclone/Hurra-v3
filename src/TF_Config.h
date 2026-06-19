// src/TF_Config.h — TinyFrame configuration for the Hurra protocol.
// See docs/specs/2026-05-23-hurra-binary-protocol-design.md §2.
#pragma once

#define TF_SOF_BYTE       0x68    // 'h' for Hurra
#define TF_ID_BYTES       1
#define TF_LEN_BYTES      1
#define TF_TYPE_BYTES     1
#define TF_CKSUM_TYPE     TF_CKSUM_CRC16
#define TF_USE_MUTEX      0
#define TF_MAX_PAYLOAD_RX 256
#define TF_SENDBUF_LEN    264     // max payload + 8 byte header overhead
#define TF_PARSER_TIMEOUT_TICKS 5  // 5 × hurra_tick period (1 ms) = 5 ms idle gap

// Listener storage. Spec §3 defines ~45 TYPE listeners; 48 leaves headroom.
// ID listeners cover transient deferred replies. TinyFrame requires >=1 generic.
#include <stdint.h>
typedef uint32_t TF_TICKS;
typedef uint8_t  TF_COUNT;
#define TF_MAX_ID_LST    4
#define TF_MAX_TYPE_LST  48
#define TF_MAX_GEN_LST   1

// Error logging stub: no console in this firmware; TF errors surface via the
// stats counters instead.
#define TF_Error(...) ((void)0)
