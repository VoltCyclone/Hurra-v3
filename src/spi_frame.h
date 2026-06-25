// spi_frame.h — pure (host-testable) codec for the board-to-board SPI link's fixed
// hot-path slot. No MMIO, no WCH headers; stdint only. The SPI driver that uses this
// codec lives in spi_link.c. This file is just the wire format.
//
// Hot-path slot (fixed 32 bytes), board B (host) -> board A (device) and the
// full-duplex return slot the other way:
//
//   off  field   size  notes
//   0    SOF     1     0x68 (reuse the Hurra SOF; resync anchor)
//   1    TYPE    1     frame tag (reuse ICC record tags; high bit = control class)
//   2    SEQ     1     monotonic per direction; wrap is harmless
//   3    LEN     1     payload bytes in use (0..SPI_FRAME_MAX_PAYLOAD)
//   4..  PAYLOAD var   LEN bytes; the rest of the region is zero-filled on pack
//   30   CRC16   2     CRC-16/CCITT-FALSE over bytes [0 .. SLOT_SIZE-3], LE
//
// CRC covers the entire slot except its own 2 bytes (including the zero-filled
// tail), so a glitch anywhere in the slot is caught and the format is
// deterministic for a given (type, seq, payload).
#ifndef SPI_FRAME_H
#define SPI_FRAME_H

#include <stdint.h>

#define SPI_FRAME_SOF          0x68u
#define SPI_FRAME_SLOT_SIZE    32u
#define SPI_FRAME_HDR_SIZE     4u   // SOF, TYPE, SEQ, LEN
#define SPI_FRAME_CRC_SIZE     2u
#define SPI_FRAME_MAX_PAYLOAD  (SPI_FRAME_SLOT_SIZE - SPI_FRAME_HDR_SIZE - SPI_FRAME_CRC_SIZE) // 26

// Byte offsets within the slot.
#define SPI_FRAME_OFF_SOF   0u
#define SPI_FRAME_OFF_TYPE  1u
#define SPI_FRAME_OFF_SEQ   2u
#define SPI_FRAME_OFF_LEN   3u
#define SPI_FRAME_OFF_PAY   4u
#define SPI_FRAME_OFF_CRC   (SPI_FRAME_SLOT_SIZE - SPI_FRAME_CRC_SIZE) // 30

typedef enum {
    SPI_FRAME_OK = 0,
    SPI_FRAME_ERR_SOF,   // first byte wasn't SPI_FRAME_SOF
    SPI_FRAME_ERR_LEN,   // LEN field exceeds SPI_FRAME_MAX_PAYLOAD
    SPI_FRAME_ERR_CRC,   // CRC mismatch
} spi_frame_result_t;

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection). Exposed so the
// SPI driver can cross-check the hardware CRC engine against the soft codec.
uint16_t spi_frame_crc16(const uint8_t *data, uint32_t len);

// Pack a frame into a 32-byte slot. Zero-fills the unused payload tail and writes
// the trailing CRC16 (little-endian). Returns SPI_FRAME_OK, or SPI_FRAME_ERR_LEN
// if len > SPI_FRAME_MAX_PAYLOAD (slot left untouched).
spi_frame_result_t spi_frame_pack(uint8_t slot[SPI_FRAME_SLOT_SIZE],
                                  uint8_t type, uint8_t seq,
                                  const uint8_t *payload, uint8_t len);

// Validate + unpack a received slot. Checks SOF, LEN bound, and CRC in that
// order. On SPI_FRAME_OK, writes *type/*seq/*len and points *payload at the
// payload bytes INSIDE slot (no copy). Out-params may be NULL if unwanted.
// On any error the out-params are left unspecified.
spi_frame_result_t spi_frame_unpack(const uint8_t slot[SPI_FRAME_SLOT_SIZE],
                                    uint8_t *type, uint8_t *seq,
                                    const uint8_t **payload, uint8_t *len);

// Frames dropped between prev_seq and seq, accounting for 8-bit wrap. 0 when seq
// is the immediate successor of prev_seq; 0 also for a repeat (seq == prev_seq).
uint8_t spi_frame_seq_gap(uint8_t prev_seq, uint8_t seq);

#endif // SPI_FRAME_H
