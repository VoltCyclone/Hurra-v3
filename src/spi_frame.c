// spi_frame.c — pure codec for the board-to-board SPI hot-path slot. See
// spi_frame.h. Host-testable: stdint only, no MMIO/WCH headers.
#include "spi_frame.h"

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final XOR.
// Matches the CH32 SPI hardware CRC so the driver can cross-check the soft codec.
// Bitwise (no table): the slot is 30 bytes, so size beats speed here.
uint16_t spi_frame_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000u) crc = (uint16_t)((crc << 1) ^ 0x1021u);
            else               crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

spi_frame_result_t spi_frame_pack(uint8_t slot[SPI_FRAME_SLOT_SIZE],
                                  uint8_t type, uint8_t seq,
                                  const uint8_t *payload, uint8_t len)
{
    if (len > SPI_FRAME_MAX_PAYLOAD) return SPI_FRAME_ERR_LEN;

    slot[SPI_FRAME_OFF_SOF]  = SPI_FRAME_SOF;
    slot[SPI_FRAME_OFF_TYPE] = type;
    slot[SPI_FRAME_OFF_SEQ]  = seq;
    slot[SPI_FRAME_OFF_LEN]  = len;

    for (uint8_t i = 0; i < len; i++)
        slot[SPI_FRAME_OFF_PAY + i] = payload[i];
    // Zero-fill the unused payload tail so the slot is deterministic and the CRC
    // covers a known pattern.
    for (uint8_t i = len; i < SPI_FRAME_MAX_PAYLOAD; i++)
        slot[SPI_FRAME_OFF_PAY + i] = 0x00u;

    uint16_t crc = spi_frame_crc16(slot, SPI_FRAME_OFF_CRC);
    slot[SPI_FRAME_OFF_CRC]     = (uint8_t)(crc & 0xFFu);
    slot[SPI_FRAME_OFF_CRC + 1] = (uint8_t)(crc >> 8);
    return SPI_FRAME_OK;
}

spi_frame_result_t spi_frame_unpack(const uint8_t slot[SPI_FRAME_SLOT_SIZE],
                                    uint8_t *type, uint8_t *seq,
                                    const uint8_t **payload, uint8_t *len)
{
    if (slot[SPI_FRAME_OFF_SOF] != SPI_FRAME_SOF) return SPI_FRAME_ERR_SOF;
    if (slot[SPI_FRAME_OFF_LEN] > SPI_FRAME_MAX_PAYLOAD) return SPI_FRAME_ERR_LEN;

    uint16_t want = (uint16_t)(slot[SPI_FRAME_OFF_CRC] |
                               ((uint16_t)slot[SPI_FRAME_OFF_CRC + 1] << 8));
    if (spi_frame_crc16(slot, SPI_FRAME_OFF_CRC) != want) return SPI_FRAME_ERR_CRC;

    if (type)    *type    = slot[SPI_FRAME_OFF_TYPE];
    if (seq)     *seq     = slot[SPI_FRAME_OFF_SEQ];
    if (len)     *len     = slot[SPI_FRAME_OFF_LEN];
    if (payload) *payload = &slot[SPI_FRAME_OFF_PAY];
    return SPI_FRAME_OK;
}

uint8_t spi_frame_seq_gap(uint8_t prev_seq, uint8_t seq)
{
    // 8-bit wrapping distance minus the one expected step. A repeat (seq ==
    // prev_seq) and the immediate successor both yield 0 lost frames.
    uint8_t delta = (uint8_t)(seq - prev_seq);   // wraps mod 256
    return delta ? (uint8_t)(delta - 1u) : 0u;
}
