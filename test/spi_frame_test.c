// Host unit test for the board-to-board SPI hot-path frame codec (src/spi_frame.c).
// Build/run via `make test`.
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "spi_frame.h"

#define CHECK(cond) do { if (!(cond)) { \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); assert(0); } } while (0)

int main(void)
{
    // 1. CRC-16/CCITT-FALSE known-answer vector: "123456789" -> 0x29B1.
    {
        const uint8_t v[] = { '1','2','3','4','5','6','7','8','9' };
        CHECK(spi_frame_crc16(v, sizeof v) == 0x29B1u);
    }

    // 2. Empty input -> CRC equals the init value 0xFFFF.
    CHECK(spi_frame_crc16((const uint8_t *)"", 0) == 0xFFFFu);

    // 3. Round-trip: pack then unpack recovers type, seq, len, and payload bytes.
    {
        uint8_t slot[SPI_FRAME_SLOT_SIZE];
        const uint8_t pay[] = { 0x02, 0x01, 0xFF, 0x00, 0x7A }; // dev_ep, proto, report...
        CHECK(spi_frame_pack(slot, 0x21 /*type*/, 0x05 /*seq*/, pay, sizeof pay) == SPI_FRAME_OK);

        // Header laid out as specified.
        CHECK(slot[SPI_FRAME_OFF_SOF]  == SPI_FRAME_SOF);
        CHECK(slot[SPI_FRAME_OFF_TYPE] == 0x21);
        CHECK(slot[SPI_FRAME_OFF_SEQ]  == 0x05);
        CHECK(slot[SPI_FRAME_OFF_LEN]  == sizeof pay);

        uint8_t type, seq, len; const uint8_t *p;
        CHECK(spi_frame_unpack(slot, &type, &seq, &p, &len) == SPI_FRAME_OK);
        CHECK(type == 0x21);
        CHECK(seq  == 0x05);
        CHECK(len  == sizeof pay);
        CHECK(memcmp(p, pay, sizeof pay) == 0);
    }

    // 4. Unused payload tail is zero-filled (deterministic slot for a given input).
    {
        uint8_t slot[SPI_FRAME_SLOT_SIZE];
        memset(slot, 0xAA, sizeof slot); // pre-dirty
        const uint8_t pay[] = { 0x11, 0x22 };
        CHECK(spi_frame_pack(slot, 0x01, 0x00, pay, sizeof pay) == SPI_FRAME_OK);
        for (unsigned i = SPI_FRAME_OFF_PAY + sizeof pay; i < SPI_FRAME_OFF_CRC; i++)
            CHECK(slot[i] == 0x00);
    }

    // 5. CRC is little-endian at the slot tail and matches a recompute over [0..29].
    {
        uint8_t slot[SPI_FRAME_SLOT_SIZE];
        const uint8_t pay[] = { 0xDE, 0xAD };
        CHECK(spi_frame_pack(slot, 0x42, 0x09, pay, sizeof pay) == SPI_FRAME_OK);
        uint16_t crc = spi_frame_crc16(slot, SPI_FRAME_OFF_CRC);
        CHECK(slot[SPI_FRAME_OFF_CRC]     == (uint8_t)(crc & 0xFF));
        CHECK(slot[SPI_FRAME_OFF_CRC + 1] == (uint8_t)(crc >> 8));
    }

    // 6. A zero-length payload is valid.
    {
        uint8_t slot[SPI_FRAME_SLOT_SIZE];
        CHECK(spi_frame_pack(slot, 0x7F, 0x03, NULL, 0) == SPI_FRAME_OK);
        uint8_t type, seq, len; const uint8_t *p;
        CHECK(spi_frame_unpack(slot, &type, &seq, &p, &len) == SPI_FRAME_OK);
        CHECK(type == 0x7F && seq == 0x03 && len == 0);
    }

    // 7. Max-size payload is valid; one over is rejected at pack with ERR_LEN.
    {
        uint8_t slot[SPI_FRAME_SLOT_SIZE];
        uint8_t big[SPI_FRAME_MAX_PAYLOAD + 1];
        memset(big, 0x5A, sizeof big);
        CHECK(spi_frame_pack(slot, 1, 1, big, SPI_FRAME_MAX_PAYLOAD) == SPI_FRAME_OK);
        CHECK(spi_frame_pack(slot, 1, 1, big, SPI_FRAME_MAX_PAYLOAD + 1) == SPI_FRAME_ERR_LEN);
    }

    // 8. Unpack rejects a bad SOF.
    {
        uint8_t slot[SPI_FRAME_SLOT_SIZE];
        const uint8_t pay[] = { 0x01 };
        spi_frame_pack(slot, 0x10, 0x00, pay, sizeof pay);
        slot[SPI_FRAME_OFF_SOF] ^= 0xFF;
        CHECK(spi_frame_unpack(slot, NULL, NULL, NULL, NULL) == SPI_FRAME_ERR_SOF);
    }

    // 9. Unpack rejects a corrupt payload byte (CRC mismatch).
    {
        uint8_t slot[SPI_FRAME_SLOT_SIZE];
        const uint8_t pay[] = { 0x01, 0x02, 0x03 };
        spi_frame_pack(slot, 0x10, 0x00, pay, sizeof pay);
        slot[SPI_FRAME_OFF_PAY + 1] ^= 0x01; // flip one bit
        CHECK(spi_frame_unpack(slot, NULL, NULL, NULL, NULL) == SPI_FRAME_ERR_CRC);
    }

    // 10. Unpack rejects an out-of-range LEN (even if CRC were valid, LEN is checked).
    {
        uint8_t slot[SPI_FRAME_SLOT_SIZE];
        spi_frame_pack(slot, 0x10, 0x00, NULL, 0);
        slot[SPI_FRAME_OFF_LEN] = SPI_FRAME_MAX_PAYLOAD + 5;
        // recompute CRC so we isolate the LEN check from the CRC check
        uint16_t crc = spi_frame_crc16(slot, SPI_FRAME_OFF_CRC);
        slot[SPI_FRAME_OFF_CRC]     = (uint8_t)(crc & 0xFF);
        slot[SPI_FRAME_OFF_CRC + 1] = (uint8_t)(crc >> 8);
        CHECK(spi_frame_unpack(slot, NULL, NULL, NULL, NULL) == SPI_FRAME_ERR_LEN);
    }

    // 11. seq_gap: consecutive => 0, repeat => 0, one lost => 1, with 8-bit wrap.
    CHECK(spi_frame_seq_gap(10, 11) == 0); // immediate successor
    CHECK(spi_frame_seq_gap(10, 10) == 0); // repeat (retransmit / no advance)
    CHECK(spi_frame_seq_gap(10, 12) == 1); // one frame lost
    CHECK(spi_frame_seq_gap(10, 14) == 3); // three lost
    CHECK(spi_frame_seq_gap(255, 0)  == 0); // wrap: 255 -> 0 is consecutive
    CHECK(spi_frame_seq_gap(254, 1)  == 2); // wrap: 254 ->(255,0)-> 1 = 2 lost

	// INJECT frame: icc_record_t bytes survive pack/unpack with TYPE_INJECT.
	{
		uint8_t slot[SPI_FRAME_SLOT_SIZE];
		// tag=INJECT_MOUSE(1), dx=+5 LE, dy=-3 LE, buttons=0, wheel=0.
		const uint8_t pay[] = { 1, 5,0, 0xFD,0xFF, 0, 0 };
		CHECK(spi_frame_pack(slot, 0x06 /*TWO_BOARD_TYPE_INJECT*/, 0x11, pay, sizeof pay)
		      == SPI_FRAME_OK);
		uint8_t type, seq, len; const uint8_t *p;
		CHECK(spi_frame_unpack(slot, &type, &seq, &p, &len) == SPI_FRAME_OK);
		CHECK(type == 0x06);
		CHECK(len == sizeof pay);
		CHECK(p[0] == 1 && p[1] == 5 && p[3] == 0xFD);
	}

    printf("spi_frame_test: all passed\n");
    return 0;
}
