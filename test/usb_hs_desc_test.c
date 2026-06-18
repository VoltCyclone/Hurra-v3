// Host unit test for the USB High-Speed descriptor helper (src/usb_hs_desc.c).
// Pure function, no MMIO — built/run via `make test`.
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "usb_hs_desc.h"

#define CHECK(cond) do { if (!(cond)) { \
    printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); assert(0); } } while (0)

int main(void)
{
    /* ---- usb_hs_synth_qualifier: build DEVICE_QUALIFIER from a device desc ---- */

    /* A minimal 18-byte device descriptor (boot mouse): class/sub/proto at [4..6],
     * bMaxPacketSize0 at [7], bNumConfigurations at [17]. */
    uint8_t dev[18] = {0};
    dev[0]  = 18;       /* bLength            */
    dev[1]  = 0x01;     /* DEVICE             */
    dev[2]  = 0x00; dev[3] = 0x02;  /* bcdUSB = 0x0200 */
    dev[4]  = 0x00;     /* bDeviceClass (per-interface) */
    dev[5]  = 0x00;     /* bDeviceSubClass    */
    dev[6]  = 0x00;     /* bDeviceProtocol    */
    dev[7]  = 64;       /* bMaxPacketSize0    */
    dev[17] = 1;        /* bNumConfigurations */

    uint8_t q[10] = {0};
    CHECK(usb_hs_synth_qualifier(dev, q) == 10);
    CHECK(q[0] == 10);          /* bLength = 10                      */
    CHECK(q[1] == 0x06);        /* bDescriptorType = DEVICE_QUALIFIER */
    CHECK(q[2] == 0x00 && q[3] == 0x02);  /* bcdUSB copied (0x0200)  */
    CHECK(q[4] == dev[4]);      /* bDeviceClass                     */
    CHECK(q[5] == dev[5]);      /* bDeviceSubClass                  */
    CHECK(q[6] == dev[6]);      /* bDeviceProtocol                  */
    CHECK(q[7] == dev[7]);      /* bMaxPacketSize0                  */
    CHECK(q[8] == dev[17]);     /* bNumConfigurations               */
    CHECK(q[9] == 0x00);        /* bReserved = 0                    */

    printf("usb_hs_desc_test: all passed\n");
    return 0;
}
