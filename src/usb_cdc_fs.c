// usb_cdc_fs.c — CDC-ACM virtual COM port on Board B's USBFS controller.
//
// Runs on the V5F core against the CH32H417 USBFS controller in Full-Speed
// (12 Mbps) DEVICE mode (PA11/PA12, USB-C port). Presents a USB CDC-ACM device
// (a virtual serial port the host opens with no driver install) to carry an
// in-band command/response channel. This is the SOLE USBFS_IRQHandler in the
// host image — usb_device.c/usb_device_fs.c/usb_device_hs.c are excluded from
// the host build (see the Makefile `ifeq ($(BOARD),host)` block), so there is no
// duplicate ISR.
//
// The EP0 standard-request enumeration state machine is ported from
// src/usb_device_fs.c, but the descriptors are STATIC (a fixed CDC-ACM device,
// not a cloned capture) and the endpoint set is fixed:
//   EP0      control, MPS 64
//   EP1 IN   interrupt notification (declared in the config descriptor as
//            CDC-ACM requires, but never armed — we send no notifications)
//   EP2 OUT  bulk, host->device command bytes -> rx ring
//   EP3 IN   bulk, device->host response bytes <- tx ring
//
// CDC class requests SET_LINE_CODING / GET_LINE_CODING / SET_CONTROL_LINE_STATE
// are ACKed (never STALLed) so host CDC drivers (Windows usbser, Linux cdc_acm,
// macOS AppleUSBCDCACM) bind cleanly; a dummy 7-byte line coding is stored and
// echoed back on GET_LINE_CODING.
//
// DMA-buffer convention (WCH FS, mirroring usb_device_fs.c): the UEPn_DMA
// registers hold the raw buffer address; CPU access uses the same address
// through a +0x20000000 alias (USBFSD_UEP_BUF). Buffers live in the dedicated
// .usbdma section. The shared USBHS 480 MHz PLL is brought up idempotently and
// never torn down (Board B's USBHS host capture depends on it; the USBFS 48 MHz
// clock is /10 of it). The V5F IRQ is routed before enabling.

#include "ch32h417_port.h"      // ch32h417.h (USBFSD struct, RCC), usb bit defs
#include "debug.h"
#include "timebase_v5f.h"
#include "usb_cdc_fs.h"
#include "usb_device_fs.h"      // usbfsd_* prototypes (stubbed below for host image)
#include <string.h>

// TIM9-based delay; the vendor Delay_Us spins on the shared SysTick0 ISR and can
// be raced to a hang by V3F (see usb_device_fs.c).
#define Delay_Us(us)  timebase_v5f_delay_us(us)

/* ------------------------------------------------------------------------ */
/* USBFS register helper macros (ported from usb_device_fs.c).              */
/* USBFSD_UEP_DMA_BASE is the address of UEP0_DMA; CTL base is UEP0_TX_CTRL. */
/* USBFSD_UEP_BUF(N) returns a CPU pointer (raw DMA addr + 0x20000000).      */
/* ------------------------------------------------------------------------ */
#define USBFSD_UEP_DMA_BASE         0x40023410
#define USBFSD_UEP_LEN_BASE         0x40023430
#define USBFSD_UEP_CTL_BASE         0x40023432

#define USBFSD_UEP_TX_CTRL(N)   (*((volatile uint8_t *)( USBFSD_UEP_CTL_BASE + (N) * 0x04 )))
#define USBFSD_UEP_RX_CTRL(N)   (*((volatile uint8_t *)( USBFSD_UEP_CTL_BASE + (N) * 0x04 + 1 )))
#define USBFSD_UEP_DMA(N)       (*((volatile uint32_t *)( USBFSD_UEP_DMA_BASE + (N) * 0x04 )))
#define USBFSD_UEP_BUF(N)       ((uint8_t *)(*((volatile uint32_t *)( USBFSD_UEP_DMA_BASE + (N) * 0x04 ))) + 0x20000000)
#define USBFSD_UEP_TLEN(N)      (*((volatile uint16_t *)( USBFSD_UEP_LEN_BASE + (N) * 0x04 )))

/* Endpoint index / direction constants (ported names). */
#define DEF_UEP_IN                  0x80
#define DEF_UEP0                    0x00

/* EP0 MPS is hardcoded 64 (full-speed control max; no cloned device). */
#define CDC_EP0_SIZE                64
/* Bulk packet size (full-speed bulk max). */
#define CDC_BULK_SIZE               64

/* Fixed CDC-ACM endpoint numbers. */
#define CDC_EP_NOTIFY               1   /* EP1 IN  — interrupt notification */
#define CDC_EP_OUT                  2   /* EP2 OUT — bulk command in        */
#define CDC_EP_IN                   3   /* EP3 IN  — bulk response out       */

/* Setup-request packet alias over the EP0 DMA buffer (CPU side). */
#define pCDC_SetupReqPak            ((PUSB_SETUP_REQ)CDC_EP0_Buf)

/* ------------------------------------------------------------------------ */
/* Static CDC-ACM descriptors. Adapted from the WCH SimulateCDC example     */
/* (Common/usb_desc.c) — IAD-free single-function layout: Communications    */
/* class device (bDeviceClass 0x02), CCI (interface 0) + DCI (interface 1). */
/* ------------------------------------------------------------------------ */
#define CDC_USB_VID  0x1A86
#define CDC_USB_PID  0xFE0C

static const uint8_t CDC_DevDescr[] = {
    0x12,                       /* bLength */
    0x01,                       /* bDescriptorType: Device */
    0x10, 0x01,                 /* bcdUSB 1.10 */
    0x02,                       /* bDeviceClass: CDC (Communications) */
    0x00,                       /* bDeviceSubClass */
    0x00,                       /* bDeviceProtocol */
    CDC_EP0_SIZE,               /* bMaxPacketSize0: 64 */
    (uint8_t)CDC_USB_VID, (uint8_t)(CDC_USB_VID >> 8),
    (uint8_t)CDC_USB_PID, (uint8_t)(CDC_USB_PID >> 8),
    0x00, 0x01,                 /* bcdDevice 1.00 */
    0x01,                       /* iManufacturer */
    0x02,                       /* iProduct */
    0x00,                       /* iSerialNumber */
    0x01,                       /* bNumConfigurations */
};

static const uint8_t CDC_CfgDescr[] = {
    /* Configuration descriptor (wTotalLength 0x43 = 67) */
    0x09, 0x02, 0x43, 0x00, 0x02, 0x01, 0x00, 0x80, 0x32,

    /* Interface 0 (CCI: Communications Class) */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,

    /* CDC functional descriptors */
    0x05, 0x24, 0x00, 0x10, 0x01,        /* Header */
    0x05, 0x24, 0x01, 0x00, 0x01,        /* Call Management (data iface 1) */
    0x04, 0x24, 0x02, 0x02,              /* Abstract Control Management */
    0x05, 0x24, 0x06, 0x00, 0x01,        /* Union (master 0, slave 1) */

    /* EP1 IN — interrupt notification */
    0x07, 0x05, 0x81, 0x03, (uint8_t)CDC_BULK_SIZE, (uint8_t)(CDC_BULK_SIZE >> 8), 0x01,

    /* Interface 1 (DCI: Data Class) */
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0A, 0x00, 0x00, 0x00,

    /* EP2 OUT — bulk */
    0x07, 0x05, 0x02, 0x02, (uint8_t)CDC_BULK_SIZE, (uint8_t)(CDC_BULK_SIZE >> 8), 0x00,

    /* EP3 IN — bulk */
    0x07, 0x05, 0x83, 0x02, (uint8_t)CDC_BULK_SIZE, (uint8_t)(CDC_BULK_SIZE >> 8), 0x00,
};

/* String descriptors. */
static const uint8_t CDC_LangDescr[] = { 0x04, 0x03, 0x09, 0x04 };
static const uint8_t CDC_ManuDescr[] = {
    0x0E, 0x03, 'w', 0, 'c', 0, 'h', 0, '.', 0, 'c', 0, 'n', 0
};
static const uint8_t CDC_ProdDescr[] = {
    0x14, 0x03, 'H', 0, 'u', 0, 'r', 0, 'r', 0, 'a', 0, ' ', 0, 'C', 0, 'D', 0, 'C', 0
};

/* ------------------------------------------------------------------------ */
/* DMA buffers — placed in the dedicated .usbdma section (uncached SRAM).   */
/* One 64-byte buffer per endpoint slot (EP0..EP3). 4*64 = 256 B.           */
/* ------------------------------------------------------------------------ */
#define CDC_NUM_EP_BUFS  4
__attribute__((section(".usbdma"), aligned(4)))
static uint8_t CDC_EP_Buf[CDC_NUM_EP_BUFS][64];
#define CDC_EP0_Buf  (CDC_EP_Buf[0])

/* ------------------------------------------------------------------------ */
/* Enumeration / setup state (ported from usb_device_fs.c).                 */
/* ------------------------------------------------------------------------ */
static const uint8_t *pCDC_Descr;            /* current descriptor walk ptr */

static volatile uint8_t  CDC_SetupReqCode;
static volatile uint8_t  CDC_SetupReqType;
static volatile uint16_t CDC_SetupReqValue;
static volatile uint16_t CDC_SetupReqIndex;
static volatile uint16_t CDC_SetupReqLen;

static volatile uint8_t  CDC_DevConfig;       /* current configuration value */
static volatile uint8_t  CDC_DevAddr;         /* pending address (applied @IN) */
static volatile uint8_t  CDC_DevSleepStatus;

static volatile uint8_t  s_configured;        /* set after SET_CONFIGURATION */

/* Dummy line coding: 115200 8N1. Stored on SET_LINE_CODING, echoed on GET. */
static volatile uint8_t  s_line_coding[7] = { 0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08 };

/* ------------------------------------------------------------------------ */
/* RX ring (EP2 OUT, host->device). The ISR latches the received length and */
/* NAKs the EP; cdc_fs_poll() drains the DMA buffer into the ring and        */
/* re-ACKs (the ISR-latch-and-NAK / foreground-drain-and-re-ACK pattern from */
/* usbfsd_poll_out). cdc_fs_rx_read() pops bytes for the caller.             */
/* ------------------------------------------------------------------------ */
#define CDC_RX_RING_SZ  512
static volatile uint8_t  s_rx_ring[CDC_RX_RING_SZ];
static volatile uint16_t s_rx_head;           /* producer (cdc_fs_poll)  */
static volatile uint16_t s_rx_tail;           /* consumer (cdc_fs_rx_read) */

static volatile uint8_t  s_ep2_rx_len;        /* bytes in EP2 DMA buffer  */
static volatile uint8_t  s_ep2_rx_pend;       /* an OUT awaits draining   */

/* ------------------------------------------------------------------------ */
/* TX ring (EP3 IN, device->host). cdc_fs_tx_write() pushes bytes;           */
/* cdc_fs_poll() (and the EP3 IN-complete ISR) loads the next packet into    */
/* the EP3 DMA buffer and arms it.                                           */
/* ------------------------------------------------------------------------ */
#define CDC_TX_RING_SZ  512
static volatile uint8_t  s_tx_ring[CDC_TX_RING_SZ];
static volatile uint16_t s_tx_head;           /* producer (cdc_fs_tx_write) */
static volatile uint16_t s_tx_tail;           /* consumer (cdc_fs_poll/ISR) */

/* ------------------------------------------------------------------------ */
/* ISR declaration — must match the vector table in core/startup_v5f.S.     */
/* ------------------------------------------------------------------------ */
void USBFS_IRQHandler(void) WCH_IRQ;

/* ------------------------------------------------------------------------ */
/* cdc_rcc_init — bring up the shared USBHS 480 MHz PLL (idempotent), derive  */
/* the 48 MHz USBFS clock (/10), enable OTG_FS + GPIOA. IDENTICAL to          */
/* usb_device_fs.c's usbfs_rcc_init: the PLL is shared with the USBHS host     */
/* controller and must NEVER be torn down here (Board B's host capture and the */
/* USBFS clock both depend on it).                                            */
/* ------------------------------------------------------------------------ */
static void cdc_rcc_init(void)
{
    if (!(RCC->CTLR & RCC_USBHS_PLLRDY)) {
        RCC_USBHS_PLLCmd(DISABLE);
        RCC_USBHSPLLCLKConfig((RCC->CTLR & RCC_HSERDY) ? RCC_USBHSPLLSource_HSE
                                                       : RCC_USBHSPLLSource_HSI);
        RCC_USBHSPLLReferConfig(RCC_USBHSPLLRefer_25M);
        RCC_USBHSPLLClockSourceDivConfig(RCC_USBHSPLL_IN_Div1);
        RCC_USBHS_PLLCmd(ENABLE);
        /* Bounded re-lock spin so a never-locking PLL cannot hang V5F. */
        for (uint32_t t = 0; t < 2000000u && !(RCC->CTLR & RCC_USBHS_PLLRDY); t++) { }
    }
    RCC_USBFSCLKConfig(RCC_USBFSCLKSource_USBHSPLL);
    RCC_USBFS48ClockSourceDivConfig(RCC_USBFS_Div10);
    RCC_HBPeriphClockCmd(RCC_HBPeriph_OTG_FS, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOA, ENABLE);
}

/* ------------------------------------------------------------------------ */
/* cdc_endp_init — configure EP0 (control), EP2 OUT (bulk), EP3 IN (bulk).    */
/* EP1 IN is declared in the config descriptor but never armed (we send no    */
/* CDC notifications). Re-runnable: the bus-reset handler calls it to re-arm   */
/* at DATA0.                                                                   */
/* Register packing: EP2/EP3 share UEP2_3_MOD, EP1 lives in UEP4_1_MOD.        */
/* ------------------------------------------------------------------------ */
static void cdc_endp_init(void)
{
    USBFSD->UEP4_1_MOD = 0;
    USBFSD->UEP2_3_MOD = 0;
    USBFSD->UEP5_6_MOD = 0;
    USBFSD->UEP7_MOD   = 0;

    /* EP0 control. */
    USBFSD->UEP0_DMA     = (uint32_t)(uintptr_t)CDC_EP_Buf[0];
    USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_RES_ACK;
    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;

    /* EP2 OUT (bulk command in) — ready to receive = ACK. */
    USBFSD_UEP_DMA(CDC_EP_OUT)     = (uint32_t)(uintptr_t)CDC_EP_Buf[CDC_EP_OUT];
    USBFSD_UEP_RX_CTRL(CDC_EP_OUT) = USBFS_UEP_R_RES_ACK;
    USBFSD->UEP2_3_MOD |= USBFS_UEP2_RX_EN;

    /* EP3 IN (bulk response out) — idle = NAK until cdc_fs_poll arms it. */
    USBFSD_UEP_DMA(CDC_EP_IN)     = (uint32_t)(uintptr_t)CDC_EP_Buf[CDC_EP_IN];
    USBFSD_UEP_TX_CTRL(CDC_EP_IN) = USBFS_UEP_T_RES_NAK;
    USBFSD->UEP2_3_MOD |= USBFS_UEP3_TX_EN;

    /* Reset ring/EP latch state. */
    s_ep2_rx_len  = 0;
    s_ep2_rx_pend = 0;
}

/* ======================================================================== */
/* Public API                                                               */
/* ======================================================================== */

void cdc_fs_init(void)
{
    cdc_rcc_init();

    /* Reset the SIE, clear all, then release (mirrors usb_device_fs.c). */
    USBFSH->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
    Delay_Us(10);
    USBFSD->BASE_CTRL = 0;

    cdc_endp_init();

    USBFSD->INT_EN    = USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER;
    USBFSD->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
    USBFSD->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;

    CDC_DevConfig      = 0;
    CDC_DevAddr        = 0;
    CDC_DevSleepStatus = 0;
    s_configured       = 0;
    s_rx_head = s_rx_tail = 0;
    s_tx_head = s_tx_tail = 0;

    // Dual-core IRQ routing: IRQn>31 are core-allocated via NVIC->IALLOCR with a
    // reset default of Core_ID_V3F. The USBFS ISR runs on V5F, so route USBFS_IRQn
    // to V5F before enabling it; otherwise the interrupt goes to V3F (no handler)
    // and USBFS_IRQHandler never fires — CDC silently never enumerates.
    NVIC_SetAllocateIRQ(USBFS_IRQn, Core_ID_V5F);
    NVIC_EnableIRQ(USBFS_IRQn);
}

bool cdc_fs_is_configured(void)
{
    return s_configured != 0;
}

uint16_t cdc_fs_rx_read(uint8_t *buf, uint16_t max)
{
    uint16_t n = 0;
    while (n < max && s_rx_tail != s_rx_head) {
        buf[n++] = s_rx_ring[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1) % CDC_RX_RING_SZ);
    }
    return n;
}

uint16_t cdc_fs_tx_write(const uint8_t *buf, uint16_t len)
{
    uint16_t n = 0;
    for (; n < len; n++) {
        uint16_t next = (uint16_t)((s_tx_head + 1) % CDC_TX_RING_SZ);
        if (next == s_tx_tail) break;       /* ring full */
        s_tx_ring[s_tx_head] = buf[n];
        s_tx_head = next;
    }
    return n;
}

/* Number of bytes currently queued in the tx ring. */
static uint16_t tx_ring_count(void)
{
    return (uint16_t)((s_tx_head - s_tx_tail + CDC_TX_RING_SZ) % CDC_TX_RING_SZ);
}

void cdc_fs_poll(void)
{
    /* ---- Drain a completed EP2 OUT into the rx ring, then re-ACK. ----
     * The ISR latched the length and left EP2 NAKing so the DMA buffer is
     * stable. Single-reader (this poll), single-writer (the ISR). */
    if (s_ep2_rx_pend) {
        uint8_t  n   = s_ep2_rx_len;
        uint8_t *src = USBFSD_UEP_BUF(CDC_EP_OUT);   /* +0x20000000 CPU alias */
        for (uint8_t i = 0; i < n; i++) {
            uint16_t next = (uint16_t)((s_rx_head + 1) % CDC_RX_RING_SZ);
            if (next == s_rx_tail) break;            /* ring full — drop rest */
            s_rx_ring[s_rx_head] = src[i];
            s_rx_head = next;
        }
        s_ep2_rx_pend = 0;
        /* Re-ACK so EP2 can receive the next OUT (toggle preserved). */
        USBFSD_UEP_RX_CTRL(CDC_EP_OUT) =
            (USBFSD_UEP_RX_CTRL(CDC_EP_OUT) & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
    }

    /* ---- Flush the tx ring onto EP3 IN when the endpoint is idle. ----
     * EP3 is idle when it is NAKing (no packet armed / previous one consumed).
     * Load up to one bulk packet and arm ACK; the IN-complete ISR re-NAKs and
     * flips the toggle, and a later poll loads the next packet. */
    if (s_configured && tx_ring_count() > 0 &&
        (USBFSD_UEP_TX_CTRL(CDC_EP_IN) & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_NAK) {
        uint8_t *dst = USBFSD_UEP_BUF(CDC_EP_IN);
        uint8_t  n   = 0;
        while (n < CDC_BULK_SIZE && s_tx_tail != s_tx_head) {
            dst[n++] = s_tx_ring[s_tx_tail];
            s_tx_tail = (uint16_t)((s_tx_tail + 1) % CDC_TX_RING_SZ);
        }
        USBFSD_UEP_TLEN(CDC_EP_IN) = n;
        USBFSD_UEP_TX_CTRL(CDC_EP_IN) =
            (USBFSD_UEP_TX_CTRL(CDC_EP_IN) & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;
    }
}

/* ======================================================================== */
/* USBFS_IRQHandler — enumeration state machine ported from usb_device_fs.c, */
/* serving static CDC descriptors and ACKing CDC class requests.            */
/* ======================================================================== */
void USBFS_IRQHandler(void)
{
    uint8_t  intflag, intst, errflag;
    uint16_t len;

    intflag = USBFSD->INT_FG;
    intst   = USBFSD->INT_ST;

    if (intflag & USBFS_UIF_TRANSFER) {
        switch (intst & USBFS_UIS_TOKEN_MASK) {

        /* ---- data-IN stage ---- */
        case USBFS_UIS_TOKEN_IN:
            switch (intst & (USBFS_UIS_TOKEN_MASK | USBFS_UIS_ENDP_MASK)) {

            /* EP0 IN: continue a multi-packet descriptor, or apply a pending
             * SET_ADDRESS at the status stage. */
            case USBFS_UIS_TOKEN_IN | DEF_UEP0:
                if (CDC_SetupReqLen == 0) {
                    USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_ACK;
                }
                if ((CDC_SetupReqType & USB_REQ_TYP_MASK) == USB_REQ_TYP_STANDARD) {
                    switch (CDC_SetupReqCode) {
                    case USB_GET_DESCRIPTOR:
                        len = CDC_SetupReqLen >= CDC_EP0_SIZE ? CDC_EP0_SIZE
                                                              : CDC_SetupReqLen;
                        memcpy(CDC_EP0_Buf, pCDC_Descr, len);
                        CDC_SetupReqLen -= len;
                        pCDC_Descr      += len;
                        USBFSD->UEP0_TX_LEN  = len;
                        USBFSD->UEP0_TX_CTRL ^= USBFS_UEP_T_TOG;
                        break;
                    case USB_SET_ADDRESS:
                        USBFSD->DEV_ADDR =
                            (USBFSD->DEV_ADDR & USBFS_UDA_GP_BIT) | CDC_DevAddr;
                        break;
                    default:
                        break;
                    }
                }
                break;

            /* EP3 IN complete (bulk response): host consumed the packet. Re-NAK
             * and flip the toggle; cdc_fs_poll loads the next packet. */
            default: {
                uint8_t ep = intst & USBFS_UIS_ENDP_MASK;
                if (ep == CDC_EP_IN) {
                    USBFSD_UEP_TX_CTRL(CDC_EP_IN) =
                        (USBFSD_UEP_TX_CTRL(CDC_EP_IN) & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
                    USBFSD_UEP_TX_CTRL(CDC_EP_IN) ^= USBFS_UEP_T_TOG;
                }
                break;
            }
            }
            break;

        /* ---- data-OUT stage ---- */
        case USBFS_UIS_TOKEN_OUT:
            switch (intst & (USBFS_UIS_TOKEN_MASK | USBFS_UIS_ENDP_MASK)) {

            /* EP0 OUT: status stage of a control-write, or the OUT-data stage of
             * a class write (SET_LINE_CODING). The 7-byte line coding lands in
             * the EP0 buffer; capture it. */
            case USBFS_UIS_TOKEN_OUT | DEF_UEP0:
                if (intst & USBFS_UIS_TOG_OK) {
                    if ((CDC_SetupReqType & USB_REQ_TYP_MASK) == USB_REQ_TYP_CLASS &&
                        CDC_SetupReqCode == CDC_SET_LINE_CODING) {
                        uint16_t pk = USBFSD->RX_LEN;
                        if (pk > 7) pk = 7;
                        for (uint16_t b = 0; b < pk; b++)
                            s_line_coding[b] = CDC_EP0_Buf[b];
                    }
                }
                /* Single-packet (<=64) control writes: close with a ZLP status. */
                USBFSD->UEP0_TX_LEN  = 0;
                USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
                break;

            /* EP2 OUT (bulk command): latch the length and NAK so the DMA buffer
             * isn't overwritten before cdc_fs_poll drains it and re-ACKs. */
            default: {
                uint8_t ep = intst & USBFS_UIS_ENDP_MASK;
                if (ep == CDC_EP_OUT) {
                    uint16_t n = USBFSD->RX_LEN;
                    if (n > CDC_BULK_SIZE) n = CDC_BULK_SIZE;
                    s_ep2_rx_len  = (uint8_t)n;
                    s_ep2_rx_pend = 1;
                    USBFSD_UEP_RX_CTRL(CDC_EP_OUT) =
                        (USBFSD_UEP_RX_CTRL(CDC_EP_OUT) & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_NAK;
                }
                break;
            }
            }
            break;

        /* ---- SETUP stage ---- */
        case USBFS_UIS_TOKEN_SETUP:
            USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_NAK;
            USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_NAK;

            CDC_SetupReqType  = pCDC_SetupReqPak->bRequestType;
            CDC_SetupReqCode  = pCDC_SetupReqPak->bRequest;
            CDC_SetupReqLen   = pCDC_SetupReqPak->wLength;
            CDC_SetupReqValue = pCDC_SetupReqPak->wValue;
            CDC_SetupReqIndex = pCDC_SetupReqPak->wIndex;
            len     = 0;
            errflag = 0;

            if ((CDC_SetupReqType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD) {
                /* ---- CDC class requests: ACK, never STALL ---- */
                if ((CDC_SetupReqType & USB_REQ_TYP_MASK) == USB_REQ_TYP_CLASS) {
                    switch (CDC_SetupReqCode) {
                    case CDC_GET_LINE_CODING:
                        /* Return the stored 7-byte line coding. */
                        for (uint8_t b = 0; b < 7; b++)
                            CDC_EP0_Buf[b] = s_line_coding[b];
                        len = 7;
                        break;
                    case CDC_SET_LINE_CODING:
                        /* 7-byte OUT payload follows; captured in the EP0 OUT
                         * stage. Just ACK here (SetupReqLen != 0 -> RX armed). */
                        break;
                    case CDC_SET_LINE_CTLSTE:
                        /* SET_CONTROL_LINE_STATE: no data stage, just ACK. */
                        break;
                    default:
                        /* Unknown class request: ACK with no data to stay robust
                         * (SEND_BREAK etc.). */
                        break;
                    }
                } else {
                    errflag = 0xFF;
                }
            } else {
                /* ---- standard requests ---- */
                switch (CDC_SetupReqCode) {
                case USB_GET_DESCRIPTOR:
                    switch ((uint8_t)(CDC_SetupReqValue >> 8)) {
                    case USB_DESCR_TYP_DEVICE:
                        pCDC_Descr = CDC_DevDescr;
                        len = CDC_DevDescr[0];
                        break;
                    case USB_DESCR_TYP_CONFIG:
                        pCDC_Descr = CDC_CfgDescr;
                        len = (uint16_t)CDC_CfgDescr[2] | ((uint16_t)CDC_CfgDescr[3] << 8);
                        break;
                    case USB_DESCR_TYP_STRING:
                        switch ((uint8_t)(CDC_SetupReqValue & 0xFF)) {
                        case 0x00: pCDC_Descr = CDC_LangDescr; len = CDC_LangDescr[0]; break;
                        case 0x01: pCDC_Descr = CDC_ManuDescr; len = CDC_ManuDescr[0]; break;
                        case 0x02: pCDC_Descr = CDC_ProdDescr; len = CDC_ProdDescr[0]; break;
                        default:   errflag = 0xFF; break;
                        }
                        break;
                    default:
                        /* device_qualifier (FS-only) and anything else: STALL. */
                        errflag = 0xFF;
                        break;
                    }
                    if (errflag != 0xFF) {
                        if (CDC_SetupReqLen > len) CDC_SetupReqLen = len;
                        len = (CDC_SetupReqLen >= CDC_EP0_SIZE) ? CDC_EP0_SIZE
                                                                : CDC_SetupReqLen;
                        memcpy(CDC_EP0_Buf, pCDC_Descr, len);
                        pCDC_Descr += len;
                    }
                    break;

                case USB_SET_ADDRESS:
                    CDC_DevAddr = (uint8_t)(CDC_SetupReqValue & 0xFF);
                    break;

                case USB_GET_CONFIGURATION:
                    CDC_EP0_Buf[0] = CDC_DevConfig;
                    if (CDC_SetupReqLen > 1) CDC_SetupReqLen = 1;
                    break;

                case USB_SET_CONFIGURATION:
                    CDC_DevConfig = (uint8_t)(CDC_SetupReqValue & 0xFF);
                    s_configured  = (CDC_DevConfig != 0);
                    break;

                case USB_GET_INTERFACE:
                    CDC_EP0_Buf[0] = 0x00;
                    if (CDC_SetupReqLen > 1) CDC_SetupReqLen = 1;
                    break;

                case USB_SET_INTERFACE:
                    break;

                case USB_CLEAR_FEATURE:
                    if ((CDC_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) {
                        if ((uint8_t)(CDC_SetupReqValue & 0xFF) == USB_REQ_FEAT_ENDP_HALT) {
                            uint8_t ep   = (uint8_t)(CDC_SetupReqIndex & 0x0F);
                            bool    isin = (CDC_SetupReqIndex & DEF_UEP_IN) != 0;
                            if (isin && ep == CDC_EP_IN) {
                                USBFSD_UEP_TX_CTRL(CDC_EP_IN) =
                                    (USBFSD_UEP_TX_CTRL(CDC_EP_IN) & ~(USBFS_UEP_T_RES_MASK | USBFS_UEP_T_TOG))
                                    | USBFS_UEP_T_RES_NAK;
                            } else if (!isin && ep == CDC_EP_OUT) {
                                USBFSD_UEP_RX_CTRL(CDC_EP_OUT) =
                                    (USBFSD_UEP_RX_CTRL(CDC_EP_OUT) & ~(USBFS_UEP_R_RES_MASK | USBFS_UEP_R_TOG))
                                    | USBFS_UEP_R_RES_ACK;
                            } else {
                                errflag = 0xFF;
                            }
                        } else {
                            errflag = 0xFF;
                        }
                    } else if ((CDC_SetupReqType & USB_REQ_RECIP_MASK) != USB_REQ_RECIP_DEVICE) {
                        errflag = 0xFF;
                    }
                    /* device-recipient CLEAR_FEATURE (remote wakeup): ACK no-op. */
                    break;

                case USB_GET_STATUS:
                    CDC_EP0_Buf[0] = 0x00;
                    CDC_EP0_Buf[1] = 0x00;
                    if (CDC_SetupReqLen > 2) CDC_SetupReqLen = 2;
                    break;

                default:
                    errflag = 0xFF;
                    break;
                }
            }

            /* Drive the EP0 response: STALL on error, else Tx/Rx data or a
             * zero-length status stage. */
            if (errflag == 0xFF) {
                USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_STALL;
                USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_STALL;
            } else {
                if (CDC_SetupReqType & DEF_UEP_IN) {
                    /* device-to-host: first data packet (DATA1) */
                    len = (CDC_SetupReqLen > CDC_EP0_SIZE) ? CDC_EP0_SIZE
                                                           : CDC_SetupReqLen;
                    CDC_SetupReqLen     -= len;
                    USBFSD->UEP0_TX_LEN  = len;
                    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
                } else {
                    /* host-to-device (or no-data): status / OUT-data stage */
                    if (CDC_SetupReqLen == 0) {
                        USBFSD->UEP0_TX_LEN  = 0;
                        USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
                    } else {
                        USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_ACK;
                    }
                }
            }
            break;

        default:
            break;
        }
        USBFSD->INT_FG = USBFS_UIF_TRANSFER;
    } else if (intflag & USBFS_UIF_BUS_RST) {
        /* bus reset: drop address/config, re-init endpoints. */
        CDC_DevConfig      = 0;
        CDC_DevAddr        = 0;
        CDC_DevSleepStatus = 0;
        s_configured       = 0;
        USBFSD->DEV_ADDR   = 0;
        cdc_endp_init();
        USBFSD->INT_FG = USBFS_UIF_BUS_RST;
    } else if (intflag & USBFS_UIF_SUSPEND) {
        USBFSD->INT_FG = USBFS_UIF_SUSPEND;
        // No Delay_Us() here: ISR context on V5F, shared non-reentrant SysTick
        // wait (see usb_device_fs.c). Read the suspend bit directly.
        if (USBFSD->MIS_ST & USBFS_UMS_SUSPEND) {
            CDC_DevSleepStatus |= 0x02;
        } else {
            CDC_DevSleepStatus &= ~0x02;
        }
    } else {
        USBFSD->INT_FG = intflag;
    }
}

/* ======================================================================== */
/* usbfsd_* no-op stubs.                                                     */
/*                                                                           */
/* The host image drops src/usb_device_fs.c (to free USBFS_IRQHandler for    */
/* the CDC driver) but KEEPS src/usb_device.c (the speed dispatcher), whose  */
/* FS branch references these symbols. On the host image that branch is dead */
/* code — two_board_host_run() never returns, so the USB-device-clone path   */
/* (main_v5f.c single-board relay / usb_merge.c) is never reached. These      */
/* stubs satisfy the linker without pulling the real FS-clone ISR back in.   */
/* They are compiled ONLY into the host image (usb_cdc_fs.c is host-only);    */
/* the device image links the real implementations from usb_device_fs.c.     */
/* ======================================================================== */
bool usbfsd_init(const captured_descriptors_t *desc) { (void)desc; return false; }
void usbfsd_poll(void) { }
bool usbfsd_send_report(uint8_t ep_num, const uint8_t *data, uint16_t len)
{ (void)ep_num; (void)data; (void)len; return false; }
bool usbfsd_is_configured(void) { return false; }
bool usbfsd_in_ep_free(uint8_t ep_num) { (void)ep_num; return false; }
int  usbfsd_poll_out(uint8_t ep_num, uint8_t **data_ptr)
{ (void)ep_num; (void)data_ptr; return -1; }
int  usbfsd_poll_ep0_report(uint8_t **data_ptr, uint16_t *wValue, uint16_t *wIndex)
{ (void)data_ptr; (void)wValue; (void)wIndex; return 0; }
void usbfsd_ep0_report_done(void) { }
