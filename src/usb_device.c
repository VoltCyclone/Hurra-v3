// usb_device.c — USBFS device driver (PC-facing HID device side).
//
// Runs on the V5F core. Implements the stable usb_device.h API against the
// CH32H417 USBFS controller in Full-Speed (12 Mbps) DEVICE mode. The EP0
// standard-request enumeration state machine is ported faithfully from the
// WCH reference driver:
//   vendor/wch/usb_reference/USBFS_CompositeKM/Common/ch32h417_usbfs_device.c
// reduced to a single HID interface with one interrupt-IN endpoint (EP1),
// serving the static boot-mouse descriptors in usb_descriptors_static.h.
//
// Phase 4 scope: COMPILE + faithful enumeration FSM. Hardware enumeration is
// a later bench step. Phase 5 replaces the static descriptors with cloned
// ones (the `desc` argument to usb_device_init) and adds OUT-EP support.
//
// DMA-buffer convention (WCH FS): the UEPn_DMA registers are programmed with
// the RAW buffer address; CPU access to that buffer uses the same address but
// with a +0x20000000 alias (USBFSD_UEP_BUF). We follow the reference exactly.

#include "ch32h417_port.h"      // ch32h417.h (USBFSD struct, RCC), usb bit defs
#include "debug.h"              // Delay_Us
#include <string.h>

#include "usb_device.h"
#include "usb_descriptors_static.h"

/* ------------------------------------------------------------------------ */
/* USBFS register helper macros (ported from ch32h417_usbfs_device.h).      */
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
#define DEF_UEP1                    0x01

/* Setup-request packet alias over EP0 DMA buffer (CPU side). */
#define pUSBFS_SetupReqPak          ((PUSB_SETUP_REQ)USBFS_EP0_Buf)

/* ------------------------------------------------------------------------ */
/* DMA buffers — placed in the dedicated .usbdma section (uncached SRAM).   */
/* EP0: 64 bytes (control). EP1: 64 bytes (FS max packet; report is 4).     */
/* ------------------------------------------------------------------------ */
__attribute__((section(".usbdma"), aligned(4))) static uint8_t USBFS_EP0_Buf[USBD_EP0_SIZE];
__attribute__((section(".usbdma"), aligned(4))) static uint8_t USBFS_EP1_Buf[64];

/* ------------------------------------------------------------------------ */
/* Enumeration / setup state.                                               */
/* ------------------------------------------------------------------------ */
static const uint8_t *pUSBFS_Descr;          /* current descriptor walk ptr */

static volatile uint8_t  USBFS_SetupReqCode;
static volatile uint8_t  USBFS_SetupReqType;
static volatile uint16_t USBFS_SetupReqValue;
static volatile uint16_t USBFS_SetupReqIndex;
static volatile uint16_t USBFS_SetupReqLen;

static volatile uint8_t  USBFS_DevConfig;     /* current configuration value */
static volatile uint8_t  USBFS_DevAddr;       /* pending address (applied @IN) */
static volatile uint8_t  USBFS_DevSleepStatus;

static volatile uint8_t  s_configured;        /* set after SET_CONFIGURATION */

/* BENCH DIAG: USBFS device-side activity counters, so the V3F UART oracle can
 * show whether the device PHY sees the PC at all when s_configured stays 0:
 *   irq    = total USBFS_IRQHandler entries (0 => no bus activity / not wired)
 *   busrst = bus-reset interrupts (host driving reset => device is on the bus)
 *   setup  = SETUP packets seen (host enumerating)
 *   lastst = last INT_ST token snapshot. */
volatile uint32_t usbd_dbg_irq;
volatile uint32_t usbd_dbg_busrst;
volatile uint32_t usbd_dbg_setup;
volatile uint32_t usbd_dbg_lastst;
volatile uint32_t usbd_dbg_alloc_before;   /* NVIC IRQ-alloc bit before SetAllocate */
volatile uint32_t usbd_dbg_alloc_after;    /* and after (1 => routed to V5F) */
/* Last SETUP request seen + a STALL (errflag) counter, so the UART oracle can
 * show exactly which control request Windows sends that the device chokes on.
 * usbd_dbg_lastsetup = bRequest<<24 | bmRequestType<<16 | wValue. */
volatile uint32_t usbd_dbg_lastsetup;
volatile uint32_t usbd_dbg_lastsetup_len;  /* wLength of the last SETUP */
volatile uint32_t usbd_dbg_stalls;         /* count of errflag=0xFF (STALL) responses */
volatile uint32_t usbd_dbg_configval;      /* SET_CONFIGURATION value seen */

/* HID class state (single interface). */
static volatile uint8_t  USBFS_HidIdle;
static volatile uint8_t  USBFS_HidProtocol;

/* ------------------------------------------------------------------------ */
/* ISR declaration — must match the vector table in core/startup_v5f.S.     */
/* ------------------------------------------------------------------------ */
void USBFS_IRQHandler(void) WCH_IRQ;

/* ------------------------------------------------------------------------ */
/* usbfs_rcc_init — port of USBFS_RCC_Init.                                 */
/*                                                                          */
/* Brings up the USBHS 480 MHz PLL, derives the 48 MHz USBFS clock from it  */
/* (/10), and enables the OTG_FS peripheral + GPIOA clocks.                 */
/*                                                                          */
/* DEPENDENCY NOTE: this enables the shared USBHS 480M PLL. A later task    */
/* initializes the USBHS HOST controller, which also needs this PLL. The    */
/* reference guards the PLL bring-up with a "already selected" check, so    */
/* re-running it is safe; but if USBHS host init reconfigures the PLL it    */
/* must not tear it down while USBFS is live. Tracked for Task 5.x.         */
/* ------------------------------------------------------------------------ */
static void usbfs_rcc_init(void)
{
    if ((RCC->PLLCFGR & RCC_SYSPLL_SEL) != RCC_SYSPLL_USBHS) {
        /* Initialize USBHS 480M PLL */
        RCC_USBHS_PLLCmd(DISABLE);
        RCC_USBHSPLLCLKConfig((RCC->CTLR & RCC_HSERDY) ? RCC_USBHSPLLSource_HSE
                                                       : RCC_USBHSPLLSource_HSI);
        RCC_USBHSPLLReferConfig(RCC_USBHSPLLRefer_25M);
        RCC_USBHSPLLClockSourceDivConfig(RCC_USBHSPLL_IN_Div1);
        RCC_USBHS_PLLCmd(ENABLE);
        while (!(RCC->CTLR & RCC_USBHS_PLLRDY));
    }
    RCC_USBFSCLKConfig(RCC_USBFSCLKSource_USBHSPLL);
    RCC_USBFS48ClockSourceDivConfig(RCC_USBFS_Div10);
    RCC_HBPeriphClockCmd(RCC_HBPeriph_OTG_FS, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOA, ENABLE);
}

/* ------------------------------------------------------------------------ */
/* usbfs_endp_init — port of USBFS_Device_Endp_Init (single IN endpoint).   */
/* ------------------------------------------------------------------------ */
static void usbfs_endp_init(void)
{
    /* Enable EP1 TX (IN). UEP4_1_MOD controls EP1 (and EP4). */
    USBFSD->UEP4_1_MOD = USBFS_UEP1_TX_EN;

    USBFSD->UEP0_DMA = (uint32_t)(uintptr_t)USBFS_EP0_Buf;
    USBFSD->UEP1_DMA = (uint32_t)(uintptr_t)USBFS_EP1_Buf;

    USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_RES_ACK;
    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
    USBFSD->UEP1_TX_CTRL = USBFS_UEP_T_RES_NAK;
}

/* ======================================================================== */
/* Public API                                                               */
/* ======================================================================== */

bool usb_device_init(const captured_descriptors_t *desc)
{
    (void)desc;  /* Phase 4: use static descriptors; Phase 5 clones `desc`. */

    usbfs_rcc_init();

    /* Reset the SIE, clear all, then release. */
    USBFSH->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
    Delay_Us(10);
    USBFSD->BASE_CTRL = 0;

    usbfs_endp_init();

    USBFSD->INT_EN    = USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER;
    USBFSD->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
    USBFSD->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;

    USBFS_DevConfig      = 0;
    USBFS_DevAddr        = 0;
    USBFS_DevSleepStatus = 0;
    s_configured         = 0;

    // DUAL-CORE IRQ ROUTING: IRQn>31 are core-allocated via NVIC->IALLOCR, and the
    // RESET DEFAULT is Core_ID_V3F (0). The USBFS enumeration ISR runs on V5F, so
    // without this the interrupt is delivered to V3F (which has no USBFS handler)
    // and V5F's USBFS_IRQHandler never fires — s_configured stays 0 forever. Route
    // USBFS_IRQn (67) to V5F before enabling it. Publish the pre/post allocation
    // bit for the UART oracle. (USBHS host works without this because it's polled.)
    usbd_dbg_alloc_before = NVIC_GetAllocateIRQ(USBFS_IRQn);
    NVIC_SetAllocateIRQ(USBFS_IRQn, Core_ID_V5F);
    usbd_dbg_alloc_after = NVIC_GetAllocateIRQ(USBFS_IRQn);

    NVIC_EnableIRQ(USBFS_IRQn);
    return true;
}

void usb_device_poll(void)
{
    /* Housekeeping only — the IRQ handler does the real work. Kept for API
     * parity with the v2 driver / host-side loop. */
}

bool usb_device_send_report(uint8_t ep_num, const uint8_t *data, uint16_t len)
{
    if (ep_num == 0 || ep_num >= USB_DEV_NUM_ENDPOINTS) {
        return false;
    }

    /* Don't arm an IN endpoint before the host has finished enumeration
     * (SET_CONFIGURATION). The reference only fills EP buffers from its
     * enumerated main loop; arming pre-config can mis-seed the DATA0/DATA1
     * toggle across a bus reset and has no effect anyway (the host won't poll
     * an unconfigured device). A bus reset clears s_configured, so this also
     * closes the reset window. */
    if (!s_configured) {
        return false;
    }

    /* TX busy if the endpoint is NOT currently NAK'ing (i.e. a previous
     * report is still armed and has not been consumed by the host). */
    if ((USBFSD_UEP_TX_CTRL(ep_num) & USBFS_UEP_T_RES_MASK) != USBFS_UEP_T_RES_NAK) {
        return false;
    }

    if (len > 64) {
        len = 64;
    }

    /* Copy into the endpoint's DMA buffer via the +0x20000000 CPU alias. */
    memcpy(USBFSD_UEP_BUF(ep_num), data, len);
    USBFSD_UEP_TLEN(ep_num) = len;

    /* Arm: set response to ACK, preserving the current data-toggle bit.
     * The IRQ flips T_TOG after each successful IN, so we just clear the
     * response field and set ACK here. */
    USBFSD_UEP_TX_CTRL(ep_num) =
        (USBFSD_UEP_TX_CTRL(ep_num) & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;

    return true;
}

bool usb_device_is_configured(void)
{
    return s_configured != 0;
}

int usb_device_poll_out(uint8_t ep_num, uint8_t **data_ptr)
{
    (void)ep_num;
    (void)data_ptr;
    /* OUT-EP support arrives in Phase 5. */
    return 0;
}

/* ======================================================================== */
/* USBFS_IRQHandler — ported standard-request enumeration state machine.    */
/* ======================================================================== */
void USBFS_IRQHandler(void)
{
    uint8_t  intflag, intst, errflag;
    uint16_t len;

    intflag = USBFSD->INT_FG;
    intst   = USBFSD->INT_ST;

    usbd_dbg_irq++;            /* bench: count every IRQ entry */
    usbd_dbg_lastst = intst;
    if (intflag & USBFS_UIF_BUS_RST) usbd_dbg_busrst++;
    if ((intst & USBFS_UIS_TOKEN_MASK) == USBFS_UIS_TOKEN_SETUP) usbd_dbg_setup++;

    if (intflag & USBFS_UIF_TRANSFER) {
        switch (intst & USBFS_UIS_TOKEN_MASK) {

        /* ---- data-IN stage ---- */
        case USBFS_UIS_TOKEN_IN:
            switch (intst & (USBFS_UIS_TOKEN_MASK | USBFS_UIS_ENDP_MASK)) {

            /* EP0 IN: continue serving a multi-packet descriptor, or apply
             * a pending SET_ADDRESS at the status stage. */
            case USBFS_UIS_TOKEN_IN | DEF_UEP0:
                if (USBFS_SetupReqLen == 0) {
                    USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_ACK;
                }

                if ((USBFS_SetupReqType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD) {
                    /* Non-standard EP0 data upload — none in Phase 4. */
                } else {
                    switch (USBFS_SetupReqCode) {
                    case USB_GET_DESCRIPTOR:
                        len = USBFS_SetupReqLen >= USBD_EP0_SIZE ? USBD_EP0_SIZE
                                                                 : USBFS_SetupReqLen;
                        memcpy(USBFS_EP0_Buf, pUSBFS_Descr, len);
                        USBFS_SetupReqLen -= len;
                        pUSBFS_Descr      += len;
                        USBFSD->UEP0_TX_LEN  = len;
                        USBFSD->UEP0_TX_CTRL ^= USBFS_UEP_T_TOG;
                        break;

                    case USB_SET_ADDRESS:
                        USBFSD->DEV_ADDR =
                            (USBFSD->DEV_ADDR & USBFS_UDA_GP_BIT) | USBFS_DevAddr;
                        break;

                    default:
                        break;
                    }
                }
                break;

            /* EP1 IN: report consumed by host. Re-NAK and flip toggle so the
             * next usb_device_send_report sends fresh DATAx. */
            case USBFS_UIS_TOKEN_IN | DEF_UEP1:
                USBFSD->UEP1_TX_CTRL =
                    (USBFSD->UEP1_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
                USBFSD->UEP1_TX_CTRL ^= USBFS_UEP_T_TOG;
                break;

            default:
                break;
            }
            break;

        /* ---- data-OUT stage ---- */
        case USBFS_UIS_TOKEN_OUT:
            switch (intst & (USBFS_UIS_TOKEN_MASK | USBFS_UIS_ENDP_MASK)) {

            /* EP0 OUT: status stage of a control-write, or class data
             * download (HID_SET_REPORT) — acked, no payload consumer yet. */
            case USBFS_UIS_TOKEN_OUT | DEF_UEP0:
                if (intst & USBFS_UIS_TOG_OK) {
                    if ((USBFS_SetupReqType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD) {
                        if ((USBFS_SetupReqType & USB_REQ_TYP_MASK) == USB_REQ_TYP_CLASS) {
                            switch (USBFS_SetupReqCode) {
                            case HID_SET_REPORT:
                                /* e.g. keyboard LED state; ignored for mouse. */
                                USBFS_SetupReqLen = 0;
                                break;
                            default:
                                break;
                            }
                        }
                    }
                }
                if (USBFS_SetupReqLen == 0) {
                    USBFSD->UEP0_TX_LEN  = 0;
                    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
                }
                break;

            default:
                break;
            }
            break;

        /* ---- SETUP stage ---- */
        case USBFS_UIS_TOKEN_SETUP:
            USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_NAK;
            USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_NAK;

            /* Latch the 8-byte setup packet from the EP0 DMA buffer. */
            USBFS_SetupReqType  = pUSBFS_SetupReqPak->bRequestType;
            USBFS_SetupReqCode  = pUSBFS_SetupReqPak->bRequest;
            USBFS_SetupReqLen   = pUSBFS_SetupReqPak->wLength;
            USBFS_SetupReqValue = pUSBFS_SetupReqPak->wValue;
            USBFS_SetupReqIndex = pUSBFS_SetupReqPak->wIndex;
            /* bench: record the last SETUP so the oracle shows where enum stalls. */
            usbd_dbg_lastsetup = ((uint32_t)USBFS_SetupReqCode << 24)
                               | ((uint32_t)USBFS_SetupReqType << 16)
                               | (uint32_t)USBFS_SetupReqValue;
            usbd_dbg_lastsetup_len = USBFS_SetupReqLen;
            len     = 0;
            errflag = 0;

            if ((USBFS_SetupReqType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD) {
                /* ---- HID class requests ---- */
                if ((USBFS_SetupReqType & USB_REQ_TYP_MASK) == USB_REQ_TYP_CLASS) {
                    switch (USBFS_SetupReqCode) {
                    case HID_SET_REPORT:
                        break;
                    case HID_SET_IDLE:
                        USBFS_HidIdle = (uint8_t)(USBFS_SetupReqValue >> 8);
                        break;
                    case HID_SET_PROTOCOL:
                        USBFS_HidProtocol = (uint8_t)USBFS_SetupReqValue;
                        break;
                    case HID_GET_IDLE:
                        USBFS_EP0_Buf[0] = USBFS_HidIdle;
                        len = 1;
                        break;
                    case HID_GET_PROTOCOL:
                        USBFS_EP0_Buf[0] = USBFS_HidProtocol;
                        len = 1;
                        break;
                    default:
                        errflag = 0xFF;
                        break;
                    }
                } else {
                    errflag = 0xFF;
                }
            } else {
                /* ---- standard requests ---- */
                switch (USBFS_SetupReqCode) {
                case USB_GET_DESCRIPTOR:
                    switch ((uint8_t)(USBFS_SetupReqValue >> 8)) {
                    case USB_DESCR_TYP_DEVICE:
                        pUSBFS_Descr = USBD_DeviceDescr;
                        len = USBD_DeviceDescr[0];
                        break;

                    case USB_DESCR_TYP_CONFIG:
                        pUSBFS_Descr = USBD_ConfigDescr;
                        /* wTotalLength from the config descriptor itself. */
                        len = (uint16_t)USBD_ConfigDescr[2] |
                              ((uint16_t)USBD_ConfigDescr[3] << 8);
                        break;

                    case USB_DESCR_TYP_HID:
                        /* HID descriptor lives inside the config set @offset 18. */
                        pUSBFS_Descr = &USBD_ConfigDescr[18];
                        len = 9;
                        break;

                    case USB_DESCR_TYP_REPORT:
                        pUSBFS_Descr = USBD_ReportDescr;
                        len = sizeof(USBD_ReportDescr);
                        break;

                    case USB_DESCR_TYP_STRING:
                        switch ((uint8_t)(USBFS_SetupReqValue & 0xFF)) {
                        case DEF_STRING_DESC_LANG:
                            pUSBFS_Descr = USBD_LangDescr;
                            len = USBD_LangDescr[0];
                            break;
                        case DEF_STRING_DESC_MANU:
                            pUSBFS_Descr = USBD_ManuDescr;
                            len = USBD_ManuDescr[0];
                            break;
                        case DEF_STRING_DESC_PROD:
                            pUSBFS_Descr = USBD_ProdDescr;
                            len = USBD_ProdDescr[0];
                            break;
                        default:
                            errflag = 0xFF;
                            break;
                        }
                        break;

                    default:
                        errflag = 0xFF;
                        break;
                    }

                    /* Clamp to requested length, prime first packet. */
                    if (USBFS_SetupReqLen > len) {
                        USBFS_SetupReqLen = len;
                    }
                    len = (USBFS_SetupReqLen >= USBD_EP0_SIZE) ? USBD_EP0_SIZE
                                                              : USBFS_SetupReqLen;
                    memcpy(USBFS_EP0_Buf, pUSBFS_Descr, len);
                    pUSBFS_Descr += len;
                    break;

                case USB_SET_ADDRESS:
                    USBFS_DevAddr = (uint8_t)(USBFS_SetupReqValue & 0xFF);
                    break;

                case USB_GET_CONFIGURATION:
                    USBFS_EP0_Buf[0] = USBFS_DevConfig;
                    if (USBFS_SetupReqLen > 1) {
                        USBFS_SetupReqLen = 1;
                    }
                    break;

                case USB_SET_CONFIGURATION:
                    USBFS_DevConfig = (uint8_t)(USBFS_SetupReqValue & 0xFF);
                    s_configured = 1;
                    usbd_dbg_configval = 0x100u | USBFS_DevConfig;  /* bench: saw SET_CONFIG */
                    break;

                case USB_CLEAR_FEATURE:
                    if ((USBFS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE) {
                        if ((uint8_t)(USBFS_SetupReqValue & 0xFF) == USB_REQ_FEAT_REMOTE_WAKEUP) {
                            USBFS_DevSleepStatus &= ~0x01;
                        } else {
                            errflag = 0xFF;
                        }
                    } else if ((USBFS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) {
                        if ((uint8_t)(USBFS_SetupReqValue & 0xFF) == USB_REQ_FEAT_ENDP_HALT) {
                            if ((uint8_t)(USBFS_SetupReqIndex & 0xFF) == (DEF_UEP_IN | DEF_UEP1)) {
                                /* un-stall EP1 IN, return to NAK */
                                USBFSD->UEP1_TX_CTRL =
                                    (USBFSD->UEP1_TX_CTRL & ~(USBFS_UEP_T_RES_MASK | USBFS_UEP_T_TOG))
                                    | USBFS_UEP_T_RES_NAK;
                            } else {
                                errflag = 0xFF;
                            }
                        } else {
                            errflag = 0xFF;
                        }
                    } else {
                        errflag = 0xFF;
                    }
                    break;

                case USB_SET_FEATURE:
                    if ((USBFS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE) {
                        if ((uint8_t)(USBFS_SetupReqValue & 0xFF) == USB_REQ_FEAT_REMOTE_WAKEUP) {
                            if (USBD_ConfigDescr[7] & 0x20) {
                                USBFS_DevSleepStatus |= 0x01;
                            } else {
                                errflag = 0xFF;
                            }
                        } else {
                            errflag = 0xFF;
                        }
                    } else if ((USBFS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) {
                        if ((uint8_t)(USBFS_SetupReqValue & 0xFF) == USB_REQ_FEAT_ENDP_HALT) {
                            if ((uint8_t)(USBFS_SetupReqIndex & 0xFF) == (DEF_UEP_IN | DEF_UEP1)) {
                                USBFSD->UEP1_TX_CTRL =
                                    (USBFSD->UEP1_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_STALL;
                            } else {
                                errflag = 0xFF;
                            }
                        } else {
                            errflag = 0xFF;
                        }
                    } else {
                        errflag = 0xFF;
                    }
                    break;

                case USB_GET_INTERFACE:
                    USBFS_EP0_Buf[0] = 0x00;
                    if (USBFS_SetupReqLen > 1) {
                        USBFS_SetupReqLen = 1;
                    }
                    break;

                case USB_SET_INTERFACE:
                    break;

                case USB_GET_STATUS:
                    USBFS_EP0_Buf[0] = 0x00;
                    USBFS_EP0_Buf[1] = 0x00;
                    if ((USBFS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE) {
                        if (USBFS_DevSleepStatus & 0x01) {
                            USBFS_EP0_Buf[0] = 0x02;
                        }
                    } else if ((USBFS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) {
                        if ((uint8_t)(USBFS_SetupReqIndex & 0xFF) == (DEF_UEP_IN | DEF_UEP1)) {
                            if ((USBFSD->UEP1_TX_CTRL & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_STALL) {
                                USBFS_EP0_Buf[0] = 0x01;
                            }
                        } else {
                            errflag = 0xFF;
                        }
                    } else {
                        errflag = 0xFF;
                    }
                    if (USBFS_SetupReqLen > 2) {
                        USBFS_SetupReqLen = 2;
                    }
                    break;

                default:
                    errflag = 0xFF;
                    break;
                }
            }

            /* Drive the EP0 response: STALL on error, else Tx/Rx the data or
             * zero-length status stage. */
            if (errflag == 0xFF) {
                usbd_dbg_stalls++;   /* bench: device STALLed this request */
                USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_STALL;
                USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_STALL;
            } else {
                if (USBFS_SetupReqType & DEF_UEP_IN) {
                    /* device-to-host: first data packet (DATA1) */
                    len = (USBFS_SetupReqLen > USBD_EP0_SIZE) ? USBD_EP0_SIZE
                                                             : USBFS_SetupReqLen;
                    USBFS_SetupReqLen   -= len;
                    USBFSD->UEP0_TX_LEN  = len;
                    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
                } else {
                    /* host-to-device (or no-data): status / OUT-data stage */
                    if (USBFS_SetupReqLen == 0) {
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
        USBFS_DevConfig      = 0;
        USBFS_DevAddr        = 0;
        USBFS_DevSleepStatus = 0;
        s_configured         = 0;

        USBFSD->DEV_ADDR = 0;
        usbfs_endp_init();
        USBFSD->INT_FG = USBFS_UIF_BUS_RST;
    } else if (intflag & USBFS_UIF_SUSPEND) {
        USBFSD->INT_FG = USBFS_UIF_SUSPEND;
        Delay_Us(10);
        if (USBFSD->MIS_ST & USBFS_UMS_SUSPEND) {
            USBFS_DevSleepStatus |= 0x02;
        } else {
            USBFS_DevSleepStatus &= ~0x02;
        }
    } else {
        USBFSD->INT_FG = intflag;
    }
}
