// usb_device_fs.c — USBFS device driver (PC-facing HID device side).
//
// Runs on the V5F core against the CH32H417 USBFS controller in Full-Speed
// (12 Mbps) DEVICE mode. The EP0 standard-request enumeration state machine is
// ported from the WCH reference driver but serves descriptors cloned from the
// real downstream device: every GET_DESCRIPTOR is answered from the
// captured_descriptors_t passed to usb_device_init. There are no static/fallback
// descriptors; if capture failed the device does not enumerate. Interrupt
// endpoints (IN and OUT) are armed dynamically to match the cloned config, so
// composite devices and arbitrary report descriptors pass through faithfully.
//
// DMA-buffer convention (WCH FS): the UEPn_DMA registers hold the raw buffer
// address; CPU access uses the same address through a +0x20000000 alias
// (USBFSD_UEP_BUF).

#include "ch32h417_port.h"      // ch32h417.h (USBFSD struct, RCC), usb bit defs
#include "debug.h"
#include "timebase_v5f.h"
#include "hid_iface_index.h"    // hid_iface_index() — per-interface wIndex bounds
#include <string.h>

// TIM9-based delay; the vendor Delay_Us spins on the shared SysTick0 ISR and can
// be raced to a hang by V3F.
#define Delay_Us(us)  timebase_v5f_delay_us(us)

#include "usb_device.h"

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

/* EP0 DMA buffer size: the FS maximum (64). EP0 transfers are packetized at the
 * runtime s_ep0_mps, which may be smaller. */
#define USBD_EP0_SIZE               64

/* Setup-request packet alias over EP0 DMA buffer (CPU side). */
#define pUSBFS_SetupReqPak          ((PUSB_SETUP_REQ)USBFS_EP0_Buf)

/* ------------------------------------------------------------------------ */
/* DMA buffers — placed in the dedicated .usbdma section (uncached SRAM).   */
/* One 64-byte buffer per endpoint (EP0..EP7). EP0 is control; EP1..EP7 are */
/* armed dynamically to match the cloned device's endpoints. 8*64 = 512 B.  */
/* ------------------------------------------------------------------------ */
__attribute__((section(".usbdma"), aligned(4)))
static uint8_t USBFS_EP_Buf[USB_DEV_NUM_ENDPOINTS][64];
#define USBFS_EP0_Buf  (USBFS_EP_Buf[0])

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

/* ------------------------------------------------------------------------ */
/* Cloned-descriptor state. s_desc points at the caller's captured_descriptors */
/* (valid for program lifetime). s_ep0_mps is the EP0 max-packet advertised and */
/* packetized at, derived from the cloned device descriptor.                   */
/* ------------------------------------------------------------------------ */
static const captured_descriptors_t *s_desc;
static uint8_t s_ep0_mps = USBD_EP0_SIZE;     /* runtime EP0 max packet (8/16/32/64) */

/* Enabled-endpoint bitmaps (bit N => EP N armed in that direction), built by  */
/* usbfs_endp_init from the cloned config. Used to dispatch IN/OUT completions */
/* and to validate endpoint-recipient control requests.                       */
static uint8_t s_in_ep_mask;
static uint8_t s_out_ep_mask;

/* Pending host->device OUT data, per endpoint. The ISR latches the received   */
/* length and NAKs the EP; usb_device_poll_out drains the DMA buffer and        */
/* re-ACKs. Single-reader (the V5F relay loop), single-writer (the ISR).        */
static volatile uint8_t s_out_rx_len[USB_DEV_NUM_ENDPOINTS];
static volatile uint8_t s_out_rx_pend[USB_DEV_NUM_ENDPOINTS];

/* HID class state, per interface — a composite clone has independent idle and
 * protocol values for each HID interface (selected by the request's wIndex). */
static volatile uint8_t  USBFS_HidIdle[MAX_INTERFACES];
static volatile uint8_t  USBFS_HidProtocol[MAX_INTERFACES];

/* Captured EP0 HID SET_REPORT (host->device control write). The ISR latches the
 * setup and accumulates its EP0 OUT payload here; the V5F relay loop drains it
 * and replays it onto the real device's EP0 (vendor config writes). Single-writer
 * (ISR), single-reader (relay). s_ep0_rpt_pending gates a complete payload and
 * back-pressures the writer: no new capture starts while one awaits drain, so the
 * reader never sees a half-written buffer. EP0_RPT_BUF_MAX (256) covers
 * multi-packet vendor reports; larger writes are truncated in the buffer but
 * still fully drained from the wire so the control transfer ACKs. */
#define EP0_RPT_BUF_MAX 256
static volatile uint8_t  s_ep0_rpt_buf[EP0_RPT_BUF_MAX];
static volatile uint16_t s_ep0_rpt_len;        /* bytes accumulated so far      */
static volatile uint16_t s_ep0_rpt_value;      /* wValue: reportType<<8 | id    */
static volatile uint16_t s_ep0_rpt_index;      /* wIndex: target interface      */
static volatile uint8_t  s_ep0_rpt_capturing;  /* between SETUP and OUT-complete */
static volatile uint8_t  s_ep0_rpt_pending;    /* complete payload ready to fwd  */

/* ------------------------------------------------------------------------ */
/* ISR declaration — must match the vector table in core/startup_v5f.S.     */
/* ------------------------------------------------------------------------ */
void USBFS_IRQHandler(void) WCH_IRQ;

/* ------------------------------------------------------------------------ */
/* usbfs_rcc_init — bring up the shared USBHS 480 MHz PLL, derive the 48 MHz   */
/* USBFS clock (/10), and enable the OTG_FS + GPIOA clocks. The PLL is shared   */
/* with the USBHS host controller; the bring-up is idempotent and must not tear */
/* down a live PLL.                                                             */
/* ------------------------------------------------------------------------ */
static void usbfs_rcc_init(void)
{
    // Bring up the USBHS 480M PLL only if not already locked. usb_host_init() may
    // have brought it up already; testing the PLL-ready bit avoids a needless
    // DISABLE + re-lock on the live host PLL and the resulting PLL-off window.
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
/* usbfs_set_mod_bit — OR the TX(IN)/RX(OUT) enable bit for endpoint `ep` into  */
/* the correct UEPx_MOD register. Register packing is asymmetric: EP1/EP4 share */
/* UEP4_1_MOD, EP2/EP3 share UEP2_3_MOD, EP5/EP6 share UEP5_6_MOD, EP7 owns     */
/* UEP7_MOD. |= preserves the other endpoint sharing the same register.        */
/* ------------------------------------------------------------------------ */
static void usbfs_set_mod_bit(uint8_t ep, bool is_in)
{
    switch (ep) {
    case 1: USBFSD->UEP4_1_MOD |= is_in ? USBFS_UEP1_TX_EN : USBFS_UEP1_RX_EN; break;
    case 2: USBFSD->UEP2_3_MOD |= is_in ? USBFS_UEP2_TX_EN : USBFS_UEP2_RX_EN; break;
    case 3: USBFSD->UEP2_3_MOD |= is_in ? USBFS_UEP3_TX_EN : USBFS_UEP3_RX_EN; break;
    case 4: USBFSD->UEP4_1_MOD |= is_in ? USBFS_UEP4_TX_EN : USBFS_UEP4_RX_EN; break;
    case 5: USBFSD->UEP5_6_MOD |= is_in ? USBFS_UEP5_TX_EN : USBFS_UEP5_RX_EN; break;
    case 6: USBFSD->UEP5_6_MOD |= is_in ? USBFS_UEP6_TX_EN : USBFS_UEP6_RX_EN; break;
    case 7: USBFSD->UEP7_MOD   |= is_in ? USBFS_UEP7_TX_EN : USBFS_UEP7_RX_EN; break;
    default: break;
    }
}

/* ------------------------------------------------------------------------ */
/* usbfs_endp_init — configure EP0 (control) plus every interrupt endpoint    */
/* advertised by the cloned device. Re-runnable: the bus-reset handler calls  */
/* it to re-arm all endpoints at DATA0. Requires s_desc to be set.            */
/* ------------------------------------------------------------------------ */
static void usbfs_endp_init(void)
{
    /* Clear all endpoint-enable bits and per-EP runtime state so a re-init
     * (bus reset) starts from a known-empty map. */
    USBFSD->UEP4_1_MOD = 0;
    USBFSD->UEP2_3_MOD = 0;
    USBFSD->UEP5_6_MOD = 0;
    USBFSD->UEP7_MOD   = 0;
    s_in_ep_mask  = 0;
    s_out_ep_mask = 0;
    for (uint8_t i = 0; i < USB_DEV_NUM_ENDPOINTS; i++) {
        s_out_rx_len[i]  = 0;
        s_out_rx_pend[i] = 0;
    }

    /* EP0 control endpoint is always present. */
    USBFSD->UEP0_DMA     = (uint32_t)(uintptr_t)USBFS_EP_Buf[0];
    USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_RES_ACK;
    USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;

    if (s_desc == NULL) return;

    /* Arm every interrupt IN/OUT endpoint the cloned device exposes. */
    for (uint8_t i = 0; i < s_desc->num_ifaces; i++) {
        const captured_iface_t *iface = &s_desc->ifaces[i];

        if (iface->interrupt_in_ep) {
            uint8_t ep = iface->interrupt_in_ep & 0x0F;
            if (ep >= 1 && ep < USB_DEV_NUM_ENDPOINTS) {
                USBFSD_UEP_DMA(ep)     = (uint32_t)(uintptr_t)USBFS_EP_Buf[ep];
                USBFSD_UEP_TX_CTRL(ep) = USBFS_UEP_T_RES_NAK;   /* IN idle = NAK */
                usbfs_set_mod_bit(ep, true);
                s_in_ep_mask |= (uint8_t)(1u << ep);
            }
        }
        if (iface->interrupt_out_ep) {
            uint8_t ep = iface->interrupt_out_ep & 0x0F;
            if (ep >= 1 && ep < USB_DEV_NUM_ENDPOINTS) {
                USBFSD_UEP_DMA(ep)     = (uint32_t)(uintptr_t)USBFS_EP_Buf[ep];
                USBFSD_UEP_RX_CTRL(ep) = USBFS_UEP_R_RES_ACK;   /* OUT ready = ACK */
                usbfs_set_mod_bit(ep, false);
                s_out_ep_mask |= (uint8_t)(1u << ep);
            }
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Cloned-descriptor lookup helpers (used by the GET_DESCRIPTOR handler).    */
/* ------------------------------------------------------------------------ */

/* Find the captured string slot for USB string index `idx`; -1 if absent. */
static int find_string(uint8_t idx)
{
    for (uint8_t i = 0; i < s_desc->num_strings; i++) {
        if (s_desc->string_index[i] == idx) return i;
    }
    return -1;
}

/* Find the captured interface with bInterfaceNumber == iface_num; -1 if absent. */
static int find_iface(uint8_t iface_num)
{
    for (uint8_t i = 0; i < s_desc->num_ifaces; i++) {
        if (s_desc->ifaces[i].iface_num == iface_num) return (int)i;
    }
    return -1;
}

/* Locate the HID descriptor (bDescriptorType 0x21) embedded in the cloned
 * config blob for interface `iface_num`. Walks descriptors by bLength, tracking
 * the current interface number. Returns a pointer into config_desc and sets
 * *out_len to its bLength; NULL if not found. Guards against bLength==0. */
static const uint8_t *find_hid_desc(uint8_t iface_num, uint8_t *out_len)
{
    const uint8_t *p   = s_desc->config_desc;
    const uint8_t *end = p + s_desc->config_desc_len;
    uint8_t cur_iface = 0xFF;

    while (p + 2 <= end) {
        uint8_t dlen  = p[0];
        uint8_t dtype = p[1];
        if (dlen < 2 || p + dlen > end) break;   /* malformed — stop */

        if (dtype == USB_DESCR_TYP_INTERF && dlen >= 9) {
            cur_iface = p[2];
        } else if (dtype == USB_DESCR_TYP_HID && cur_iface == iface_num) {
            *out_len = dlen;
            return p;
        }
        p += dlen;
    }
    return NULL;
}

/* ======================================================================== */
/* Public API                                                               */
/* ======================================================================== */

bool usbfsd_init(const captured_descriptors_t *desc)
{
    /* The device serves ONLY the cloned descriptors. With no valid capture there
     * is nothing to present, so we refuse to enumerate (the caller treats a false
     * return as fatal). */
    if (desc == NULL || !desc->valid) return false;
    s_desc = desc;

    /* EP0 max packet: advertise + packetize at the cloned device's
     * bMaxPacketSize0 (device_desc[7]) so the descriptor stays byte-exact and
     * self-consistent. Validate to a legal control-EP size; fall back to 64. */
    uint8_t mps = s_desc->device_desc[7];
    s_ep0_mps = (mps == 8 || mps == 16 || mps == 32 || mps == 64) ? mps : 64;

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

    // Dual-core IRQ routing: IRQn>31 are core-allocated via NVIC->IALLOCR with a
    // reset default of Core_ID_V3F. The USBFS ISR runs on V5F, so route USBFS_IRQn
    // (67) to V5F before enabling it; otherwise the interrupt goes to V3F (no
    // handler) and USBFS_IRQHandler never fires.
    NVIC_SetAllocateIRQ(USBFS_IRQn, Core_ID_V5F);

    NVIC_EnableIRQ(USBFS_IRQn);
    return true;
}

void usbfsd_poll(void)
{
    /* No-op: the IRQ handler does the work. Present for API parity. */
}

bool usbfsd_send_report(uint8_t ep_num, const uint8_t *data, uint16_t len)
{
    if (ep_num == 0 || ep_num >= USB_DEV_NUM_ENDPOINTS) {
        return false;
    }

    /* Only push to an endpoint armed as an IN for the cloned device; guards
     * against a stale mapping after re-enumeration with a different topology. */
    if (!(s_in_ep_mask & (1u << ep_num))) {
        return false;
    }

    /* Do not arm an IN endpoint before SET_CONFIGURATION: arming pre-config can
     * mis-seed the DATA0/DATA1 toggle across a bus reset and the host will not
     * poll an unconfigured device anyway. A bus reset clears s_configured. */
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

bool usbfsd_is_configured(void)
{
    return s_configured != 0;
}

int usbfsd_poll_out(uint8_t ep_num, uint8_t **data_ptr)
{
    if (ep_num == 0 || ep_num >= USB_DEV_NUM_ENDPOINTS) return -1;
    if (!(s_out_ep_mask & (1u << ep_num)))              return -1;  /* not configured */
    if (!s_out_rx_pend[ep_num])                         return 0;   /* nothing pending */

    int n = s_out_rx_len[ep_num];
    if (data_ptr) *data_ptr = USBFSD_UEP_BUF(ep_num);   /* +0x20000000 CPU alias */

    /* Single-threaded relay loop: the caller consumes the buffer synchronously
     * before the next OUT can land (the ISR left the EP NAK'ing). Clear pending
     * and re-ACK so the EP can receive again. */
    s_out_rx_pend[ep_num] = 0;
    USBFSD_UEP_RX_CTRL(ep_num) =
        (USBFSD_UEP_RX_CTRL(ep_num) & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
    return n;
}

/* Expose a completed EP0 HID SET_REPORT for the relay to forward. Returns the
 * payload length (>0) with *data/*value/*index populated, or 0 if none pending.
 * The buffer stays valid until usb_device_ep0_report_done() is called — until
 * then the ISR will not start a new capture (pending flag back-pressures it). */
int usbfsd_poll_ep0_report(uint8_t **data_ptr, uint16_t *wValue,
                               uint16_t *wIndex)
{
    if (!s_ep0_rpt_pending) return 0;
    if (data_ptr) *data_ptr = (uint8_t *)s_ep0_rpt_buf;
    if (wValue)   *wValue   = s_ep0_rpt_value;
    if (wIndex)   *wIndex   = s_ep0_rpt_index;
    return (int)s_ep0_rpt_len;
}

/* Release the EP0 report buffer after the relay has forwarded it, re-opening
 * capture for the next SET_REPORT. */
void usbfsd_ep0_report_done(void)
{
    s_ep0_rpt_len     = 0;
    s_ep0_rpt_pending = 0;
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
                        len = USBFS_SetupReqLen >= s_ep0_mps ? s_ep0_mps
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

            /* Any non-EP0 IN: report consumed by host. Re-NAK and flip toggle so
             * the next usb_device_send_report sends fresh DATAx. */
            default: {
                uint8_t ep = intst & USBFS_UIS_ENDP_MASK;
                if (ep && (s_in_ep_mask & (1u << ep))) {
                    USBFSD_UEP_TX_CTRL(ep) =
                        (USBFSD_UEP_TX_CTRL(ep) & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
                    USBFSD_UEP_TX_CTRL(ep) ^= USBFS_UEP_T_TOG;
                }
                break;
            }
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
                                /* Vendor/class config write. Append this EP0 OUT
                                 * packet to the staging buffer; the relay replays it
                                 * onto the real device's EP0. RX_LEN is this packet's
                                 * byte count (<= s_ep0_mps). Consume until SetupReqLen
                                 * reaches 0, then publish. Buffer overflow truncates
                                 * the copy but the bytes are still drained off the
                                 * wire so the toggle stays in sync and the transfer
                                 * completes. */
                                if (s_ep0_rpt_capturing) {
                                    uint16_t pk = USBFSD->RX_LEN;
                                    if (pk > s_ep0_mps) pk = s_ep0_mps;
                                    for (uint16_t b = 0; b < pk; b++) {
                                        if (s_ep0_rpt_len < EP0_RPT_BUF_MAX)
                                            s_ep0_rpt_buf[s_ep0_rpt_len++] =
                                                USBFS_EP0_Buf[b];
                                    }
                                }
                                if (USBFS_SetupReqLen > s_ep0_mps)
                                    USBFS_SetupReqLen -= s_ep0_mps;
                                else
                                    USBFS_SetupReqLen = 0;
                                /* Last packet consumed: hand the payload to the relay. */
                                if (USBFS_SetupReqLen == 0 && s_ep0_rpt_capturing) {
                                    s_ep0_rpt_capturing = 0;
                                    s_ep0_rpt_pending   = 1;
                                }
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
                } else {
                    /* More OUT data to come: re-arm EP0 RX with the flipped toggle
                     * so the next DATAx packet is accepted (multi-packet payload). */
                    USBFSD->UEP0_RX_CTRL ^= USBFS_UEP_R_TOG;
                    USBFSD->UEP0_RX_CTRL =
                        (USBFSD->UEP0_RX_CTRL & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_ACK;
                }
                break;

            /* Any non-EP0 OUT: host->device data (e.g. vendor HID++). Latch the
             * received length and NAK further OUT so the DMA buffer isn't
             * overwritten before usb_device_poll_out drains it and re-ACKs. */
            default: {
                uint8_t ep = intst & USBFS_UIS_ENDP_MASK;
                if (ep && (s_out_ep_mask & (1u << ep))) {
                    uint16_t n = USBFSD->RX_LEN;
                    if (n > 64) n = 64;
                    s_out_rx_len[ep]  = (uint8_t)n;
                    s_out_rx_pend[ep] = 1;
                    USBFSD_UEP_RX_CTRL(ep) =
                        (USBFSD_UEP_RX_CTRL(ep) & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_NAK;
                }
                break;
            }
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
            len     = 0;
            errflag = 0;

            if ((USBFS_SetupReqType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD) {
                /* ---- HID class requests ---- */
                if ((USBFS_SetupReqType & USB_REQ_TYP_MASK) == USB_REQ_TYP_CLASS) {
                    switch (USBFS_SetupReqCode) {
                    case HID_SET_REPORT:
                        /* Begin capturing the EP0 OUT payload, but only if the relay
                         * has drained the previous one (pending flag is the
                         * back-pressure gate). If still pending the bytes are dropped
                         * from the staging buffer but the transfer is still ACKed
                         * below, so the host's control write completes. */
                        if (!s_ep0_rpt_pending) {
                            s_ep0_rpt_capturing = 1;
                            s_ep0_rpt_len       = 0;
                            s_ep0_rpt_value     = USBFS_SetupReqValue;
                            s_ep0_rpt_index     = USBFS_SetupReqIndex;
                        }
                        break;
                    case HID_SET_IDLE: {
                        uint8_t ifc;
                        if (hid_iface_index(USBFS_SetupReqIndex, MAX_INTERFACES, &ifc))
                            USBFS_HidIdle[ifc] = (uint8_t)(USBFS_SetupReqValue >> 8);
                        else
                            errflag = 0xFF;
                        break;
                    }
                    case HID_SET_PROTOCOL: {
                        uint8_t ifc;
                        if (hid_iface_index(USBFS_SetupReqIndex, MAX_INTERFACES, &ifc))
                            USBFS_HidProtocol[ifc] = (uint8_t)USBFS_SetupReqValue;
                        else
                            errflag = 0xFF;
                        break;
                    }
                    case HID_GET_IDLE: {
                        uint8_t ifc;
                        if (hid_iface_index(USBFS_SetupReqIndex, MAX_INTERFACES, &ifc)) {
                            USBFS_EP0_Buf[0] = USBFS_HidIdle[ifc];
                            len = 1;
                        } else {
                            errflag = 0xFF;
                        }
                        break;
                    }
                    case HID_GET_PROTOCOL: {
                        uint8_t ifc;
                        if (hid_iface_index(USBFS_SetupReqIndex, MAX_INTERFACES, &ifc)) {
                            USBFS_EP0_Buf[0] = USBFS_HidProtocol[ifc];
                            len = 1;
                        } else {
                            errflag = 0xFF;
                        }
                        break;
                    }
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
                    /* Every descriptor is served from the cloned capture (s_desc);
                     * any descriptor we don't have is STALLed. */
                    switch ((uint8_t)(USBFS_SetupReqValue >> 8)) {
                    case USB_DESCR_TYP_DEVICE:
                        pUSBFS_Descr = s_desc->device_desc;
                        len = s_desc->device_desc[0];
                        break;

                    case USB_DESCR_TYP_CONFIG:
                        pUSBFS_Descr = s_desc->config_desc;
                        /* wTotalLength from the blob, clamped to what we captured. */
                        len = (uint16_t)s_desc->config_desc[2] |
                              ((uint16_t)s_desc->config_desc[3] << 8);
                        if (len > s_desc->config_desc_len)
                            len = s_desc->config_desc_len;
                        break;

                    case USB_DESCR_TYP_HID: {
                        /* The HID descriptor is embedded in the config blob; find
                         * the one for the requested interface (wIndex). */
                        uint8_t hlen = 0;
                        const uint8_t *hd = find_hid_desc(
                            (uint8_t)(USBFS_SetupReqIndex & 0xFF), &hlen);
                        if (hd) { pUSBFS_Descr = hd; len = hlen; }
                        else    { errflag = 0xFF; }
                        break;
                    }

                    case USB_DESCR_TYP_REPORT: {
                        /* HID report descriptor for the requested interface. */
                        int k = find_iface((uint8_t)(USBFS_SetupReqIndex & 0xFF));
                        if (k >= 0 && s_desc->ifaces[k].has_hid_desc &&
                            s_desc->ifaces[k].hid_report_desc_len) {
                            pUSBFS_Descr = s_desc->ifaces[k].hid_report_desc;
                            len = s_desc->ifaces[k].hid_report_desc_len;
                        } else {
                            errflag = 0xFF;
                        }
                        break;
                    }

                    case USB_DESCR_TYP_STRING: {
                        uint8_t sidx = (uint8_t)(USBFS_SetupReqValue & 0xFF);
                        if (sidx == DEF_STRING_DESC_LANG) {
                            if (s_desc->langid_desc_len) {
                                pUSBFS_Descr = s_desc->langid_desc;
                                len = s_desc->langid_desc[0];
                            } else { errflag = 0xFF; }
                        } else if (sidx == DEF_STRING_DESC_OS) {
                            if (s_desc->ms_os_desc_len) {
                                pUSBFS_Descr = s_desc->ms_os_desc;
                                len = s_desc->ms_os_desc[0];
                            } else { errflag = 0xFF; }
                        } else {
                            int k = find_string(sidx);
                            if (k >= 0) {
                                pUSBFS_Descr = s_desc->string_desc[k];
                                len = s_desc->string_desc_len[k];
                            } else { errflag = 0xFF; }
                        }
                        break;
                    }

                    case USB_DESCR_TYP_BOS:
                        if (s_desc->bos_desc_len) {
                            pUSBFS_Descr = s_desc->bos_desc;
                            len = (uint16_t)s_desc->bos_desc[2] |
                                  ((uint16_t)s_desc->bos_desc[3] << 8);
                            if (len > s_desc->bos_desc_len)
                                len = s_desc->bos_desc_len;
                        } else {
                            errflag = 0xFF;
                        }
                        break;

                    default:
                        /* device_qualifier (FS-only) and anything else: STALL. */
                        errflag = 0xFF;
                        break;
                    }

                    /* Clamp to requested length, prime first packet. Skip when the
                     * lookup STALLed (pUSBFS_Descr may be stale/NULL). */
                    if (errflag != 0xFF) {
                        if (USBFS_SetupReqLen > len) {
                            USBFS_SetupReqLen = len;
                        }
                        len = (USBFS_SetupReqLen >= s_ep0_mps) ? s_ep0_mps
                                                               : USBFS_SetupReqLen;
                        memcpy(USBFS_EP0_Buf, pUSBFS_Descr, len);
                        pUSBFS_Descr += len;
                    }
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
                    /* SET_CONFIGURATION(0) de-configures the device: drop the
                     * endpoint map so a subsequent re-configure starts clean. */
                    s_configured = (USBFS_DevConfig != 0);
                    if (!s_configured) {
                        s_in_ep_mask  = 0;
                        s_out_ep_mask = 0;
                        USBFSD->UEP4_1_MOD = 0;
                        USBFSD->UEP2_3_MOD = 0;
                        USBFSD->UEP5_6_MOD = 0;
                        USBFSD->UEP7_MOD   = 0;
                    }
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
                            uint8_t ep   = (uint8_t)(USBFS_SetupReqIndex & 0x0F);
                            bool    isin = (USBFS_SetupReqIndex & DEF_UEP_IN) != 0;
                            if (isin && (s_in_ep_mask & (1u << ep))) {
                                /* un-stall the IN EP, return to NAK (toggle reset) */
                                USBFSD_UEP_TX_CTRL(ep) =
                                    (USBFSD_UEP_TX_CTRL(ep) & ~(USBFS_UEP_T_RES_MASK | USBFS_UEP_T_TOG))
                                    | USBFS_UEP_T_RES_NAK;
                            } else if (!isin && (s_out_ep_mask & (1u << ep))) {
                                USBFSD_UEP_RX_CTRL(ep) =
                                    (USBFSD_UEP_RX_CTRL(ep) & ~(USBFS_UEP_R_RES_MASK | USBFS_UEP_R_TOG))
                                    | USBFS_UEP_R_RES_ACK;
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
                            if (s_desc->config_desc[7] & 0x20) {
                                USBFS_DevSleepStatus |= 0x01;
                            } else {
                                errflag = 0xFF;
                            }
                        } else {
                            errflag = 0xFF;
                        }
                    } else if ((USBFS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) {
                        if ((uint8_t)(USBFS_SetupReqValue & 0xFF) == USB_REQ_FEAT_ENDP_HALT) {
                            uint8_t ep   = (uint8_t)(USBFS_SetupReqIndex & 0x0F);
                            bool    isin = (USBFS_SetupReqIndex & DEF_UEP_IN) != 0;
                            if (isin && (s_in_ep_mask & (1u << ep))) {
                                USBFSD_UEP_TX_CTRL(ep) =
                                    (USBFSD_UEP_TX_CTRL(ep) & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_STALL;
                            } else if (!isin && (s_out_ep_mask & (1u << ep))) {
                                USBFSD_UEP_RX_CTRL(ep) =
                                    (USBFSD_UEP_RX_CTRL(ep) & ~USBFS_UEP_R_RES_MASK) | USBFS_UEP_R_RES_STALL;
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
                        uint8_t ep   = (uint8_t)(USBFS_SetupReqIndex & 0x0F);
                        bool    isin = (USBFS_SetupReqIndex & DEF_UEP_IN) != 0;
                        if (isin && (s_in_ep_mask & (1u << ep))) {
                            if ((USBFSD_UEP_TX_CTRL(ep) & USBFS_UEP_T_RES_MASK) == USBFS_UEP_T_RES_STALL) {
                                USBFS_EP0_Buf[0] = 0x01;
                            }
                        } else if (!isin && (s_out_ep_mask & (1u << ep))) {
                            if ((USBFSD_UEP_RX_CTRL(ep) & USBFS_UEP_R_RES_MASK) == USBFS_UEP_R_RES_STALL) {
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
                USBFSD->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_STALL;
                USBFSD->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_STALL;
            } else {
                if (USBFS_SetupReqType & DEF_UEP_IN) {
                    /* device-to-host: first data packet (DATA1) */
                    len = (USBFS_SetupReqLen > s_ep0_mps) ? s_ep0_mps
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

        /* Abandon any in-flight or pending EP0 report capture: a re-enumeration
         * invalidates it, and the relay must not replay stale config bytes. */
        s_ep0_rpt_capturing  = 0;
        s_ep0_rpt_pending    = 0;
        s_ep0_rpt_len        = 0;

        USBFSD->DEV_ADDR = 0;
        usbfs_endp_init();
        USBFSD->INT_FG = USBFS_UIF_BUS_RST;
    } else if (intflag & USBFS_UIF_SUSPEND) {
        USBFSD->INT_FG = USBFS_UIF_SUSPEND;
        // Do not call Delay_Us() here: this runs in ISR context on V5F and Delay_Us
        // is a non-reentrant SysTick wait shared with the relay-loop foreground;
        // firing during a foreground Delay_Us wedges V5F. The MIS_ST suspend bit is
        // already valid when the IRQ fires, so read it directly.
        if (USBFSD->MIS_ST & USBFS_UMS_SUSPEND) {
            USBFS_DevSleepStatus |= 0x02;
        } else {
            USBFS_DevSleepStatus &= ~0x02;
        }
    } else {
        USBFSD->INT_FG = intflag;
    }
}
