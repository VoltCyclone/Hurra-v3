// usb_device.c — USBFS device driver (PC-facing HID device side).
//
// Runs on the V5F core. Implements the stable usb_device.h API against the
// CH32H417 USBFS controller in Full-Speed (12 Mbps) DEVICE mode. The EP0
// standard-request enumeration state machine is ported from the WCH reference
// driver (vendor/wch/usb_reference/USBFS_CompositeKM/Common/ch32h417_usbfs_device.c)
// but serves descriptors CLONED FROM THE REAL DOWNSTREAM DEVICE — every
// GET_DESCRIPTOR is answered from the captured_descriptors_t passed to
// usb_device_init (src/desc_capture.c). There are NO static/fallback
// descriptors: if capture failed the device does not enumerate. Interrupt
// endpoints (IN and OUT) are armed dynamically to match the cloned config, so
// composite devices and arbitrary report descriptors pass through faithfully.
//
// DMA-buffer convention (WCH FS): the UEPn_DMA registers are programmed with
// the RAW buffer address; CPU access to that buffer uses the same address but
// with a +0x20000000 alias (USBFSD_UEP_BUF). We follow the reference exactly.

#include "ch32h417_port.h"      // ch32h417.h (USBFSD struct, RCC), usb bit defs
#include "debug.h"
#include "timebase_v5f.h"      // timebase_v5f_delay_us — race-free TIM9 delay
#include <string.h>

// Use the TIM9-based delay, not the vendor Delay_Us (which spins on the shared
// SysTick0->ISR and can be raced to a hang by V3F).
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

/* EP0 control-endpoint buffer size. Re-homed here from the (now-deleted) static
 * descriptor header: the device clones the captured device descriptor verbatim,
 * but the EP0 DMA buffer is always sized at the 64-byte FS maximum; the value we
 * actually packetize EP0 transfers at is the runtime s_ep0_mps (see below). */
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
/* Cloned-descriptor state. The device serves ONLY the descriptors captured  */
/* from the real downstream device (no static/fallback set). s_desc points    */
/* at the caller's file-static captured_descriptors_t (valid for program      */
/* lifetime). s_ep0_mps is the EP0 max-packet we advertise + packetize at,    */
/* derived from the cloned device descriptor.                                 */
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
    // GUARD FIX: only bring up the USBHS 480M PLL if it is NOT already locked.
    // The old guard tested SYSPLL_SEL (whether the USBHS PLL feeds SYSCLK), which
    // in the 400M-HSE profile is ALWAYS false → the block always ran, needlessly
    // doing RCC_USBHS_PLLCmd(DISABLE) + re-lock on the LIVE host PLL every device
    // init. usb_host_init() already brought the PLL up before we get here, so test
    // the actual PLL-ready bit and skip the teardown when it's already up. This
    // removes the brief PLL-off window (and the unbounded re-lock spin below) from
    // the normal path — a plausible freeze site on re-enumeration.
    if (!(RCC->CTLR & RCC_USBHS_PLLRDY)) {
        /* Initialize USBHS 480M PLL */
        RCC_USBHS_PLLCmd(DISABLE);
        RCC_USBHSPLLCLKConfig((RCC->CTLR & RCC_HSERDY) ? RCC_USBHSPLLSource_HSE
                                                       : RCC_USBHSPLLSource_HSI);
        RCC_USBHSPLLReferConfig(RCC_USBHSPLLRefer_25M);
        RCC_USBHSPLLClockSourceDivConfig(RCC_USBHSPLL_IN_Div1);
        RCC_USBHS_PLLCmd(ENABLE);
        // BOUNDED re-lock spin (was unbounded — a never-locking PLL hung V5F here
        // forever). Mirror the host side's bounded wait.
        for (uint32_t t = 0; t < 2000000u && !(RCC->CTLR & RCC_USBHS_PLLRDY); t++) { }
    }
    RCC_USBFSCLKConfig(RCC_USBFSCLKSource_USBHSPLL);
    RCC_USBFS48ClockSourceDivConfig(RCC_USBFS_Div10);
    RCC_HBPeriphClockCmd(RCC_HBPeriph_OTG_FS, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOA, ENABLE);
}

/* ------------------------------------------------------------------------ */
/* usbfs_set_mod_bit — OR the TX(IN)/RX(OUT) enable bit for endpoint `ep`     */
/* into the correct UEPx_MOD register. The packing is asymmetric: EP1 & EP4   */
/* share UEP4_1_MOD, EP2 & EP3 share UEP2_3_MOD, EP5 & EP6 share UEP5_6_MOD,   */
/* EP7 lives alone in UEP7_MOD. Use |= because two endpoints can share one     */
/* register (e.g. a composite mouse on EP1-IN + keyboard on EP4-IN).           */
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

bool usb_device_init(const captured_descriptors_t *desc)
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

    /* Only push to an endpoint we actually armed as an IN for the cloned device.
     * Guards against a stale mapping after a re-enumeration with a different
     * topology. */
    if (!(s_in_ep_mask & (1u << ep_num))) {
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

/* ======================================================================== */
/* USBFS_IRQHandler — ported standard-request enumeration state machine.    */
/* ======================================================================== */
volatile uint8_t usbd_dbg_in_isr;   /* bench: 1 while inside the ISR (stuck-detect) */

void USBFS_IRQHandler(void)
{
    uint8_t  intflag, intst, errflag;
    uint16_t len;

    usbd_dbg_in_isr = 1;      /* bench: if this stays 1, V5F is wedged in the ISR */
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
             * the next usb_device_send_report sends fresh DATAx. Generalized over
             * all cloned IN endpoints (was hardcoded to EP1). */
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
                usbd_dbg_stalls++;   /* bench: device STALLed this request */
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

        USBFSD->DEV_ADDR = 0;
        usbfs_endp_init();
        USBFSD->INT_FG = USBFS_UIF_BUS_RST;
    } else if (intflag & USBFS_UIF_SUSPEND) {
        USBFSD->INT_FG = USBFS_UIF_SUSPEND;
        // CRITICAL: do NOT call Delay_Us() here. This runs in ISR context on V5F,
        // and Delay_Us is a non-reentrant SysTick blocking wait shared with the
        // relay-loop foreground (usbhs_transact). When this ISR fired during a
        // foreground Delay_Us (the host EP poll), V5F WEDGED with no trap and the
        // millis timebase frozen — the real "no reports / hang on PC plug-in" bug.
        // The reference's 10us settle is just to debounce the suspend line; the
        // MIS_ST suspend bit is already valid when the IRQ fires, so read it now.
        if (USBFSD->MIS_ST & USBFS_UMS_SUSPEND) {
            USBFS_DevSleepStatus |= 0x02;
        } else {
            USBFS_DevSleepStatus &= ~0x02;
        }
    } else {
        USBFSD->INT_FG = intflag;
    }
    usbd_dbg_in_isr = 0;      /* bench: clean ISR exit */
}
