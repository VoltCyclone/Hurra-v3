// usb_device_hs.c — USBHSD device driver (PC-facing HID device side, High-Speed).
//
// Runs on the V5F core against the CH32H417 USBHSD controller in High-Speed
// (480 Mbps) DEVICE mode. The EP0 standard-request enumeration state machine
// serves descriptors cloned from the real downstream device: every
// GET_DESCRIPTOR is answered from the captured_descriptors_t passed to
// usb_device_init. There are no static/fallback descriptors; if capture failed
// the device does not enumerate. Interrupt endpoints (IN and OUT) are armed
// dynamically to match the cloned config, so composite devices and arbitrary
// report descriptors pass through faithfully.
//
// Register-layer differences from the USBFS sibling driver (enumeration logic is
// identical):
//   * Response encoding is inverted: USBHSD ACK=0x02, NAK=0x00, STALL=0x01.
//     Always use the USBHS_UEP_*_RES_* constants.
//   * INT_ST has no token bits. Dispatch is by direction (UDIS_EP_DIR) + EP id
//     (UDIS_EP_ID_MASK); SETUP is detected via UEP0_RX_CTRL & UEP_R_SETUP_IS.
//   * No +0x20000000 DMA alias. Per-EP UEPn_TX_DMA/UEPn_RX_DMA hold the raw
//     buffer address; the CPU uses the same address.
//   * Data toggle is driven explicitly by the ISR (hardware auto-toggle off);
//     see usbhsd_endp_init.
//   * RX length is per-EP: EP0 OUT is UEP0_RX_LEN, non-EP0 is UEPn_RX_LEN.

#include "ch32h417_port.h"      // ch32h417.h (USBHSD struct, RCC), usb bit defs
#include "debug.h"
#include "icc.h"                // dbg_stage() / DBG_V5F_DEV_INIT
#include "timebase_v5f.h"
#include "usb_hs_desc.h"        // usb_hs_synth_qualifier (HS DEVICE_QUALIFIER)
#include "hid_iface_index.h"    // hid_iface_index() — per-interface wIndex bounds
#include <string.h>

// TIM9-based delay; the vendor Delay_Us spins on the shared SysTick0 ISR and can
// be raced to a hang by V3F.
#define Delay_Us(us)  timebase_v5f_delay_us(us)

#include "usb_device.h"

/* ------------------------------------------------------------------------ */
/* Endpoint index / direction constants (ported names).                      */
/* ------------------------------------------------------------------------ */
#define DEF_UEP_IN                  0x80
#define DEF_UEP0                    0x00

/* EP0 DMA buffer size: 64 (maximum). EP0 transfers are packetized at the runtime
 * s_ep0_mps. */
#define USBD_EP0_SIZE               64

/* Setup-request packet alias over the EP0 DMA buffer (plain address; no
 * +0x20000000 alias on USBHSD). */
#define pUSBHS_SetupReqPak          ((PUSB_SETUP_REQ)USBHS_EP0_Buf)

/* ------------------------------------------------------------------------ */
/* DMA buffers — placed in the dedicated .usbdma section (uncached SRAM).   */
/* One 64-byte buffer per endpoint (EP0..EP7). EP0 is control; EP1..EP7 are */
/* armed dynamically to match the cloned device's endpoints. 8*64 = 512 B.  */
/* ------------------------------------------------------------------------ */
__attribute__((section(".usbdma"), aligned(4)))
static uint8_t USBHS_EP_Buf[USB_DEV_NUM_ENDPOINTS][64];
#define USBHS_EP0_Buf  (USBHS_EP_Buf[0])

/* ------------------------------------------------------------------------ */
/* Per-EP MMIO accessors. The USBHSD register struct names each endpoint's     */
/* control/length/DMA registers individually; these switch helpers index them  */
/* by EP number so cloned-topology arming stays data-driven without raw pointer */
/* math over the interleaved register layout.                                  */
/* ------------------------------------------------------------------------ */

/* Write the per-EP IN (TX) control byte. */
static inline void uep_tx_ctrl_set(uint8_t ep, uint8_t v)
{
    switch (ep) {
    case 0: USBHSD->UEP0_TX_CTRL = v; break;
    case 1: USBHSD->UEP1_TX_CTRL = v; break;
    case 2: USBHSD->UEP2_TX_CTRL = v; break;
    case 3: USBHSD->UEP3_TX_CTRL = v; break;
    case 4: USBHSD->UEP4_TX_CTRL = v; break;
    case 5: USBHSD->UEP5_TX_CTRL = v; break;
    case 6: USBHSD->UEP6_TX_CTRL = v; break;
    case 7: USBHSD->UEP7_TX_CTRL = v; break;
    default: break;
    }
}

static inline uint8_t uep_tx_ctrl_get(uint8_t ep)
{
    switch (ep) {
    case 0: return USBHSD->UEP0_TX_CTRL;
    case 1: return USBHSD->UEP1_TX_CTRL;
    case 2: return USBHSD->UEP2_TX_CTRL;
    case 3: return USBHSD->UEP3_TX_CTRL;
    case 4: return USBHSD->UEP4_TX_CTRL;
    case 5: return USBHSD->UEP5_TX_CTRL;
    case 6: return USBHSD->UEP6_TX_CTRL;
    case 7: return USBHSD->UEP7_TX_CTRL;
    default: return 0;
    }
}

static inline void uep_rx_ctrl_set(uint8_t ep, uint8_t v)
{
    switch (ep) {
    case 0: USBHSD->UEP0_RX_CTRL = v; break;
    case 1: USBHSD->UEP1_RX_CTRL = v; break;
    case 2: USBHSD->UEP2_RX_CTRL = v; break;
    case 3: USBHSD->UEP3_RX_CTRL = v; break;
    case 4: USBHSD->UEP4_RX_CTRL = v; break;
    case 5: USBHSD->UEP5_RX_CTRL = v; break;
    case 6: USBHSD->UEP6_RX_CTRL = v; break;
    case 7: USBHSD->UEP7_RX_CTRL = v; break;
    default: break;
    }
}

static inline uint8_t uep_rx_ctrl_get(uint8_t ep)
{
    switch (ep) {
    case 0: return USBHSD->UEP0_RX_CTRL;
    case 1: return USBHSD->UEP1_RX_CTRL;
    case 2: return USBHSD->UEP2_RX_CTRL;
    case 3: return USBHSD->UEP3_RX_CTRL;
    case 4: return USBHSD->UEP4_RX_CTRL;
    case 5: return USBHSD->UEP5_RX_CTRL;
    case 6: return USBHSD->UEP6_RX_CTRL;
    case 7: return USBHSD->UEP7_RX_CTRL;
    default: return 0;
    }
}

/* Set the per-EP IN transmit length. */
static inline void uep_tx_len_set(uint8_t ep, uint16_t v)
{
    switch (ep) {
    case 0: USBHSD->UEP0_TX_LEN = v; break;
    case 1: USBHSD->UEP1_TX_LEN = v; break;
    case 2: USBHSD->UEP2_TX_LEN = v; break;
    case 3: USBHSD->UEP3_TX_LEN = v; break;
    case 4: USBHSD->UEP4_TX_LEN = v; break;
    case 5: USBHSD->UEP5_TX_LEN = v; break;
    case 6: USBHSD->UEP6_TX_LEN = v; break;
    case 7: USBHSD->UEP7_TX_LEN = v; break;
    default: break;
    }
}

/* Read the per-EP OUT receive length (bytes the host sent in the last OUT). */
static inline uint16_t uep_rx_len_get(uint8_t ep)
{
    switch (ep) {
    case 0: return USBHSD->UEP0_RX_LEN;
    case 1: return USBHSD->UEP1_RX_LEN;
    case 2: return USBHSD->UEP2_RX_LEN;
    case 3: return USBHSD->UEP3_RX_LEN;
    case 4: return USBHSD->UEP4_RX_LEN;
    case 5: return USBHSD->UEP5_RX_LEN;
    case 6: return USBHSD->UEP6_RX_LEN;
    case 7: return USBHSD->UEP7_RX_LEN;
    default: return 0;
    }
}

/* Program the per-EP TX and RX DMA base + max length for a cloned endpoint. */
static inline void uep_dma_set(uint8_t ep, uint32_t addr)
{
    switch (ep) {
    case 0: USBHSD->UEP0_DMA = addr; break;
    case 1: USBHSD->UEP1_TX_DMA = addr; USBHSD->UEP1_RX_DMA = addr; break;
    case 2: USBHSD->UEP2_TX_DMA = addr; USBHSD->UEP2_RX_DMA = addr; break;
    case 3: USBHSD->UEP3_TX_DMA = addr; USBHSD->UEP3_RX_DMA = addr; break;
    case 4: USBHSD->UEP4_TX_DMA = addr; USBHSD->UEP4_RX_DMA = addr; break;
    case 5: USBHSD->UEP5_TX_DMA = addr; USBHSD->UEP5_RX_DMA = addr; break;
    case 6: USBHSD->UEP6_TX_DMA = addr; USBHSD->UEP6_RX_DMA = addr; break;
    case 7: USBHSD->UEP7_TX_DMA = addr; USBHSD->UEP7_RX_DMA = addr; break;
    default: break;
    }
}

static inline void uep_max_len_set(uint8_t ep, uint32_t v)
{
    switch (ep) {
    case 0: USBHSD->UEP0_MAX_LEN = v; break;
    case 1: USBHSD->UEP1_MAX_LEN = v; break;
    case 2: USBHSD->UEP2_MAX_LEN = v; break;
    case 3: USBHSD->UEP3_MAX_LEN = v; break;
    case 4: USBHSD->UEP4_MAX_LEN = v; break;
    case 5: USBHSD->UEP5_MAX_LEN = v; break;
    case 6: USBHSD->UEP6_MAX_LEN = v; break;
    case 7: USBHSD->UEP7_MAX_LEN = v; break;
    default: break;
    }
}

/* ------------------------------------------------------------------------ */
/* Enumeration / setup state.                                               */
/* ------------------------------------------------------------------------ */
static const uint8_t *pUSBHS_Descr;          /* current descriptor walk ptr */

static volatile uint8_t  USBHS_SetupReqCode;
static volatile uint8_t  USBHS_SetupReqType;
static volatile uint16_t USBHS_SetupReqValue;
static volatile uint16_t USBHS_SetupReqIndex;
static volatile uint16_t USBHS_SetupReqLen;

static volatile uint8_t  USBHS_DevConfig;     /* current configuration value */
static volatile uint8_t  USBHS_DevAddr;       /* pending address (applied @IN) */
static volatile uint8_t  USBHS_DevSleepStatus;

static volatile uint8_t  s_configured;        /* set after SET_CONFIGURATION */

/* ------------------------------------------------------------------------ */
/* Cloned-descriptor state. The device serves ONLY the descriptors captured  */
/* from the real downstream device (no static/fallback set). s_desc points    */
/* at the caller's file-static captured_descriptors_t (valid for program      */
/* lifetime). s_ep0_mps is the EP0 max-packet we advertise + packetize at.     */
/* ------------------------------------------------------------------------ */
static const captured_descriptors_t *s_desc;
static uint8_t s_ep0_mps = USBD_EP0_SIZE;     /* runtime EP0 max packet (8/16/32/64) */

/* Synthesized DEVICE_QUALIFIER, built once from the cloned device descriptor. An
 * HS device must answer GET_DESCRIPTOR(DEVICE_QUALIFIER); built only for an HS
 * clone. */
static uint8_t s_qualifier[10];

/* True when cloning a High-Speed device (s_desc->speed == USB_SPEED_HIGH). Drives
 * BASE_MODE and whether we answer DEVICE_QUALIFIER. A FS/LS clone serves its
 * captured descriptors verbatim and STALLs the qualifier. */
static bool s_is_hs;

/* Enabled-endpoint bitmaps (bit N => EP N armed in that direction). */
static uint8_t s_in_ep_mask;
static uint8_t s_out_ep_mask;

/* Pending host->device OUT data, per endpoint. The ISR latches the received   */
/* length and NAKs the EP; usb_device_poll_out drains the DMA buffer and        */
/* re-ACKs. Single-reader (the V5F relay loop), single-writer (the ISR).        */
static volatile uint8_t s_out_rx_len[USB_DEV_NUM_ENDPOINTS];
static volatile uint8_t s_out_rx_pend[USB_DEV_NUM_ENDPOINTS];

/* HID class state, per interface — a composite clone has independent idle and
 * protocol values for each HID interface (selected by the request's wIndex). */
static volatile uint8_t  USBHS_HidIdle[MAX_INTERFACES];
static volatile uint8_t  USBHS_HidProtocol[MAX_INTERFACES];

/* Captured EP0 HID SET_REPORT (host->device control write). The ISR latches the
 * SET_REPORT setup and accumulates its EP0 OUT payload here; the V5F relay loop
 * drains it and replays it onto the real device's EP0. Single-writer (this ISR),
 * single-reader (the relay). s_ep0_rpt_pending gates a COMPLETE payload ready to
 * forward and back-pressures the writer. */
#define EP0_RPT_BUF_MAX 256
static volatile uint8_t  s_ep0_rpt_buf[EP0_RPT_BUF_MAX];
static volatile uint16_t s_ep0_rpt_len;        /* bytes accumulated so far      */
static volatile uint16_t s_ep0_rpt_value;      /* wValue: reportType<<8 | id    */
static volatile uint16_t s_ep0_rpt_index;      /* wIndex: target interface      */
static volatile uint8_t  s_ep0_rpt_capturing;  /* between SETUP and OUT-complete */
static volatile uint8_t  s_ep0_rpt_pending;    /* complete payload ready to fwd  */

/* ------------------------------------------------------------------------ */
/* ISR declaration — must match the vector table in core/startup_v5f.S       */
/* (USBHS_IRQHandler at slot 56 / USBHS_IRQn).                               */
/* ------------------------------------------------------------------------ */
void USBHS_IRQHandler(void) WCH_IRQ;

/* ------------------------------------------------------------------------ */
/* usbhsd_rcc_init — bring up the USBHS device clocks. The USBHS 480M PLL is    */
/* shared with the host, so the bring-up is guarded on the PLL-ready bit and is */
/* idempotent: if the PLL is already locked, only the UTMI + USBHS peripheral   */
/* clocks are (re)enabled and a live PLL is never torn down.                    */
/* ------------------------------------------------------------------------ */
static void usbhsd_rcc_init(void)
{
    if (!(RCC->CTLR & RCC_USBHS_PLLRDY)) {
        /* Initialize USBHS 480M PLL (HSE preferred, HSI fallback). */
        RCC_USBHS_PLLCmd(DISABLE);
        RCC_USBHSPLLCLKConfig((RCC->CTLR & RCC_HSERDY) ? RCC_USBHSPLLSource_HSE
                                                       : RCC_USBHSPLLSource_HSI);
        RCC_USBHSPLLReferConfig(RCC_USBHSPLLRefer_25M);
        RCC_USBHSPLLClockSourceDivConfig(RCC_USBHSPLL_IN_Div1);
        RCC_USBHS_PLLCmd(ENABLE);
        /* BOUNDED re-lock spin (a never-locking PLL must not hang V5F). */
        for (uint32_t t = 0; t < 2000000u && !(RCC->CTLR & RCC_USBHS_PLLRDY); t++) { }
    }
    RCC_UTMIcmd(ENABLE);
    RCC_HBPeriphClockCmd(RCC_HBPeriph_USBHS, ENABLE);
}

/* ------------------------------------------------------------------------ */
/* usbhsd_endp_init — configure EP0 (control) plus every interrupt endpoint   */
/* advertised by the cloned device. Re-runnable: the bus-reset handler calls  */
/* it to re-arm all endpoints. Requires s_desc to be set.                     */
/*                                                                            */
/* USBHSD uses 16-bit UEP_TX_EN/UEP_RX_EN bitfields (bit N => EP N enabled) —  */
/* a clean replacement for USBFS's asymmetric UEPx_MOD register packing.      */
/* ------------------------------------------------------------------------ */
static void usbhsd_endp_init(void)
{
    /* Clear all endpoint-enable bits and per-EP runtime state so a re-init
     * (bus reset) starts from a known-empty map. */
    USBHSD->UEP_TX_EN = 0;
    USBHSD->UEP_RX_EN = 0;
    s_in_ep_mask  = 0;
    s_out_ep_mask = 0;
    for (uint8_t i = 0; i < USB_DEV_NUM_ENDPOINTS; i++) {
        s_out_rx_len[i]  = 0;
        s_out_rx_pend[i] = 0;
    }

    /* Data toggle is driven explicitly by the ISR, not hardware auto-toggle:
     * with UEP_*_TOG_AUTO=0xFF the first descriptor IN after SETUP went out as the
     * wrong DATAx and the host rejected every descriptor. The toggle is driven by
     * hand instead — SETUP data stage starts at DATA1, each subsequent EP0 IN
     * packet XORs DATA1, and every IN/OUT completion first clears UEP_*_DONE — so
     * auto-toggle stays off (registers left at reset 0). */

    /* EP0 must be enabled in the UEP_TX_EN/UEP_RX_EN bitfields, not just have its
     * response registers set; otherwise the controller never ACKs SETUP, no
     * TRANSFER interrupt fires, and the host loops reset->suspend. */
    USBHSD->UEP_TX_EN |= USBHS_UEP0_T_EN;
    USBHSD->UEP_RX_EN |= USBHS_UEP0_R_EN;
    USBHSD->UEP0_DMA     = (uint32_t)(uintptr_t)USBHS_EP_Buf[0];
    USBHSD->UEP0_MAX_LEN = s_ep0_mps;
    USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_ACK;
    USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_NAK;

    if (s_desc == NULL) return;

    /* Arm every interrupt IN/OUT endpoint the cloned device exposes. */
    for (uint8_t i = 0; i < s_desc->num_ifaces; i++) {
        const captured_iface_t *iface = &s_desc->ifaces[i];

        if (iface->interrupt_in_ep) {
            uint8_t ep = iface->interrupt_in_ep & 0x0F;
            if (ep >= 1 && ep < USB_DEV_NUM_ENDPOINTS) {
                uep_dma_set(ep, (uint32_t)(uintptr_t)USBHS_EP_Buf[ep]);
                uep_max_len_set(ep, 64);
                uep_tx_ctrl_set(ep, USBHS_UEP_T_RES_NAK);   /* IN idle = NAK */
                USBHSD->UEP_TX_EN |= (uint16_t)(1u << ep);
                s_in_ep_mask |= (uint8_t)(1u << ep);
            }
        }
        if (iface->interrupt_out_ep) {
            uint8_t ep = iface->interrupt_out_ep & 0x0F;
            if (ep >= 1 && ep < USB_DEV_NUM_ENDPOINTS) {
                uep_dma_set(ep, (uint32_t)(uintptr_t)USBHS_EP_Buf[ep]);
                uep_max_len_set(ep, 64);
                uep_rx_ctrl_set(ep, USBHS_UEP_R_RES_ACK);   /* OUT ready = ACK */
                USBHSD->UEP_RX_EN |= (uint16_t)(1u << ep);
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

bool usbhsd_init(const captured_descriptors_t *desc)
{
    /* The device serves ONLY the cloned descriptors. With no valid capture there
     * is nothing to present, so we refuse to enumerate (the caller treats a false
     * return as fatal). */
    if (desc == NULL || !desc->valid) return false;
    s_desc = desc;

    /* EP0 max packet: advertise + packetize at the cloned device's
     * bMaxPacketSize0 (device_desc[7]). Validate to a legal control-EP size and
     * fall back to 64 so a malformed cloned value cannot desync packetization. */
    uint8_t mps = s_desc->device_desc[7];
    s_ep0_mps = (mps == 8 || mps == 16 || mps == 32 || mps == 64) ? mps : 64;

    /* Serve descriptors verbatim at the captured device's speed; bcdUSB and
     * bInterval are not rewritten. BASE_MODE below matches s_desc->speed. */
    s_is_hs = (s_desc->speed == 2 /*USB_SPEED_HIGH*/);

    /* A High-Speed device MUST answer GET_DESCRIPTOR(DEVICE_QUALIFIER); a Full- or
     * Low-Speed device legitimately has none (and must STALL it). Synthesize the
     * qualifier only for the HS case, from the captured device descriptor. */
    if (s_is_hs) {
        usb_hs_synth_qualifier(s_desc->device_desc, s_qualifier);
    }

    usbhsd_rcc_init();

    /* Bring-up sequence from the WCH USBHS device reference: hold UD_RST_LINK with
     * the PHY un-suspended, program INT_EN + endpoints + speed while reset is
     * asserted, then drop reset by writing the final enable word. Order matters:
     * INT_EN, then endp_init, then BASE_MODE, then CONTROL last. */
    USBHSD->CONTROL = USBHS_UD_RST_LINK | USBHS_UD_PHY_SUSPENDM;

    USBHSD->INT_EN  = USBHS_UDIE_BUS_RST | USBHS_UDIE_SUSPEND |
                      USBHS_UDIE_BUS_SLEEP | USBHS_UDIE_LPM_ACT |
                      USBHS_UDIE_TRANSFER | USBHS_UDIE_LINK_RDY;

    usbhsd_endp_init();

    /* Device speed = captured device's speed: HS -> UD_SPEED_HIGH, FS/LOW ->
     * UD_SPEED_FULL (the USBHSD PHY runs FS/LS device signaling in that mode). */
    USBHSD->BASE_MODE = s_is_hs ? USBHS_UD_SPEED_HIGH : USBHS_UD_SPEED_FULL;

    /* Release link reset and enable the device: UD_DEV_EN drives the D+ pull-up,
     * UD_DMA_EN enables endpoint DMA, UD_LPM_EN per the reference, PHY out of
     * suspend. */
    USBHSD->CONTROL = USBHS_UD_DEV_EN | USBHS_UD_DMA_EN |
                      USBHS_UD_LPM_EN | USBHS_UD_PHY_SUSPENDM;

    USBHS_DevConfig      = 0;
    USBHS_DevAddr        = 0;
    USBHS_DevSleepStatus = 0;
    s_configured         = 0;

    // Dual-core IRQ routing: IRQn>31 are core-allocated via NVIC->IALLOCR with a
    // reset default of Core_ID_V3F. The USBHS ISR runs on V5F, so route USBHS_IRQn
    // (56) to V5F before enabling it; otherwise the interrupt goes to V3F (no
    // device handler) and USBHS_IRQHandler never fires.
    NVIC_SetAllocateIRQ(USBHS_IRQn, Core_ID_V5F);

    NVIC_EnableIRQ(USBHS_IRQn);
    return true;
}

void usbhsd_poll(void)
{
    /* No-op: the IRQ handler does the work. Present for API parity. */
}

bool usbhsd_send_report(uint8_t ep_num, const uint8_t *data, uint16_t len)
{
    if (ep_num == 0 || ep_num >= USB_DEV_NUM_ENDPOINTS) {
        return false;
    }

    /* Only push to an endpoint we actually armed as an IN for the cloned device. */
    if (!(s_in_ep_mask & (1u << ep_num))) {
        return false;
    }

    /* Don't arm an IN endpoint before enumeration completes (SET_CONFIGURATION).
     * A bus reset clears s_configured, closing the reset window too. */
    if (!s_configured) {
        return false;
    }

    /* TX busy if the endpoint is not currently NAK'ing (a previous report is
     * still armed and unconsumed). USBHSD NAK == 0x00, so "busy" is RES != NAK. */
    if ((uep_tx_ctrl_get(ep_num) & USBHS_UEP_T_RES_MASK) != USBHS_UEP_T_RES_NAK) {
        return false;
    }

    if (len > 64) {
        len = 64;
    }

    /* Copy into the endpoint's DMA buffer (direct address — no +0x20000000). */
    memcpy(USBHS_EP_Buf[ep_num], data, len);
    uep_tx_len_set(ep_num, len);

    /* Arm: set response to ACK, preserving the current data-toggle bits. The ISR's
     * non-EP0 IN handler XORs the toggle after each successful IN (auto-toggle is
     * OFF), so we only clear+set the response field here. */
    uep_tx_ctrl_set(ep_num,
        (uint8_t)((uep_tx_ctrl_get(ep_num) & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK));

    return true;
}

bool usbhsd_in_ep_free(uint8_t ep_num)
{
    if (ep_num == 0 || ep_num >= USB_DEV_NUM_ENDPOINTS) return false;
    if (!(s_in_ep_mask & (1u << ep_num)))               return false;
    if (!s_configured)                                  return false;
    /* Free == TX currently NAK'ing (previous report consumed). USBHSD NAK == 0. */
    return (uep_tx_ctrl_get(ep_num) & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_NAK;
}

bool usbhsd_is_configured(void)
{
    return s_configured != 0;
}

int usbhsd_poll_out(uint8_t ep_num, uint8_t **data_ptr)
{
    if (ep_num == 0 || ep_num >= USB_DEV_NUM_ENDPOINTS) return -1;
    if (!(s_out_ep_mask & (1u << ep_num)))              return -1;  /* not configured */
    if (!s_out_rx_pend[ep_num])                         return 0;   /* nothing pending */

    int n = s_out_rx_len[ep_num];
    if (data_ptr) *data_ptr = USBHS_EP_Buf[ep_num];   /* direct DMA-buffer address */

    /* Single-threaded relay loop: the caller consumes the buffer synchronously
     * before the next OUT can land (the ISR left the EP NAK'ing). Clear pending
     * and re-ACK so the EP can receive again. */
    s_out_rx_pend[ep_num] = 0;
    uep_rx_ctrl_set(ep_num,
        (uint8_t)((uep_rx_ctrl_get(ep_num) & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_ACK));
    return n;
}

/* Expose a completed EP0 HID SET_REPORT for the relay to forward. Returns the
 * payload length (>0) with the data/value/index out-params populated, or 0 if
 * none pending. */
int usbhsd_poll_ep0_report(uint8_t **data_ptr, uint16_t *wValue,
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
void usbhsd_ep0_report_done(void)
{
    s_ep0_rpt_len     = 0;
    s_ep0_rpt_pending = 0;
}

/* ======================================================================== */
/* USBHS_IRQHandler — ported standard-request enumeration state machine.    */
/*                                                                          */
/* USBHSD has NO token bits in INT_ST. Dispatch is:                          */
/*   dir = INT_ST & UDIS_EP_DIR   (nonzero => IN / device-to-host)           */
/*   ep  = INT_ST & UDIS_EP_ID_MASK                                          */
/*   SETUP = (ep==0 && dir==0 && (UEP0_RX_CTRL & UEP_R_SETUP_IS))            */
/* All handshake responses use the USBHS (inverted vs USBFS) encoding.       */
/* ======================================================================== */
void USBHS_IRQHandler(void)
{
    uint8_t  intflag, intst, errflag;
    uint16_t len;

    intflag = USBHSD->INT_FG;
    intst   = USBHSD->INT_ST;

    if (intflag & USBHS_UDIF_TRANSFER) {
        uint8_t ep  = intst & USBHS_UDIS_EP_ID_MASK;
        uint8_t dir = intst & USBHS_UDIS_EP_DIR;     /* nonzero => IN */

        if (dir) {
            /* ---- data-IN stage (device-to-host) ---- */
            if (ep == 0) {
                /* EP0 IN: clear the TX DONE bit, then continue serving a
                 * multi-packet descriptor or apply a pending SET_ADDRESS. */
                USBHSD->UEP0_TX_CTRL &= ~USBHS_UEP_T_DONE;
                if (USBHS_SetupReqLen == 0) {
                    USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
                }

                if ((USBHS_SetupReqType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD) {
                    /* Non-standard EP0 data upload — none here. */
                } else {
                    switch (USBHS_SetupReqCode) {
                    case USB_GET_DESCRIPTOR:
                        len = USBHS_SetupReqLen >= s_ep0_mps ? s_ep0_mps
                                                             : USBHS_SetupReqLen;
                        memcpy(USBHS_EP0_Buf, pUSBHS_Descr, len);
                        USBHS_SetupReqLen -= len;
                        pUSBHS_Descr      += len;
                        USBHSD->UEP0_TX_LEN  = len;
                        /* Toggle DATA1<->DATA0 for each successive IN packet, then
                         * set ACK (explicit toggle — auto-toggle is OFF). */
                        USBHSD->UEP0_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
                        USBHSD->UEP0_TX_CTRL =
                            (USBHSD->UEP0_TX_CTRL & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;
                        break;

                    case USB_SET_ADDRESS:
                        USBHSD->DEV_AD = USBHS_DevAddr;
                        break;

                    default:
                        break;
                    }
                }
            } else {
                /* Any non-EP0 IN: report consumed by host. Clear DONE, XOR the
                 * data toggle (explicit — auto-toggle is OFF), and re-NAK so the
                 * next usb_device_send_report sends fresh DATAx. */
                if (s_in_ep_mask & (1u << ep)) {
                    uint8_t v = uep_tx_ctrl_get(ep);
                    v &= ~USBHS_UEP_T_DONE;
                    v ^= USBHS_UEP_T_TOG_DATA1;
                    v = (uint8_t)((v & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK);
                    uep_tx_ctrl_set(ep, v);
                }
            }
        } else if (ep == 0 && (USBHSD->UEP0_RX_CTRL & USBHS_UEP_R_SETUP_IS)) {
            /* ---- SETUP stage ---- */
            USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
            USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_NAK;

            /* Latch the 8-byte setup packet from the EP0 DMA buffer. */
            USBHS_SetupReqType  = pUSBHS_SetupReqPak->bRequestType;
            USBHS_SetupReqCode  = pUSBHS_SetupReqPak->bRequest;
            USBHS_SetupReqLen   = pUSBHS_SetupReqPak->wLength;
            USBHS_SetupReqValue = pUSBHS_SetupReqPak->wValue;
            USBHS_SetupReqIndex = pUSBHS_SetupReqPak->wIndex;
            len     = 0;
            errflag = 0;

            if ((USBHS_SetupReqType & USB_REQ_TYP_MASK) != USB_REQ_TYP_STANDARD) {
                /* ---- HID class requests ---- */
                if ((USBHS_SetupReqType & USB_REQ_TYP_MASK) == USB_REQ_TYP_CLASS) {
                    switch (USBHS_SetupReqCode) {
                    case HID_SET_REPORT:
                        /* Begin capturing the EP0 OUT payload that follows, but
                         * only if the relay has drained the previous one (the
                         * pending flag is the back-pressure gate). */
                        if (!s_ep0_rpt_pending) {
                            s_ep0_rpt_capturing = 1;
                            s_ep0_rpt_len       = 0;
                            s_ep0_rpt_value     = USBHS_SetupReqValue;
                            s_ep0_rpt_index     = USBHS_SetupReqIndex;
                        }
                        break;
                    case HID_SET_IDLE: {
                        uint8_t ifc;
                        if (hid_iface_index(USBHS_SetupReqIndex, MAX_INTERFACES, &ifc))
                            USBHS_HidIdle[ifc] = (uint8_t)(USBHS_SetupReqValue >> 8);
                        else
                            errflag = 0xFF;
                        break;
                    }
                    case HID_SET_PROTOCOL: {
                        uint8_t ifc;
                        if (hid_iface_index(USBHS_SetupReqIndex, MAX_INTERFACES, &ifc))
                            USBHS_HidProtocol[ifc] = (uint8_t)USBHS_SetupReqValue;
                        else
                            errflag = 0xFF;
                        break;
                    }
                    case HID_GET_IDLE: {
                        uint8_t ifc;
                        if (hid_iface_index(USBHS_SetupReqIndex, MAX_INTERFACES, &ifc)) {
                            USBHS_EP0_Buf[0] = USBHS_HidIdle[ifc];
                            len = 1;
                        } else {
                            errflag = 0xFF;
                        }
                        break;
                    }
                    case HID_GET_PROTOCOL: {
                        uint8_t ifc;
                        if (hid_iface_index(USBHS_SetupReqIndex, MAX_INTERFACES, &ifc)) {
                            USBHS_EP0_Buf[0] = USBHS_HidProtocol[ifc];
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
                switch (USBHS_SetupReqCode) {
                case USB_GET_DESCRIPTOR:
                    /* Every descriptor is served from the cloned capture (s_desc);
                     * the HS-only DEVICE_QUALIFIER is synthesized; anything else
                     * is STALLed. */
                    switch ((uint8_t)(USBHS_SetupReqValue >> 8)) {
                    case USB_DESCR_TYP_DEVICE:
                        pUSBHS_Descr = s_desc->device_desc;
                        len = s_desc->device_desc[0];
                        break;

                    case USB_DESCR_TYP_CONFIG:
                        pUSBHS_Descr = s_desc->config_desc;
                        /* wTotalLength from the blob, clamped to what we captured. */
                        len = (uint16_t)s_desc->config_desc[2] |
                              ((uint16_t)s_desc->config_desc[3] << 8);
                        if (len > s_desc->config_desc_len)
                            len = s_desc->config_desc_len;
                        break;

                    case USB_DESCR_TYP_QUALIF:
                        /* A HS device MUST answer the qualifier; a FS/LS device has
                         * none and must STALL it. */
                        if (s_is_hs) {
                            pUSBHS_Descr = s_qualifier;
                            len = s_qualifier[0];
                        } else {
                            errflag = 0xFF;
                        }
                        break;

                    case USB_DESCR_TYP_HID: {
                        uint8_t hlen = 0;
                        const uint8_t *hd = find_hid_desc(
                            (uint8_t)(USBHS_SetupReqIndex & 0xFF), &hlen);
                        if (hd) { pUSBHS_Descr = hd; len = hlen; }
                        else    { errflag = 0xFF; }
                        break;
                    }

                    case USB_DESCR_TYP_REPORT: {
                        int k = find_iface((uint8_t)(USBHS_SetupReqIndex & 0xFF));
                        if (k >= 0 && s_desc->ifaces[k].has_hid_desc &&
                            s_desc->ifaces[k].hid_report_desc_len) {
                            pUSBHS_Descr = s_desc->ifaces[k].hid_report_desc;
                            len = s_desc->ifaces[k].hid_report_desc_len;
                        } else {
                            errflag = 0xFF;
                        }
                        break;
                    }

                    case USB_DESCR_TYP_STRING: {
                        uint8_t sidx = (uint8_t)(USBHS_SetupReqValue & 0xFF);
                        if (sidx == DEF_STRING_DESC_LANG) {
                            if (s_desc->langid_desc_len) {
                                pUSBHS_Descr = s_desc->langid_desc;
                                len = s_desc->langid_desc[0];
                            } else { errflag = 0xFF; }
                        } else if (sidx == DEF_STRING_DESC_OS) {
                            if (s_desc->ms_os_desc_len) {
                                pUSBHS_Descr = s_desc->ms_os_desc;
                                len = s_desc->ms_os_desc[0];
                            } else { errflag = 0xFF; }
                        } else {
                            int k = find_string(sidx);
                            if (k >= 0) {
                                pUSBHS_Descr = s_desc->string_desc[k];
                                len = s_desc->string_desc_len[k];
                            } else { errflag = 0xFF; }
                        }
                        break;
                    }

                    case USB_DESCR_TYP_BOS:
                        if (s_desc->bos_desc_len) {
                            pUSBHS_Descr = s_desc->bos_desc;
                            len = (uint16_t)s_desc->bos_desc[2] |
                                  ((uint16_t)s_desc->bos_desc[3] << 8);
                            if (len > s_desc->bos_desc_len)
                                len = s_desc->bos_desc_len;
                        } else {
                            errflag = 0xFF;
                        }
                        break;

                    default:
                        /* other-speed-config and anything else: STALL. */
                        errflag = 0xFF;
                        break;
                    }

                    /* Clamp to requested length, prime first packet. Skip when the
                     * lookup STALLed (pUSBHS_Descr may be stale/NULL). */
                    if (errflag != 0xFF) {
                        if (USBHS_SetupReqLen > len) {
                            USBHS_SetupReqLen = len;
                        }
                        len = (USBHS_SetupReqLen >= s_ep0_mps) ? s_ep0_mps
                                                               : USBHS_SetupReqLen;
                        memcpy(USBHS_EP0_Buf, pUSBHS_Descr, len);
                        pUSBHS_Descr += len;
                    }
                    break;

                case USB_SET_ADDRESS:
                    USBHS_DevAddr = (uint8_t)(USBHS_SetupReqValue & 0xFF);
                    break;

                case USB_GET_CONFIGURATION:
                    USBHS_EP0_Buf[0] = USBHS_DevConfig;
                    if (USBHS_SetupReqLen > 1) {
                        USBHS_SetupReqLen = 1;
                    }
                    break;

                case USB_SET_CONFIGURATION:
                    USBHS_DevConfig = (uint8_t)(USBHS_SetupReqValue & 0xFF);
                    /* SET_CONFIGURATION(0) de-configures the device: drop the
                     * interrupt-endpoint map (keep EP0 control armed) so a
                     * subsequent re-configure starts clean. */
                    s_configured = (USBHS_DevConfig != 0);
                    if (!s_configured) {
                        s_in_ep_mask  = 0;
                        s_out_ep_mask = 0;
                        USBHSD->UEP_TX_EN = USBHS_UEP0_T_EN;
                        USBHSD->UEP_RX_EN = USBHS_UEP0_R_EN;
                    }
                    break;

                case USB_CLEAR_FEATURE:
                    if ((USBHS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE) {
                        if ((uint8_t)(USBHS_SetupReqValue & 0xFF) == USB_REQ_FEAT_REMOTE_WAKEUP) {
                            USBHS_DevSleepStatus &= ~0x01;
                        } else {
                            errflag = 0xFF;
                        }
                    } else if ((USBHS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) {
                        if ((uint8_t)(USBHS_SetupReqValue & 0xFF) == USB_REQ_FEAT_ENDP_HALT) {
                            uint8_t e    = (uint8_t)(USBHS_SetupReqIndex & 0x0F);
                            bool    isin = (USBHS_SetupReqIndex & DEF_UEP_IN) != 0;
                            if (isin && (s_in_ep_mask & (1u << e))) {
                                /* un-stall the IN EP, return to NAK */
                                uep_tx_ctrl_set(e,
                                    (uint8_t)((uep_tx_ctrl_get(e) & ~USBHS_UEP_T_RES_MASK)
                                              | USBHS_UEP_T_RES_NAK));
                            } else if (!isin && (s_out_ep_mask & (1u << e))) {
                                uep_rx_ctrl_set(e,
                                    (uint8_t)((uep_rx_ctrl_get(e) & ~USBHS_UEP_R_RES_MASK)
                                              | USBHS_UEP_R_RES_ACK));
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
                    if ((USBHS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE) {
                        if ((uint8_t)(USBHS_SetupReqValue & 0xFF) == USB_REQ_FEAT_REMOTE_WAKEUP) {
                            if (s_desc->config_desc[7] & 0x20) {
                                USBHS_DevSleepStatus |= 0x01;
                            } else {
                                errflag = 0xFF;
                            }
                        } else {
                            errflag = 0xFF;
                        }
                    } else if ((USBHS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) {
                        if ((uint8_t)(USBHS_SetupReqValue & 0xFF) == USB_REQ_FEAT_ENDP_HALT) {
                            uint8_t e    = (uint8_t)(USBHS_SetupReqIndex & 0x0F);
                            bool    isin = (USBHS_SetupReqIndex & DEF_UEP_IN) != 0;
                            if (isin && (s_in_ep_mask & (1u << e))) {
                                uep_tx_ctrl_set(e,
                                    (uint8_t)((uep_tx_ctrl_get(e) & ~USBHS_UEP_T_RES_MASK)
                                              | USBHS_UEP_T_RES_STALL));
                            } else if (!isin && (s_out_ep_mask & (1u << e))) {
                                uep_rx_ctrl_set(e,
                                    (uint8_t)((uep_rx_ctrl_get(e) & ~USBHS_UEP_R_RES_MASK)
                                              | USBHS_UEP_R_RES_STALL));
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
                    USBHS_EP0_Buf[0] = 0x00;
                    if (USBHS_SetupReqLen > 1) {
                        USBHS_SetupReqLen = 1;
                    }
                    break;

                case USB_SET_INTERFACE:
                    break;

                case USB_GET_STATUS:
                    USBHS_EP0_Buf[0] = 0x00;
                    USBHS_EP0_Buf[1] = 0x00;
                    if ((USBHS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_DEVICE) {
                        if (USBHS_DevSleepStatus & 0x01) {
                            USBHS_EP0_Buf[0] = 0x02;
                        }
                    } else if ((USBHS_SetupReqType & USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP) {
                        uint8_t e    = (uint8_t)(USBHS_SetupReqIndex & 0x0F);
                        bool    isin = (USBHS_SetupReqIndex & DEF_UEP_IN) != 0;
                        if (isin && (s_in_ep_mask & (1u << e))) {
                            if ((uep_tx_ctrl_get(e) & USBHS_UEP_T_RES_MASK) == USBHS_UEP_T_RES_STALL) {
                                USBHS_EP0_Buf[0] = 0x01;
                            }
                        } else if (!isin && (s_out_ep_mask & (1u << e))) {
                            if ((uep_rx_ctrl_get(e) & USBHS_UEP_R_RES_MASK) == USBHS_UEP_R_RES_STALL) {
                                USBHS_EP0_Buf[0] = 0x01;
                            }
                        } else {
                            errflag = 0xFF;
                        }
                    } else {
                        errflag = 0xFF;
                    }
                    if (USBHS_SetupReqLen > 2) {
                        USBHS_SetupReqLen = 2;
                    }
                    break;

                default:
                    errflag = 0xFF;
                    break;
                }
            }

            /* Drive the EP0 response: STALL on error, else Tx/Rx the data or
             * zero-length status stage. The data stage after a SETUP always starts
             * at DATA1 — set it explicitly (auto-toggle is OFF). */
            if (errflag == 0xFF) {
                USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_STALL;
                USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_STALL;
            } else {
                if (USBHS_SetupReqType & DEF_UEP_IN) {
                    /* device-to-host: first data packet (DATA1) */
                    len = (USBHS_SetupReqLen > s_ep0_mps) ? s_ep0_mps
                                                          : USBHS_SetupReqLen;
                    USBHS_SetupReqLen   -= len;
                    USBHSD->UEP0_TX_LEN  = len;
                    USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
                } else {
                    /* host-to-device (or no-data): status / OUT-data stage (DATA1) */
                    if (USBHS_SetupReqLen == 0) {
                        USBHSD->UEP0_TX_LEN  = 0;
                        USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
                    } else {
                        USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
                    }
                }
            }
        } else if (ep == 0) {
            /* ---- EP0 OUT stage ---- (status stage of a control-write, or class
             * data download such as HID_SET_REPORT). UEP_R_SETUP_IS was not set,
             * so this is genuine OUT data, not a SETUP. Clear the RX response
             * (the reference NAKs here) before processing. */
            USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_NAK;
            if ((USBHS_SetupReqType & USB_REQ_TYP_MASK) == USB_REQ_TYP_CLASS) {
                switch (USBHS_SetupReqCode) {
                case HID_SET_REPORT:
                    /* Vendor/class config write. Append this EP0 OUT packet to
                     * the staging buffer; the relay replays the whole thing
                     * onto the real device's EP0. */
                    if (s_ep0_rpt_capturing) {
                        uint16_t pk = USBHSD->UEP0_RX_LEN;
                        if (pk > s_ep0_mps) pk = s_ep0_mps;
                        for (uint16_t b = 0; b < pk; b++) {
                            if (s_ep0_rpt_len < EP0_RPT_BUF_MAX)
                                s_ep0_rpt_buf[s_ep0_rpt_len++] = USBHS_EP0_Buf[b];
                        }
                    }
                    if (USBHS_SetupReqLen > s_ep0_mps)
                        USBHS_SetupReqLen -= s_ep0_mps;
                    else
                        USBHS_SetupReqLen = 0;
                    /* Last packet consumed: hand the payload to the relay. */
                    if (USBHS_SetupReqLen == 0 && s_ep0_rpt_capturing) {
                        s_ep0_rpt_capturing = 0;
                        s_ep0_rpt_pending   = 1;
                    }
                    break;
                default:
                    break;
                }
            }
            if (USBHS_SetupReqLen == 0) {
                USBHSD->UEP0_TX_LEN  = 0;
                USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
            } else {
                /* More OUT data to come: re-ACK EP0 RX at DATA1. */
                USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_ACK;
            }
        } else {
            /* ---- non-EP0 OUT ---- host->device data (e.g. vendor HID++). Latch
             * the received length and NAK further OUT so the DMA buffer isn't
             * overwritten before usb_device_poll_out drains it and re-ACKs. */
            if (s_out_ep_mask & (1u << ep)) {
                uint16_t n = uep_rx_len_get(ep);
                if (n > 64) n = 64;
                s_out_rx_len[ep]  = (uint8_t)n;
                s_out_rx_pend[ep] = 1;
                uint8_t v = uep_rx_ctrl_get(ep);
                v &= ~USBHS_UEP_R_DONE;
                v = (uint8_t)((v & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_NAK);
                uep_rx_ctrl_set(ep, v);
            }
        }
        USBHSD->INT_FG = USBHS_UDIF_TRANSFER;

    } else if (intflag & USBHS_UDIF_BUS_RST) {
        /* bus reset: drop address/config, re-init endpoints. */
        USBHS_DevConfig      = 0;
        USBHS_DevAddr        = 0;
        USBHS_DevSleepStatus = 0;
        s_configured         = 0;

        /* Abandon any in-flight or pending EP0 report capture. */
        s_ep0_rpt_capturing  = 0;
        s_ep0_rpt_pending    = 0;
        s_ep0_rpt_len        = 0;

        USBHSD->DEV_AD = 0;
        usbhsd_endp_init();
        USBHSD->INT_FG = USBHS_UDIF_BUS_RST;

    } else if (intflag & USBHS_UDIF_LINK_RDY) {
        /* Link handshake complete. MIS_ST & UDMS_HS_MOD confirms High-Speed was
         * negotiated (vs falling back to Full-Speed); stamp the stage oracle so
         * V3F can report whether the clone came up at HS. */
        if (USBHSD->MIS_ST & USBHS_UDMS_HS_MOD) {
            dbg_stage(DBG_V5F_DEV_INIT);   /* HS negotiated */
        }
        USBHSD->INT_FG = USBHS_UDIF_LINK_RDY;

    } else if (intflag & USBHS_UDIF_SUSPEND) {
        USBHSD->INT_FG = USBHS_UDIF_SUSPEND;
        // Do not call Delay_Us() here: this runs in ISR context on V5F and Delay_Us
        // is a non-reentrant SysTick wait shared with the relay-loop foreground. The
        // MIS_ST suspend bit is already valid when the IRQ fires, so read it now.
        if (USBHSD->MIS_ST & USBHS_UDMS_SUSPEND) {
            USBHS_DevSleepStatus |= 0x02;
        } else {
            USBHS_DevSleepStatus &= ~0x02;
        }
    } else {
        USBHSD->INT_FG = intflag;
    }
}
