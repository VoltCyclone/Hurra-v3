// usb_host.c — USBHS host driver (real-HID-device-facing host side).
//
// Runs on the V5F core. Implements the stable usb_host.h API against the
// CH32H417 USBHS controller acting as a High-Speed USB *host* toward the
// captured HID device (mouse/keyboard). Ported faithfully from the WCH
// reference driver:
//   vendor/wch/usb_reference/USBHS_Host_KM/Common/USB_Host/ch32h417_usbhs_host.c
//
// The WCH USBHS host IP is TOKEN/TRANSACTION based (NOT EHCI/schedules):
//   - write PID + endpoint + data-toggle into the single CONTROL register,
//     OR'd with USBHS_UH_HOST_ACTION to launch the transaction,
//   - poll INT_FLAG for USBHS_UHIF_TRANSFER (transfer-done),
//   - read the response PID from INT_ST (& USBHS_UH_T_TOKEN_MASK),
//   - classify ACK / NAK / STALL / DATA0/1.
// No QH/qTD linked lists.
//
// DMA convention (USBHS, unlike USBFS): program RX_DMA / TX_DMA with the RAW
// buffer address and read/write the buffer directly. There is NO +0x20000000
// CPU alias for USBHS (that alias is a USBFS-only quirk).
//
// Phase-5 scope: COMPILE/LINK into the V5F image + faithful transaction /
// control / enumeration sequences. Hardware verification is a later bench step.

#include "ch32h417_port.h"      // ch32h417.h (USBHSH struct, RCC), usb bit defs
#include "debug.h"              // (delays now via timebase_v5f, not vendor Delay_*)
#include "board.h"              // LED_GPIO_* — SysTick-free bench diag blink
#include "icc.h"                // dbg_stage() / DBG_STAGE_ADDR — UART-readable marker
#include "timebase_v5f.h"      // timebase_v5f_delay_us — race-free TIM9 delay
#include <string.h>

#include "usb_host.h"

// V5F host driver uses the TIM9-based delay (race-free), NOT the vendor Delay_Us
// which spins on the shared SysTick0->ISR and wedges when V3F delays concurrently
// (the "hang on PC enumerate" bug). Alias keeps the ported call sites unchanged.
#define Delay_Us(us)  timebase_v5f_delay_us(us)

// Fine-grained sub-stage markers for the usb_host_init() path (0x70..). V3F
// reads DBG_STAGE_ADDR out of shared SRAM and prints it over the command UART,
// so the value where V5F freezes names the exact stalling statement — readable
// without SWD (which the running V5F has NAKed by disabling SWJ) and without
// counting the LED blink-checkpoints by eye. See src/icc.h and main_v3f.c.
#define DBG_V5F_HOST_RCC_ENTER   0x70   // entered usbhs_rcc_init
#define DBG_V5F_HOST_PLL_WAIT    0x71   // about to spin on USBHS PLL lock
#define DBG_V5F_HOST_PLL_RDY     0x72   // PLL ready (or already up), before UTMI
#define DBG_V5F_HOST_UTMI_ON     0x73   // UTMI clock enabled
#define DBG_V5F_HOST_CLK_ON      0x74   // USBHS peripheral clock enabled (rcc done)
#define DBG_V5F_HOST_MMIO        0x75   // rcc_init returned, before USBHSH MMIO writes
#define DBG_V5F_HOST_CFG_DONE    0x76   // all USBHSH config writes survived

/* ------------------------------------------------------------------------ */
/* Constants ported from usb_host_config.h / ch32h417_usbhs_host.h.          */
/* The vendored host headers pull in the whole app stack (app_km.h etc.) so  */
/* we re-declare the handful of constants we actually need here, matching    */
/* the reference values verbatim.                                            */
/* ------------------------------------------------------------------------ */
#define ERR_SUCCESS                 0x00
#define ERR_USB_CONNECT             0x15
#define ERR_USB_DISCON              0x16
#define ERR_USB_BUF_OVER            0x17
#define ERR_USB_TRANSFER            0x20
#define ERR_USB_UNKNOWN             0xFE

#define DEF_BUS_RESET_TIMEOUT       30          // USBHS bus reset timeout
#define DEF_WAIT_USB_TRANSFER_CNT   1000        // Wait for the USB transfer to complete
#define DEF_CTRL_TRANS_TIMEOVER_CNT 60000       // Control transmission delay timing (NAK retries)

/* ------------------------------------------------------------------------ */
/* DMA buffers — placed in the dedicated .usbdma section (uncached SRAM).    */
/* Sized for High-Speed (512-byte max packet). USBHS programs RX_DMA/TX_DMA  */
/* with the raw address and the CPU reads the buffer directly (no alias).    */
/* ------------------------------------------------------------------------ */
#define USBHS_HOST_BUF_SIZE 512

__attribute__((section(".usbdma"), aligned(4))) static uint8_t USBHS_RX_Buf[USBHS_HOST_BUF_SIZE];
__attribute__((section(".usbdma"), aligned(4))) static uint8_t USBHS_TX_Buf[USBHS_HOST_BUF_SIZE];

/* ------------------------------------------------------------------------ */
/* Per-slot interrupt endpoint tables.                                       */
/* `tog` holds the data-toggle in CONTROL-register form (USBHS_UH_T_TOG_*).  */
/* ------------------------------------------------------------------------ */
typedef struct {
    uint8_t  addr;     /* device address */
    uint8_t  ep;       /* endpoint number (0..15) */
    uint8_t  maxpkt;   /* max packet size / 1 (capped at 255 here, HID ≤64) */
    uint16_t tog;      /* current data toggle (USBHS_UH_T_TOG_DATA0/DATA1) */
    uint8_t  used;
} intr_slot_t;

static intr_slot_t in_slots[MAX_INTR_EPS];
static intr_slot_t out_slots[MAX_INTR_OUT_EPS];

/* Cached link speed of the attached device (USB_SPEED_*). Drives low-speed
 * signaling: a low-speed device on this HS-capable root port needs the PHY in
 * full-speed mode (CFG.FORCE_FS) AND every token prefixed with the low-speed
 * PRE preamble (CONTROL.PRE_PID_EN). Updated on port reset / speed query. */
static uint8_t s_dev_speed = USB_SPEED_FULL;

/* ------------------------------------------------------------------------ */
/* usbhs_rcc_init — port of USBHS_RCC_Init (ENABLE path).                    */
/*                                                                           */
/* SHARED 480M PLL COORDINATION: the USBFS *device* driver (usb_device.c,    */
/* usbfs_rcc_init) ALSO brings up this same USBHS 480M PLL — both peripherals*/
/* derive their clocks from it. The reference guards the PLL bring-up with a */
/* "is the system PLL already sourced from USBHS?" check, so re-running it is */
/* idempotent: if the PLL is already enabled/locked we do NOT tear it down,  */
/* we just enable the UTMI + USBHS peripheral clocks. This makes init order  */
/* between usb_device_init() and usb_host_init() irrelevant and never        */
/* disturbs a PLL that the device side may already be running on.            */
/*                                                                           */
/* BENCH DIAG NOTE: the SysTick-free LED checkpoint/panic-blink helpers that */
/* used to live here were removed. They overlaid ~half a dozen blink trains  */
/* onto PC3, which made the LED unreadable (slow/irregular). They are        */
/* replaced by dbg_stage() markers (0x70..0x76) that V3F reads out of shared */
/* SRAM and prints over the command UART — a single, unambiguous oracle.     */
/* ------------------------------------------------------------------------ */
static void usbhs_rcc_init(void)
{
    dbg_stage(DBG_V5F_HOST_RCC_ENTER);   /* 0x70: entered usbhs_rcc_init */
    // GUARD FIX: test the USBHS PLL-ready bit, not SYSPLL_SEL. SYSPLL_SEL selects
    // what feeds SYSCLK (always !=USBHS in the 400M-HSE profile), so the old guard
    // always ran the teardown. On the host's FIRST call the PLL isn't up yet so
    // this is equivalent, but testing PLLRDY is correct and makes a re-call skip
    // tearing down a live PLL.
    if (!(RCC->CTLR & RCC_USBHS_PLLRDY)) {
        /* PLL not yet up — bring up the 480M PLL from HSE
         * (matching usb_device.c's HSE/HSI fallback for robustness). */
        RCC_USBHS_PLLCmd(DISABLE);
        RCC_USBHSPLLCLKConfig((RCC->CTLR & RCC_HSERDY) ? RCC_USBHSPLLSource_HSE
                                                       : RCC_USBHSPLLSource_HSI);
        RCC_USBHSPLLReferConfig(RCC_USBHSPLLRefer_25M);
        RCC_USBHSPLLClockSourceDivConfig(RCC_USBHSPLL_IN_Div1);
        RCC_USBHS_PLLCmd(ENABLE);
        /* BENCH DIAG: this was an UNBOUNDED spin where the V5F LED sat solid-on.
         * The stage marker (read by V3F over UART) now localizes a stall here:
         * if V5F freezes showing 0x71, the USBHS 480M PLL never asserts PLLRDY.
         * The spin is bounded so a never-locking PLL advances to 0x72 rather than
         * hanging forever (lets V3F's UART report distinguish "PLL never locked"
         * from "hung in the spin"). SysTick-free: pure RCC reads + an iteration
         * counter, so it depends on nothing the rest of V5F bring-up might break. */
        dbg_stage(DBG_V5F_HOST_PLL_WAIT);   /* 0x71: spinning on USBHS PLL lock */
        {
            uint32_t pll_wait = 0;
            while (!(RCC->CTLR & RCC_USBHS_PLLRDY)) {
                if (++pll_wait >= 2000000u) break;   /* raw spin cap, ~10s ballpark */
            }
        }
    }
    /* PLL already up (possibly by usb_device.c) — just (re)enable our clocks. */
    dbg_stage(DBG_V5F_HOST_PLL_RDY);     /* 0x72: PLL phase done, before UTMI/USBHS clk */
    RCC_UTMIcmd(ENABLE);
    dbg_stage(DBG_V5F_HOST_UTMI_ON);     /* 0x73: UTMI clk enabled */
    RCC_HBPeriphClockCmd(RCC_HBPeriph_USBHS, ENABLE);
    dbg_stage(DBG_V5F_HOST_CLK_ON);      /* 0x74: USBHS peripheral clk enabled */
}

/* ------------------------------------------------------------------------ */
/* usbhs_transact — port of USBHSH_Transact, the core token primitive.       */
/*                                                                           */
/* @param endp_pid_number  low nibble = token PID (USB_PID_IN/OUT/SETUP),    */
/*                         high nibble = endpoint number << 4.               */
/* @param endp_tog        data toggle, in CONTROL-register form              */
/*                         (USBHS_UH_T_TOG_DATA0 / DATA1).                    */
/* @param timeout         NAK-retry budget (0 = return immediately on NAK).  */
/*                                                                           */
/* Sequence per attempt:                                                     */
/*   1. CONTROL = HOST_ACTION | pid | tog | (ep<<4)  -> launches token       */
/*   2. INT_FLAG = TRANSFER (W1C) to clear the prior done flag               */
/*   3. spin DEF_WAIT_USB_TRANSFER_CNT * 1us until INT_FLAG TRANSFER set      */
/*   4. clear the token field in CONTROL                                      */
/*   5. read response PID = INT_ST & USBHS_UH_T_TOKEN_MASK and classify       */
/* Returns ERR_SUCCESS, or (PID | ERR_USB_TRANSFER) for NAK/STALL, or a      */
/* connect/disconnect/unknown error code.                                    */
/* ------------------------------------------------------------------------ */
static uint8_t usbhs_transact(uint8_t endp_pid_number, uint16_t endp_tog, uint32_t timeout)
{
    uint8_t  r, trans_retry;
    uint16_t i;
    uint8_t  endp_number = endp_pid_number & 0xf0;
    uint8_t  endp_pid    = endp_pid_number & 0x0f;

    /* Normalise the toggle to DATA0/DATA1 for IN/OUT, 0 otherwise — the
     * reference accepts a couple of legacy 0x80/0x40 toggle encodings too. */
    if (endp_pid == USB_PID_IN) {
        endp_tog = ((endp_tog & 0x80) || (endp_tog & USBHS_UH_T_TOG_DATA1))
                       ? USBHS_UH_T_TOG_DATA1 : 0;
    } else if (endp_pid == USB_PID_OUT) {
        endp_tog = ((endp_tog & 0x40) || (endp_tog & USBHS_UH_T_TOG_DATA1))
                       ? USBHS_UH_T_TOG_DATA1 : 0;
    } else {
        endp_tog = 0;
    }

    /* Low-speed device: every token must be prefixed with the PRE preamble so the
     * full-speed bus segment relays it at low speed. (FORCE_FS in CFG puts the HS
     * PHY into full-speed signaling; PRE_PID then marks each packet low-speed.) */
    uint32_t pre = (s_dev_speed == USB_SPEED_LOW) ? USBHS_UH_PRE_PID_EN : 0;

    trans_retry = 0;
    do {
        *(volatile uint32_t *)0x2017F0E0u = 0x40A20000u | trans_retry;  /* transact: loop top */
        /* Launch the transaction. */
        USBHSH->CONTROL = USBHS_UH_HOST_ACTION | pre | endp_pid | endp_tog | endp_number;

        /* Clear the transfer-done flag (W1C), then wait for it to re-assert. */
        USBHSH->INT_FLAG = USBHS_UHIF_TRANSFER;
        *(volatile uint32_t *)0x2017F0E0u = 0x40A30000u | trans_retry;  /* transact: pre INT_FLAG wait */
        for (i = DEF_WAIT_USB_TRANSFER_CNT;
             (i != 0) && ((USBHSH->INT_FLAG & USBHS_UHIF_TRANSFER) == 0);
             i--) {
            Delay_Us(1);
        }
        *(volatile uint32_t *)0x2017F0E0u = 0x40A40000u | (i & 0xFFFF);  /* transact: post-wait, i remaining */

        /* Retire the token from CONTROL. */
        USBHSH->CONTROL = (USBHSH->CONTROL & ~USBHS_UH_T_TOKEN_MASK);

        if ((USBHSH->INT_FLAG & USBHS_UHIF_TRANSFER) == 0) {
            return ERR_USB_UNKNOWN;
        }

        /* Mid-transaction connect/disconnect handling. */
        if (USBHSH->PORT_STATUS_CHG & USBHS_UHIF_PORT_CONNECT) {
            USBHSH->PORT_STATUS_CHG = USBHS_UHIF_PORT_CONNECT;
            Delay_Us(200);
            if (USBHSH->PORT_STATUS & USBHS_UHIS_PORT_CONNECT) {
                if (USBHSH->CFG & USBHS_UH_SOF_EN) {
                    return ERR_USB_CONNECT;
                }
            } else {
                return ERR_USB_DISCON;
            }
        } else if (USBHSH->INT_FLAG & USBHS_UHIF_TRANSFER) {
            /* Transfer completed — inspect the response PID. */
            r = USBHSH->INT_ST & USBHS_UH_T_TOKEN_MASK;
            if (endp_pid == USB_PID_IN) {
                /* IN: response DATAx must match the toggle we requested. */
                if ((r == USB_PID_DATA0 && endp_tog == USBHS_UH_T_TOG_DATA0) ||
                    (r == USB_PID_DATA1 && endp_tog == USBHS_UH_T_TOG_DATA1)) {
                    return ERR_SUCCESS;
                }
            } else {
                /* OUT/SETUP: device ACK / NYET means success. */
                if ((r == USB_PID_ACK) || (r == USB_PID_NYET)) {
                    return ERR_SUCCESS;
                }
            }
            if (r == USB_PID_STALL) {
                return (r | ERR_USB_TRANSFER);
            }
            if (r == USB_PID_NAK) {
                if (timeout == 0) {
                    return (r | ERR_USB_TRANSFER);
                }
                if (timeout < 0xFFFF) {
                    timeout--;
                }
                --trans_retry;   /* keep retrying without burning the cap */
            } else switch (endp_pid) {
                case USB_PID_SETUP:
                case USB_PID_OUT:
                    if (r) {
                        return (r | ERR_USB_TRANSFER);
                    }
                    break;
                case USB_PID_IN:
                    if ((r == USB_PID_DATA0) || (r == USB_PID_DATA1)) {
                        ;   /* toggle mismatch handled above; fall through retry */
                    } else if (r) {
                        return (r | ERR_USB_TRANSFER);
                    }
                    break;
                default:
                    return ERR_USB_UNKNOWN;
            }
        } else {
            /* Spurious wakeup/SOF/halt flags — clear and retry. */
            USBHSH->INT_FLAG = USBHS_UHIF_WKUP_ACT | USBHS_UHIF_RESUME_ACT |
                               USBHS_UHIF_TRANSFER | USBHS_UHIF_SOF_ACT |
                               USBHS_UHIF_TX_HALT | USBHS_UHIF_FIFO_OVER;
        }
        Delay_Us(15);
    } while (++trans_retry < 10);

    return ERR_USB_TRANSFER;
}

/* ======================================================================== */
/* Public API                                                                */
/* ======================================================================== */

bool usb_host_init(void)
{
    usbhs_rcc_init();
    dbg_stage(DBG_V5F_HOST_MMIO);        /* 0x75: rcc_init returned, before USBHSH MMIO */

    /* Reset link, hold PHY out of suspend. */
    USBHSH->CFG        = USBHS_RST_LINK | USBHS_UH_PHY_SUSPENDM;
    USBHSH->TX_DMA     = (uint32_t)(uintptr_t)USBHS_TX_Buf;
    USBHSH->RX_DMA     = (uint32_t)(uintptr_t)USBHS_RX_Buf;
    USBHSH->RX_MAX_LEN = USBHS_HOST_BUF_SIZE;
    USBHSH->PORT_CFG   = USBHS_UH_PD_EN | USBHS_UH_HOST_EN;
    USBHSH->FRAME     |= USBHS_UH_SOF_CNT_EN;
    USBHSH->CFG        = USBHS_UH_SOF_EN | USBHS_UH_DMA_EN | USBHS_UH_PHY_SUSPENDM;
    dbg_stage(DBG_V5F_HOST_CFG_DONE);    /* 0x76: all USBHSH config writes survived */

    memset(in_slots, 0, sizeof(in_slots));
    memset(out_slots, 0, sizeof(out_slots));
    return true;
}

void usb_host_power_on(void)
{
    /* On the USBHS host IP, port power + host mode are enabled via PORT_CFG
     * (PD_EN | HOST_EN) and SOF generation via CFG.SOF_EN — all done in
     * usb_host_init(). This re-asserts them so power_on() is a safe, explicit
     * call regardless of init ordering. */
    USBHSH->PORT_CFG = USBHS_UH_PD_EN | USBHS_UH_HOST_EN;
    USBHSH->CFG     |= USBHS_UH_SOF_EN;
}

bool usb_host_device_connected(void)
{
    /* A device is present if the port reports CONNECT in PORT_STATUS. We also
     * acknowledge any pending connect-change edge so it doesn't latch. */
    if (USBHSH->PORT_STATUS_CHG & USBHS_UHIF_PORT_CONNECT) {
        USBHSH->PORT_STATUS_CHG = USBHS_UHIF_PORT_CONNECT;  /* W1C the change */
    }
    return (USBHSH->PORT_STATUS & USBHS_UHIS_PORT_CONNECT) != 0;
}

void usb_host_port_reset(void)
{
    /* Port of USBHSH_ResetRootHubPort: address 0, drive bus reset, wait for
     * the reset-done change bit, then clear it. */
    uint32_t i = 0;
    USBHSH->DEV_ADDR = 0x00;
    USBHSH->PORT_CTRL |= USBHS_UH_SET_PORT_RESET;

    while (i++ < 10 * DEF_BUS_RESET_TIMEOUT) {
        if (USBHSH->PORT_STATUS_CHG & USBHS_UHIF_PORT_RESET) {
            USBHSH->PORT_STATUS_CHG = USBHS_UHIF_PORT_RESET;   /* W1C */
            break;
        }
        Delay_Us(100);
    }
    /* Re-enable SOF so the port stays active after reset. */
    USBHSH->CFG |= USBHS_UH_SOF_EN;
}

uint8_t usb_host_device_speed(void)
{
    uint16_t speed = USBHSH->PORT_STATUS & USBHS_UHIS_PORT_SPEED_MASK;
    uint8_t  spd;

    if (speed == USBHS_UHIS_PORT_LS) {
        spd = USB_SPEED_LOW;
    } else if (speed == USBHS_UHIS_PORT_HS) {
        spd = USB_SPEED_HIGH;
    } else {
        /* USBHS_UHIS_PORT_FS == 0 (and any default) -> full speed. */
        spd = USB_SPEED_FULL;
    }

    /* Cache for the transact PRE-PID decision (low-speed only). NOTE: do NOT
     * touch CFG.FORCE_FS here — toggling the PHY signaling mode on every speed
     * query regressed full/high-speed enumeration. */
    s_dev_speed = spd;
    return spd;
}

/* ------------------------------------------------------------------------ */
/* usb_host_control_transfer — port of USBHSH_CtrlTransfer.                  */
/*                                                                           */
/* Stages:                                                                    */
/*   SETUP : DEV_ADDR=addr, copy 8-byte setup -> TX_Buf, TX_LEN=8,           */
/*           usbhs_transact(SETUP, DATA0).                                    */
/*   DATA  : force CONTROL toggle to DATA1, then for each packet flip toggle. */
/*           IN  -> usbhs_transact(IN), copy RX_LEN bytes out of RX_Buf,      */
/*                  stop on a short (< maxpkt) or zero-length packet.         */
/*           OUT -> fill TX_Buf, TX_LEN, usbhs_transact(OUT).                 */
/*   STATUS: opposite-direction zero-length packet, DATA1.                    */
/*                                                                           */
/* Returns total DATA-stage bytes transferred (>=0), or negative on          */
/* STALL/error/timeout. timeout_ms gives an overall wall-clock budget.        */
/* ------------------------------------------------------------------------ */
int usb_host_control_transfer(uint8_t addr, uint8_t maxpkt,
    const usb_setup_t *setup, uint8_t *data, uint32_t timeout_ms)
{
    uint8_t  s;
    uint16_t rem_len, rx_len, rx_cnt, tx_cnt;
    int      total = 0;
    uint8_t *pbuf = data;

    if (maxpkt == 0) {
        maxpkt = 8;   /* default EP0 size */
    }

    /* Overall wall-clock deadline. Each transact() already has its own NAK
     * retry budget; this guards against a wedged device. We approximate by
     * tracking elapsed microseconds via the per-stage Delay_Us() calls is
     * impractical, so we use timeout_ms as a coarse outer guard around the
     * SETUP/DATA/STATUS stages by bounding the per-transact NAK retries. */
    uint32_t nak_budget = (timeout_ms == 0) ? DEF_CTRL_TRANS_TIMEOVER_CNT
                                            : (timeout_ms * 1000u);

    USBHSH->DEV_ADDR = addr;

    /* ---- SETUP stage (always DATA0) ---- */
    Delay_Us(100);
    memcpy(USBHS_TX_Buf, setup, sizeof(usb_setup_t));
    USBHSH->TX_LEN = sizeof(usb_setup_t);
    s = usbhs_transact(USB_PID_SETUP | 0x00, USBHS_UH_T_TOG_DATA0, nak_budget);
    if (s != ERR_SUCCESS) {
        // BENCH DIAG: stamp the SETUP-stage failure detail at 0x2017F050:
        // [7:0]=transact return code s, [15:8]=INT_FLAG, [23:16]=INT_ST,
        // [31:24]=PORT_STATUS low. Lets the oracle show WHY the first SETUP failed
        // (no response / NAK / STALL / connect-drop) instead of just ret=-1.
        *(volatile uint32_t *)0x2017F050u =
            ((uint32_t)(s & 0xFF)) |
            ((uint32_t)(USBHSH->INT_FLAG & 0xFF) << 8) |
            ((uint32_t)(USBHSH->INT_ST & 0xFF) << 16) |
            ((uint32_t)(USBHSH->PORT_STATUS & 0xFF) << 24);
        return -1;
    }

    /* DATA stage starts at DATA1 (per USB control transfer rules). */
    USBHSH->CONTROL = (USBHSH->CONTROL & ~USBHS_UH_T_TOG_MASK) | USBHS_UH_T_TOG_DATA1;
    rem_len = setup->wLength;

    if (rem_len && pbuf) {
        if (setup->bmRequestType & USB_REQ_TYP_IN) {
            /* ---- device-to-host (IN) ---- */
            while (rem_len) {
                Delay_Us(100);
                s = usbhs_transact(USB_PID_IN, USBHSH->CONTROL, nak_budget);
                if (s != ERR_SUCCESS) {
                    return -2;
                }
                USBHSH->CONTROL ^= USBHS_UH_T_TOG_DATA1;   /* flip toggle */

                rx_len = (USBHSH->RX_LEN < rem_len) ? (uint16_t)USBHSH->RX_LEN : rem_len;
                rem_len -= rx_len;
                total   += rx_len;
                for (rx_cnt = 0; rx_cnt != rx_len; rx_cnt++) {
                    *pbuf++ = USBHS_RX_Buf[rx_cnt];
                }
                /* Short packet (< maxpkt) or ZLP ends the data stage. */
                if ((USBHSH->RX_LEN == 0) || (USBHSH->RX_LEN & (maxpkt - 1))) {
                    break;
                }
            }
            USBHSH->TX_LEN = 0;   /* status stage will be OUT */
        } else {
            /* ---- host-to-device (OUT) ---- */
            while (rem_len) {
                Delay_Us(100);
                USBHSH->TX_LEN = (rem_len >= maxpkt) ? maxpkt : rem_len;
                for (tx_cnt = 0; tx_cnt != USBHSH->TX_LEN; tx_cnt++) {
                    USBHS_TX_Buf[tx_cnt] = *pbuf++;
                }
                s = usbhs_transact(USB_PID_OUT | 0x00, USBHSH->CONTROL, nak_budget);
                if (s != ERR_SUCCESS) {
                    return -3;
                }
                USBHSH->CONTROL ^= USBHS_UH_T_TOG_DATA1;

                rem_len -= USBHSH->TX_LEN;
                total   += USBHSH->TX_LEN;
            }
        }
    } else {
        /* No data stage: status direction is determined by TX_LEN below. The
         * reference relies on TX_LEN reflecting the SETUP packet (8) here, so
         * a no-data control transfer's status stage is IN. Mirror that. */
    }

    /* ---- STATUS stage (opposite direction, zero-length, DATA1) ---- */
    Delay_Us(100);
    s = usbhs_transact((USBHSH->TX_LEN) ? (USB_PID_IN | 0x00) : (USB_PID_OUT | 0x00),
                       USBHS_UH_T_TOG_DATA1, nak_budget);
    if (s != ERR_SUCCESS) {
        return -4;
    }

    if (USBHSH->TX_LEN == 0) {
        return total;   /* status was OUT (ZLP) — success */
    }
    if (USBHSH->RX_LEN == 0) {
        return total;   /* status was IN (ZLP returned) — success */
    }

    /* Status IN returned unexpected data. */
    return -5;
}

/* Synchronous simplification of v2's async control path. v2 had a
 * fire-and-forget control OUT that armed DMA and returned immediately, with
 * usb_host_control_async_busy() polled until completion. On this token-based
 * IP a control transfer is naturally synchronous, so we just run it to
 * completion here and report "never busy". Callers that loop on _async_busy()
 * therefore see it finish on the first poll. */
void usb_host_control_transfer_fire(uint8_t addr, uint8_t maxpkt,
    const usb_setup_t *setup, uint8_t *data)
{
    (void)usb_host_control_transfer(addr, maxpkt, setup, data, 2000);
}

bool usb_host_control_async_busy(void)
{
    return false;   /* synchronous: control transfers complete in-line */
}

/* ------------------------------------------------------------------------ */
/* Interrupt-IN endpoints.                                                    */
/* ------------------------------------------------------------------------ */
void usb_host_interrupt_init(uint8_t index, uint8_t addr, uint8_t ep,
    uint16_t maxpkt)
{
    if (index >= MAX_INTR_EPS) {
        return;
    }
    in_slots[index].addr   = addr;
    in_slots[index].ep     = ep & 0x0F;
    in_slots[index].maxpkt = (maxpkt > 255) ? 255 : (uint8_t)maxpkt;
    in_slots[index].tog    = USBHS_UH_T_TOG_DATA0;   /* IN data starts at DATA0 */
    in_slots[index].used   = 1;
}

/* Port of USBHSH_GetEndpData, zero-copy variant: returns a pointer directly
 * into the RX DMA buffer (valid only until the next poll on any index, since
 * all interrupt-IN endpoints share USBHS_RX_Buf). timeout 0 => no NAK retry,
 * so a device with nothing to report returns immediately. */
/* BENCH DIAG: last interrupt-IN poll outcome, so the V3F oracle can show WHY the
 * relay sees host_in=0. polls = total IN polls; last_s = last transact status
 * (0=SUCCESS, 0x2A=NAK|TRANSFER, 0x2E=STALL|TRANSFER, 0xFE=UNKNOWN/no-resp);
 * oks = successful polls; last_n = last RX byte count; last_addr/ep snapshot. */
volatile uint32_t usbh_dbg_in_polls;
volatile uint32_t usbh_dbg_in_oks;
volatile uint8_t  usbh_dbg_in_last_s;
volatile uint8_t  usbh_dbg_in_last_n;
volatile uint8_t  usbh_dbg_in_addr;
volatile uint8_t  usbh_dbg_in_ep;
/* Per-slot last transact status + last nonzero RX len, so the oracle can show
 * EACH endpoint (the composite mouse's IN EP separately from the vendor EP that
 * legitimately always NAKs). Indexed by slot. */
volatile uint8_t  usbh_dbg_slot_s[MAX_INTR_EPS];
volatile uint8_t  usbh_dbg_slot_n[MAX_INTR_EPS];

int usb_host_interrupt_poll_zerocopy(uint8_t index, uint8_t **data_ptr, uint16_t len)
{
    (void)len;
    if (index >= MAX_INTR_EPS || !in_slots[index].used) {
        return 0;
    }

    USBHSH->DEV_ADDR = in_slots[index].addr;
    uint8_t s = usbhs_transact(USB_PID_IN | (in_slots[index].ep << 4),
                               in_slots[index].tog, 0);
    usbh_dbg_in_polls++;
    usbh_dbg_in_last_s = s;
    usbh_dbg_in_addr   = in_slots[index].addr;
    usbh_dbg_in_ep     = in_slots[index].ep;
    usbh_dbg_slot_s[index] = s;
    if (s != ERR_SUCCESS) {
        return 0;   /* NAK / no data / error */
    }

    uint16_t n = (uint16_t)(USBHSH->RX_LEN & USBHS_UH_RX_LEN);
    usbh_dbg_in_oks++;
    usbh_dbg_in_last_n = (uint8_t)n;
    usbh_dbg_slot_n[index] = (uint8_t)n;
    if (data_ptr) {
        *data_ptr = USBHS_RX_Buf;
    }
    in_slots[index].tog ^= USBHS_UH_T_TOG_DATA1;   /* flip toggle for next IN */
    return (int)n;
}

int usb_host_interrupt_poll(uint8_t index, uint8_t *data, uint16_t len)
{
    uint8_t *p = NULL;
    int n = usb_host_interrupt_poll_zerocopy(index, &p, len);
    if (n <= 0 || p == NULL) {
        return n;
    }
    uint16_t copy = (n > len) ? len : (uint16_t)n;
    memcpy(data, p, copy);
    return (int)copy;
}

void usb_host_interrupt_dump_state(void)
{
    /* Diagnostic stub. Reads a couple of live host registers so the call has
     * an observable effect if a debugger is attached; otherwise a no-op. */
    volatile uint16_t port_status = USBHSH->PORT_STATUS;
    volatile uint8_t  int_flag    = USBHSH->INT_FLAG;
    (void)port_status;
    (void)int_flag;
}

/* ------------------------------------------------------------------------ */
/* Interrupt-OUT endpoints (host->device, e.g. Logitech vendor reports).      */
/* ------------------------------------------------------------------------ */
void usb_host_interrupt_out_init(uint8_t index, uint8_t addr, uint8_t ep,
    uint16_t maxpkt)
{
    if (index >= MAX_INTR_OUT_EPS) {
        return;
    }
    out_slots[index].addr   = addr;
    out_slots[index].ep     = ep & 0x0F;
    out_slots[index].maxpkt = (maxpkt > 255) ? 255 : (uint8_t)maxpkt;
    out_slots[index].tog    = USBHS_UH_T_TOG_DATA0;
    out_slots[index].used   = 1;
}

/* Port of USBHSH_SendEndpData. Synchronous: runs the OUT token to completion
 * and returns true on ACK, false on STALL/NAK-timeout/error. The "previous
 * send still in flight" case from the async v2 API cannot occur here (we block
 * until the token completes), so we never return false for that reason. */
bool usb_host_interrupt_out_send(uint8_t index, const uint8_t *data, uint16_t len)
{
    if (index >= MAX_INTR_OUT_EPS || !out_slots[index].used) {
        return false;
    }
    if (len > USBHS_HOST_BUF_SIZE) {
        len = USBHS_HOST_BUF_SIZE;
    }

    USBHSH->DEV_ADDR = out_slots[index].addr;
    memcpy(USBHS_TX_Buf, data, len);
    USBHSH->TX_LEN = len;

    uint8_t s = usbhs_transact(USB_PID_OUT | (out_slots[index].ep << 4),
                               out_slots[index].tog, DEF_CTRL_TRANS_TIMEOVER_CNT);
    if (s != ERR_SUCCESS) {
        return false;
    }
    out_slots[index].tog ^= USBHS_UH_T_TOG_DATA1;
    return true;
}
