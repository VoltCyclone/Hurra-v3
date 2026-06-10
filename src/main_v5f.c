// main_v5f.c — slave: rendezvous, then echo ICC PINGs as PONGs.
//
// IPC interrupt-enable:
//   IPC_ITConfig(IPC_CH0, IPC_CH_Sta_Bit0, ENABLE) enables the ENA bit in the
//   IPC->ENA register for CH0/Bit0, which is the status bit set by V3F's
//   icc_ring_doorbell_v5f() -> IPC_SetFlagStatus(IPC_CH0, IPC_CH_Sta_Bit0).
//   Signature confirmed from vendor/wch/Peripheral/inc/ch32h417_ipc.h:
//     void IPC_ITConfig(IPC_Channel_TypeDef IPC_CH,
//                       IPC_ChannelStateBit_TypeDef TPC_Sta_Bit,
//                       FunctionalState NewState);
#include "ch32h417_port.h"
#include "icc.h"
#include "debug.h"
#include "usb_device.h"         // Phase-4 bring-up: USBFS device driver

int main(void)
{
    SystemAndCoreClockUpdate();
    Delay_Init();

    // --- Phase-4 bring-up scaffolding (build/link check only) -------------
    // Brings up the USBFS device side with static boot-mouse descriptors and,
    // once the host has configured us, emits a periodic idle mouse report on
    // EP1. There is no hardware in CI, so this only verifies that the USBFS
    // driver links and is reachable. Phase 5 rewrites main_v5f as the
    // host->device relay loop and removes this block.
    usb_device_init(NULL);                  // NULL: use static descriptors

    icc_init_v5f();                         // wait for V3F's shared-block magic
    HSEM_FastTake(HSEM_ID0);
    HSEM_ReleaseOneSem(HSEM_ID0, 0);

    // Enable IPC CH0 interrupt source (Bit0 = V3F->V5F doorbell) + NVIC.
    // IPC_ITConfig sets the corresponding bit in IPC->ENA so that the IPC
    // peripheral will assert the IPC_CH0_IRQn line when V3F sets Bit0.
    // Signature from ch32h417_ipc.h line 76:
    //   void IPC_ITConfig(IPC_Channel_TypeDef, IPC_ChannelStateBit_TypeDef, FunctionalState)
    IPC_ITConfig(IPC_CH0, IPC_CH_Sta_Bit0, ENABLE);
    NVIC_EnableIRQ(IPC_CH0_IRQn);

    for (;;) {
        icc_record_t r;
        bool got = false;
        while (icc_recv_from_v3f(&r)) {
            got = true;
            if (r.tag == ICC_TAG_PING) {
                icc_record_t p = { .tag = ICC_TAG_PONG };
                p.b[0] = r.b[0]; p.b[1] = r.b[1]; p.b[2] = r.b[2]; p.b[3] = r.b[3];
                (void)icc_send_to_v3f(&p);
            }
        }

        // Phase-4 bring-up: once enumerated, arm an idle mouse report on EP1.
        // Removed in Phase 5 when main_v5f becomes the relay loop.
        usb_device_poll();
        if (usb_device_is_configured()) {
            static const uint8_t mouse4[4] = { 0, 0, 0, 0 };  // no movement
            (void)usb_device_send_report(1, mouse4, sizeof(mouse4));
        }

        if (!got) __asm volatile("wfi");
    }
}
