# CH32H417 EVT SDK — register & boot reference (extracted)

Distilled from the official `openwch/ch32h417` EVT SDK. Every entry cites the
SDK file it came from (paths relative to the EVT repo root). This is the
authoritative register/API reference for the Hurra-v3 driver code. When in
doubt, open the cited SDK file.

> **Toolchain note:** EVT projects build `-march` ≈ `rv32imacbxw` (RV32IMAC +
> bitmanip + WCH `xw`), `-mabi=ilp32` (soft-float; the FPU is enabled in
> hardware via `mstatus` but the ABI is soft in all examples), `-Os`, newlib-nano,
> GCC 12. We match this: `-march=rv32imac -mabi=ilp32`. (`.wvproj` files.)

## 1. Dual-core boot — **V3F is the master/boot core**

V3F boots from reset and explicitly releases V5F. Not an option byte — a runtime
register write. (`EVT/EXAM/CPU/HSEM/HSEM_CoreSync/V3F/User/main.c`,
`EVT/EXAM/CPU/IPC/IPC/V3F/User/main.c`)

```c
// Fixed start addresses (EVT/EXAM/SRC/Debug/debug.h:52-59)
#define Core_V3F_StartAddr   0x00000000
#define Core_V5F_StartAddr   0x00010000

// Run-mode select (debug.h:42-48) — default builds BOTH cores
#define Run_Core   Run_Core_V3FandV5F

// V3F main(): wake V5F, then coordinate
NVIC_WakeUp_V5F(Core_V5F_StartAddr);   // core_riscv.h:539 — sets V5F PC, releases it
HSEM_ITConfig(HSEM_ID0, ENABLE);
// (optional) V3F may PWR_EnterSTOPMode and wait for V5F's HSEM release

// V5F main(): handshake back
HSEM_FastTake(HSEM_ID0);               // ch32h417_hsem.c
HSEM_ReleaseOneSem(HSEM_ID0, 0);       // wakes V3F if it slept
```

`NVIC_WakeUp_V5F` (`EVT/EXAM/SRC/Core/core_riscv.h:539`):
```c
RV_STATIC_INLINE void NVIC_WakeUp_V5F(uint32_t addr){
    addr &= ~0x3FF;            // start addr must be 1024-byte aligned
    NVIC->WAKEIP[1] = addr;    // V5F PC
    NVIC->SCTLR |= (1<<5);
}
// NVIC_WakeUp_V3F uses WAKEIP[0] (symmetric). PFIC/NVIC base 0xE000E000.
// NVIC_GetCurrentCoreID() -> 1 = V5F, 0 = V3F (SCTLR bit16). core_riscv.h:551
```

**Two images, merged.** V3F and V5F build as separate ELF/hex. The V5F (slave)
`.wvproj` has a `mergedOptions` block that produces `Merge.bin` (0xFF fill),
pulling in `../V3F/.../*_V3F.hex`; flashed at `0x08000000`. Memory maps don't
overlap (see §7). Debug via OpenOCD `wch-dual-core.cfg`.

**HSEM** — core-private peripheral at `0xE000C000`, `HSEM_Type`
(`core_riscv.h:78-96`, `:159`). API in `ch32h417_hsem.{h,c}`:
```c
ErrorStatus HSEM_FastTake(HSEM_ID_TypeDef id);                 // 1-step take
void        HSEM_ReleaseOneSem(HSEM_ID_TypeDef id, uint32_t procID);
ErrorStatus HSEM_Take(HSEM_ID_TypeDef id, uint32_t procID);
void        HSEM_ITConfig(HSEM_ID_TypeDef, FunctionalState);
void        HSEM_ClearFlag(HSEM_ID_TypeDef);
// IDs HSEM_ID0..31. Core-id consts: HSEM_Core_ID_V3F=0x0, HSEM_Core_ID_V5F=0x100
```

**IPC** — core-private at `0xE000D000`, `IPC_Type` (`core_riscv.h:99-110`,`:160`).
API in `ch32h417_ipc.{h,c}`:
```c
void     IPC_Init(IPC_InitTypeDef*);   // IPC_CH, TxCID, RxCID, TxIER, RxIER, AutoEN
void     IPC_WriteMSG(IPC_MSG_TypeDef, uint32_t);
uint32_t IPC_ReadMSG(IPC_MSG_TypeDef);
void     IPC_SetFlagStatus / IPC_ClearFlagStatus / IPC_GetFlagStatus(IPC_Channel_TypeDef, IPC_ChannelStateBit_TypeDef);
ITStatus IPC_GetITStatus(IPC_Channel_TypeDef, IPC_ChannelStateBit_TypeDef);
void     IPC_ITConfig(IPC_Channel_TypeDef, IPC_ChannelStateBit_TypeDef, FunctionalState);
// Channels IPC_CH0..3; messages IPC_MSG0..3; state bits IPC_CH_Sta_Bit0..7.
// IRQ handlers: IPC_CH0_Handler .. IPC_CH3_Handler  (NOT *_IRQHandler)
// Example: EVT/EXAM/CPU/IPC/IPC/Common/hardware.c:29-90
```

## 2. Clocks (`system_ch32h417.c` from a USB example)

- `HSE_VALUE = 25 MHz` (`ch32h417.h:23`).
- USB examples run **V5F @ 400 MHz, V3F @ 100 MHz** from HSE:
  `#define SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSE 400000000` (active profile).
- `SystemInit()` resets RCC regs then `SetSysClock()`. **V3F calls `SystemInit()`;
  V5F does NOT** (V3F already set the PLL). Both call `SystemAndCoreClockUpdate()`.
- USBFS clock (`ch32h417_usbfs_device.c:51 USBFS_RCC_Init`):
```c
RCC_USBHSPLLCLKConfig(RCC_USBHSPLLSource_HSE);
RCC_USBHSPLLReferConfig(RCC_USBHSPLLRefer_25M);
RCC_USBHSPLLClockSourceDivConfig(RCC_USBHSPLL_IN_Div1);
RCC_USBHS_PLLCmd(ENABLE);  while(!(RCC->CTLR & RCC_USBHS_PLLRDY));
RCC_USBFSCLKConfig(RCC_USBFSCLKSource_USBHSPLL);
RCC_USBFS48ClockSourceDivConfig(RCC_USBFS_Div10);  // 480/10 = 48 MHz
RCC_HBPeriphClockCmd(RCC_HBPeriph_OTG_FS, ENABLE);
```
- USBHS host clock (`ch32h417_usbhs_host.c:29 USBHS_RCC_Init`): same 480 MHz PLL,
  then `RCC_UTMIcmd(ENABLE); RCC_HBPeriphClockCmd(RCC_HBPeriph_USBHS, ENABLE);`

## 3. USBFS device (PC-facing HID)

Base `USBFS_BASE = 0x40023400` (`ch32h417.h:1676`). `USBFSD` device view
(`ch32h417.h:1807`), struct `USBFSD_TypeDef` (`ch32h417.h:1140-1193`). Flat
macros (`ch32h417_usbfs_device.h:42-56`): `USBFSD_UEP_DMA_BASE 0x40023410`,
`USBFSD_UEP_LEN_BASE 0x40023430`, `USBFSD_UEP_CTL_BASE 0x40023432`.

Key bits (`ch32h417_usb.h` USBFS block ~`:1018-1192`):
- TX_CTRL: `USBFS_UEP_T_RES_ACK 0x00`, `_NAK 0x02`, `_STALL 0x03`, `_RES_MASK 0x03`,
  `USBFS_UEP_T_TOG 0x04`, `USBFS_UEP_T_AUTO_TOG 0x08`.
- RX_CTRL: `USBFS_UEP_R_RES_ACK/_NAK/_STALL`, `USBFS_UEP_R_TOG 0x04`, `_AUTO_TOG 0x08`.
- INT_FG: `USBFS_UIF_TRANSFER 0x02`, `USBFS_UIF_BUS_RST 0x01`, `USBFS_UIF_SUSPEND 0x04`,
  `USBFS_U_TOG_OK 0x40`, `USBFS_U_IS_NAK 0x80`.
- INT_ST token: `USBFS_UIS_TOKEN_MASK 0x30` → OUT 0x00 / SOF 0x10 / IN 0x20 / SETUP 0x30;
  `USBFS_UIS_ENDP_MASK 0x0F`; `USBFS_UIS_TOG_OK 0x40`.
- BASE_CTRL: `USBFS_UC_DEV_PU_EN`, `_INT_BUSY`, `_DMA_EN`, `_RESET_SIE`, `_CLR_ALL`.
- UDEV_CTRL: `USBFS_UD_PD_DIS 0x80`, `USBFS_UD_PORT_EN 0x01`, `USBFS_UD_LOW_SPEED 0x04`.

Endpoint init (`ch32h417_usbfs_device.c:77`): set `UEPn_MOD`, `UEPn_DMA = buf`,
`UEP0_RX_CTRL = ACK`, `UEPn_TX_CTRL = NAK`. Device enable (`:99`):
`INT_EN = SUSPEND|BUS_RST|TRANSFER`, `BASE_CTRL = DEV_PU_EN|INT_BUSY|DMA_EN`,
`UDEV_CTRL = PD_DIS|PORT_EN`, `NVIC_EnableIRQ(USBFS_IRQn)`.

Arm an IN (HID report): when `(UEPn_TX_CTRL & RES_MASK)==NAK`, copy into buf,
set `UEPn_TX_LEN`, set CTRL `…|RES_ACK`. EP0 descriptor upload flips
`UEP0_TX_CTRL ^= USBFS_UEP_T_TOG`. SETUP handler at `:240`.

**DMA buffers:** 4-byte-aligned plain arrays, **no special section**. The CPU
accesses a buffer via `USBFSD_UEP_BUF(N)` which adds **`+0x20000000`**
(`ch32h417_usbfs_device.h:55`). **USBFS-only.**

## 4. USBHS host (capture the real device)

Base `USBHSH_BASE = 0x40053000 + 0x100` (`ch32h417.h:1701-1703`,`:1821`), struct
`USBHSH_TypeDef` (`ch32h417.h:1099-1131`). **Token/transaction-based**: PID +
endpoint + toggle all go into the single 32-bit `CONTROL` register (there is no
per-EP PID register on USBHS).

Key bits (`ch32h417_usb.h:859-989`):
- CONTROL: `USBHS_UH_HOST_ACTION 0x010000` (launch), `USBHS_UH_T_TOKEN_MASK 0x0F`
  (PID), `USBHS_UH_T_TOG_MASK 0x0300`, `_TOG_DATA0 0x0000`, `_TOG_DATA1 0x0100`.
  Endpoint number OR'd as `(endp<<4)`.
- PIDs (`:39-41`): `USB_PID_SETUP 0x0D`, `USB_PID_IN 0x09`, `USB_PID_OUT 0x01`;
  responses `USB_PID_ACK/NAK/STALL/NYET/DATA0/DATA1`.
- INT_FLAG: `USBHS_UHIF_TRANSFER 0x10`, `USBHS_UHIF_PORT_CONNECT 0x01`. INT_ST
  carries received PID via `& USBHS_UH_T_TOKEN_MASK`.
- CFG: `USBHS_UH_SOF_EN 0x20`, `USBHS_UD_DMA_EN`, `USBHS_UH_PHY_SUSPENDM`, `USBHS_RST_LINK`.
- PORT_CFG: `USBHS_UH_PD_EN 0x80`, `USBHS_UH_HOST_EN 0x01`. PORT_CTRL: `USBHS_UH_SET_PORT_RESET 0x01`.
- PORT_STATUS: `USBHS_UHIS_PORT_CONNECT 0x01`, speed `_PORT_SPEED_MASK 0x0600` →
  FS 0x0000 / LS 0x0200 / HS 0x0400.

Init (`ch32h417_usbhs_host.c:67`): `CFG = RST_LINK|PHY_SUSPENDM`; `TX_DMA/RX_DMA =
&buf` (**no +0x20000000**); `RX_MAX_LEN = 512`; `PORT_CFG = PD_EN|HOST_EN`;
`FRAME |= SOF_CNT_EN`; `CFG = SOF_EN|DMA_EN|PHY_SUSPENDM`.

Core transaction (`USBHSH_Transact`, `:243`):
```c
USBHSH->CONTROL = USBHS_UH_HOST_ACTION | pid | tog | (endp<<4);
USBHSH->INT_FLAG = USBHS_UHIF_TRANSFER;                 // W1C
while(timeout-- && !(USBHSH->INT_FLAG & USBHS_UHIF_TRANSFER)) Delay_Us(1);
r = USBHSH->INT_ST & USBHS_UH_T_TOKEN_MASK;             // response PID
```
Control transfer `USBHSH_CtrlTransfer` (`:392`); interrupt-IN poll
`USBHSH_GetEndpData` (`:674`): Transact(IN|endp<<4), on success `*plen =
USBHSH->RX_LEN; memcpy(buf, USBHS_RX_Buf, len); tog ^= DATA1`. Speed
`USBHSH_CheckRootHubPortSpeed` (`:143`); reset `USBHSH_ResetRootHubPort` (`:186`).
Buffers 4-byte aligned, set directly into RX/TX_DMA (`:19-20`,`:73-74`).

## 5. USART + DMA (`EVT/EXAM/USART/USART_DMA/Common/hardware.c`)

StdPeriph: `USART_Init(USARTx,&cfg)` (BaudRate→BRR, 8N1, Tx|Rx), `USART_Cmd`,
`USART_DMACmd(USARTx, USART_DMAReq_Tx|Rx, ENABLE)`. DMA peripheral addr =
`&USARTx->DATAR`. `DMA_Init(DMA1_ChannelN,&cfg)` with `DMA_DIR_PeripheralSRC`
(RX) / `_PeripheralDST` (TX), `DMA_Mode_Circular` for an RX ring, byte size,
`MemoryInc_Enable`. USART2 on HB1 bus (`RCC_HB1PeriphClockCmd`), DMA1 on HB
(`RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1,…)`). API in `ch32h417_dma.h`:
`DMA_Init/Cmd/GetFlagStatus/SetCurrDataCounter/GetCurrDataCounter`.

## 6. GPIO + timer (LED)

GPIO (`EVT/EXAM/GPIO/GPIO_Toggle`): `RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOx,…)`,
`GPIO_Init` (`GPIO_Mode_Out_PP`, `GPIO_Speed_Very_High`), `GPIO_WriteBit /
GPIO_SetBits / GPIO_ResetBits`; direct regs `GPIOx->BSHR`, `GPIOx->OUTDR`.
Timer (`EVT/EXAM/TIM/TIM_INT`): `TIM_TimeBaseInit(TIMx,{Period=ARR,Prescaler=PSC,
Up})`, `TIM_ITConfig(TIMx,TIM_IT_Update,ENABLE)`, `NVIC_EnableIRQ(TIMx_…_IRQn)`,
`TIM_Cmd`. Handler `void TIMx_..._IRQHandler(void)
__attribute__((interrupt("WCH-Interrupt-fast")))`, clear with
`TIM_ClearITPendingBit(TIMx,TIM_IT_Update)`. TIM2-5 on HB1; TIM1/8 on HB2.

## 7. Startup, linker, vectoring

Startup `EVT/EXAM/SRC/Startup/startup_ch32h417_v5f.S` / `_v3f.S`; linkers
`EVT/EXAM/SRC/Ld/V5F/Link_v5f.ld` / `V3F/Link_v3f.ld`.

- Entry `_start: j handle_reset`. `handle_reset` sets gp/sp, enables zero-wait
  flash, copies `.load`→RAM_LOAD, calls `_load_base` which copies `.highcode`
  (`.vector/.text/.rodata`) flash→**ITCM (RAM_CODE)**, copies `.data`, zeroes
  `.bss`, sets CSRs (`mstatus 0x6088` = FP+IE, `mtvec` vectored), `mret`→`main`.
  **Code runs from ITCM, not flash.**
- Vector table `.section .vector`, `_vector_base`. Core traps first
  (`NMI/HardFault/SysTick0/1_Handler`), then `IPC_CH0..3_Handler`, `HSEM_Handler`,
  then external `*_IRQHandler` incl. `USBHS_IRQHandler`, `USBFS_IRQHandler`,
  `USARTn_IRQHandler`, `TIM1_UP_IRQHandler`, `DMA1_Channeln_IRQHandler`. Handlers
  `.weak` → override by defining same name. ISR attr
  `interrupt("WCH-Interrupt-fast")`.

Memory maps:
| | V3F (master) | V5F (slave) |
|---|---|---|
| FLASH ORIGIN | `0x00000000`, 64K | `0x00010000`, 128K |
| RAM_CODE (ITCM, code) | `0x20100000`, 64K | `0x200A0000`, 128K |
| RAM (DTCM, .data/.bss) | `0x20110000+256`, 448K-256 | `0x200C0000+768`, 256K-768 |
| RAM_SHARED (.shared, NOLOAD) | `0x20178000`, 32K | `0x20178000`, 32K (same) |

PFIC API (`core_riscv.h`): `NVIC_EnableIRQ/DisableIRQ(IRQn)`,
`NVIC_SetPriority(IRQn,prio)`, `NVIC_SetAllocateIRQ(IRQn,CoreID)` (assign IRQ to a
core), `NVIC_GetCurrentCoreID()`. IRQn enum `ch32h417.h:39-169`:
`IPC_CH0_IRQn=16`, `HSEM_IRQn=28`, `USART2_IRQn=45`, `USART1_IRQn=48`,
`USBHS_IRQn=56`, `DMA1_Channel1_IRQn=57`, `USBFS_IRQn=67`, `TIM1_UP_IRQn=71`,
`TIM2_IRQn=74`.

## 8. Includes / library layout

- `ch32h417.h` — device header (IRQn enum, bases, structs); includes
  `core_riscv.h` + `system_ch32h417.h`.
- `ch32h417_conf.h` — per-project; includes all `ch32h417_<periph>.h` StdPeriph headers.
- `debug.h` — `Delay_Init/Ms/Us`, `USART_Printf_Init`, `Run_Core`/`Core_*_StartAddr`.
- Library in `EVT/EXAM/SRC/`: `Core/` (core_riscv = PFIC/HSEM/IPC/SysTick),
  `Peripheral/{inc,src}` (StdPeriph), `Startup/`, `Ld/`, `Debug/`.
  **`EVT/PUB/` is PDFs/schematics only — no source.**
