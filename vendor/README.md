# Vendored WCH CH32H417 EVT library

Subset of github.com/openwch/ch32h417 (EVT SDK, V1.0.x, 2025).

- `wch/Core` — core_riscv.{c,h} (PFIC, HSEM, IPC, SysTick), startup CSR helpers
- `wch/Peripheral/{inc,src}` — StdPeriph drivers (rcc, gpio, usart, dma, tim, usb, hsem, ipc, ...)
- `wch/Startup` — startup_ch32h417_v5f.S / _v3f.S
- `wch/Ld` — stock V5F/V3F linker scripts (we derive ours from these)
- `wch/Debug` — Delay_*, USART_Printf_Init
- `wch/usb_reference/` — UNBUILT reference examples cited by the plan. Do not compile.

Upstream is unmodified. Our code lives in `src/`, `core/`, `include/`.
