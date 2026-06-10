#pragma once
// Umbrella header: vendored WCH device + StdPeriph + core (PFIC/HSEM/IPC).
#include "ch32h417.h"          // device: IRQn enum, bases, register structs
#include "ch32h417_conf.h"     // StdPeriph: rcc/gpio/usart/dma/tim/usb/hsem/ipc
#include "core_riscv.h"        // PFIC, HSEM, IPC, SysTick, NVIC_WakeUp_V5F
