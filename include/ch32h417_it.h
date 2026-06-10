/********************************************************************************
 * ch32h417_it.h — interrupt-handler header (Hurra-v3 canonical copy).
 *
 * The vendored StdPeriph ch32h417_conf.h pulls this in. In the WCH EVT examples
 * it lives in each project's User/ dir and only chains to debug.h. We keep our
 * own minimal copy on the include path so our umbrella header resolves without
 * depending on any vendor example's User/ folder. Our actual ISRs are defined
 * in our driver .c files (led.c, uart.c, icc.c, usb_*.c), not here.
 ********************************************************************************/
#ifndef __CH32H417_IT_H
#define __CH32H417_IT_H

#include "debug.h"

#endif
