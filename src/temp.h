#pragma once
#include <stdint.h>
// V3F-only internal temperature sensor (ADC1, clocked from HCLK to avoid the
// USBHS PLL). Call temp_init() once; temp_read_c() returns degrees Celsius.
void   temp_init(void);
int8_t temp_read_c(void);
