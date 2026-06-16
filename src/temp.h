#pragma once
#include <stdint.h>
// V3F-only internal temperature sensor (ADC1 ch16, clocked from HCLK so it never
// touches the USBHS PLL the relay depends on). temp_init() once; temp_read_c()
// returns degrees Celsius (approx, on-die junction).
void   temp_init(void);
int8_t temp_read_c(void);
