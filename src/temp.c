#include "temp.h"
#include "ch32h417_conf.h"

// V3F-only ADC1 internal temperature sensor.
// Clocked from HCLK (RCC_ADCCLKSource_HCLK = 0x00) — entirely independent of the
// USBHS PLL that the V5F relay depends on, so this never interferes with the relay.
//
// TempSensor_Volt_To_Temper() input unit: millivolts (mV).
// Confirmed by reading vendor/wch/Peripheral/src/ch32h417_adc.c:1100 — the function
// docstring says "@param Value - Voltage Value(mv)" and the formula converts mV to
// Celsius using a factory-calibrated slope from OTP at 0x1FFFF76C.
// We convert the 12-bit raw ADC value to mV: mv = (raw * 3300) / 4096, assuming a
// 3.3 V reference and 12-bit resolution (4096 counts full-scale).

void temp_init(void)
{
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_ADC1, ENABLE);
    // Clock ADC from HCLK — does NOT touch the USBHS PLL (relay-safe).
    RCC_ADCCLKConfig(RCC_ADCCLKSource_HCLK);

    ADC_InitTypeDef adc = {0};
    ADC_DeInit(ADC1);
    adc.ADC_Mode               = ADC_Mode_Independent;
    adc.ADC_ScanConvMode       = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;   // single conversion on demand
    adc.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign           = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel       = 1;
    ADC_Init(ADC1, &adc);

    ADC_TempSensorVrefintCmd(ENABLE);       // enable the internal temp/vref block
    ADC_Cmd(ADC1, ENABLE);

    // Calibration spins are hardware-self-clearing, but guard them anyway: a
    // failed ADC must not hang V3F at boot before the display comes up (the temp
    // read is non-essential — better to ship uncalibrated than to wedge).
    uint32_t guard = 0;
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1)) { if (++guard > 100000u) return; }
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1))      { if (++guard > 200000u) return; }
}

int8_t temp_read_c(void)
{
    // Single on-demand conversion of the temp sensor channel (16) with a long
    // sample time (CyclesMode5 = mode 5, the longest available).
    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_CyclesMode5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    // Guard prevents any hang — bail with sentinel if EOC never arrives.
    uint32_t guard = 0;
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {
        if (++guard > 100000u) return -128;
    }

    int32_t raw = (int32_t)ADC_GetConversionValue(ADC1);
    // Convert raw 12-bit count to millivolts (3.3 V ref, 4096 counts full-scale).
    // TempSensor_Volt_To_Temper expects mV — verified from ch32h417_adc.c source.
    int32_t mv = (raw * 3300) / 4096;
    int32_t c  = TempSensor_Volt_To_Temper(mv);
    if (c < -128) c = -128;
    if (c >  127) c =  127;
    return (int8_t)c;
}
