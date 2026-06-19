#include "temp.h"
#include "ch32h417_conf.h"

// V3F-only ADC1 internal temperature sensor, clocked from HCLK
// (RCC_ADCCLKSource_HCLK) so it stays independent of the USBHS PLL the relay
// depends on.
//
// TempSensor_Volt_To_Temper() takes millivolts and applies a factory-calibrated
// slope from OTP. Raw 12-bit counts convert to mV as (raw * 3300) / 4096
// (3.3 V reference, 4096 counts full-scale).

void temp_init(void)
{
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_ADC1, ENABLE);
    // Clock ADC from HCLK; leaves the USBHS PLL untouched.
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

    // Guard the calibration spins so a failed ADC cannot hang boot; the temp
    // read is non-essential.
    uint32_t guard = 0;
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1)) { if (++guard > 100000u) return; }
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1))      { if (++guard > 200000u) return; }
}

int8_t temp_read_c(void)
{
    // On-demand conversion of the temp sensor channel at the longest sample time.
    ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 1, ADC_SampleTime_CyclesMode5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    // Bail with sentinel if EOC never arrives.
    uint32_t guard = 0;
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {
        if (++guard > 100000u) return -128;
    }

    int32_t raw = (int32_t)ADC_GetConversionValue(ADC1);
    // Raw 12-bit count to millivolts (3.3 V ref, 4096 counts full-scale).
    int32_t mv = (raw * 3300) / 4096;
    int32_t c  = TempSensor_Volt_To_Temper(mv);
    if (c < -128) c = -128;
    if (c >  127) c =  127;
    return (int8_t)c;
}
