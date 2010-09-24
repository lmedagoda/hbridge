#ifndef __CURRENT_MEASUREMENT_H
#define __CURRENT_MEASUREMENT_H
#include "inc/stm32f10x_type.h"
#include "inc/stm32f10x_adc.h"

#define USED_REGULAR_ADC_CHANNELS 12

extern vu16 adc_values[USED_REGULAR_ADC_CHANNELS];

void requestNewADCValues();
void waitForNewADCValues();
u32 getBatteryVoltage();
void currentMeasurementInit();
u32 calculateCurrent(s32 currentPwmValue);
void measureACS712BaseVoltage();


#endif
