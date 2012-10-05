#ifndef __CURRENT_MEASUREMENT_H
#define __CURRENT_MEASUREMENT_H

#include <stdint.h>

#define USED_REGULAR_ADC_CHANNELS 12

extern volatile uint16_t adc_values[USED_REGULAR_ADC_CHANNELS];

void requestNewADCValues();
void waitForNewADCValues();
void currentMeasurementInit();
uint32_t calculateCurrent();
void measureACS712BaseVoltage();


#endif
