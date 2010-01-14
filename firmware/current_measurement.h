#ifndef __CURRENT_MEASUREMENT_H
#define __CURRENT_MEASUREMENT_H
#include "inc/stm32f10x_type.h"
#include "inc/stm32f10x_adc.h"

#define USED_REGULAR_ADC_CHANNELS 16

extern vu16 adc_values[USED_REGULAR_ADC_CHANNELS];

extern u32 SQR1Forward;
extern u32 SQR2Forward;
extern u32 SQR3Forward;
extern u32 SQR1Reverse;
extern u32 SQR2Reverse;
extern u32 SQR3Reverse;

void requestNewADCValues();
void waitForNewADCValues();
u32 getBatteryVoltage();

void currentMeasurementInit();

u32 calculateCurrent(s32 currentPwmValue);

void measureACS712BaseVoltage();



/**
 * This function configures the channel from which
 * the current measurement is taken, in respect of
 * the direction in which the motor turns.
 */
static inline void configureCurrentMeasurement(vu8 dir) {
  // Disable for Configuration
  ADC1->CR2 &= ~0x01;
  DMA1_Channel1->CCR &= ~0x01;

  DMA1_Channel1->CNDTR = USED_REGULAR_ADC_CHANNELS;

  if(dir) {
    ADC1->SQR1 = SQR1Forward;
    ADC1->SQR2 = SQR2Forward;
    ADC1->SQR3 = SQR3Forward;
  } else {
    ADC1->SQR1 = SQR1Reverse;
    ADC1->SQR2 = SQR2Reverse;
    ADC1->SQR3 = SQR3Reverse;
  }
  
  //Enable End of Conversion interupt
  //ADC1->CR1 |= ADC_IT_EOC

  //enable dma
  DMA1_Channel1->CCR |= 0x01;

  // Enable ADC1
  ADC1->CR2 |= 0x01;
};


#endif
