#ifndef __ADC_H
#define __ADC_H

#define USED_REGULAR_ADC_CHANNELS 16

extern vu16 adc_values[USED_REGULAR_ADC_CHANNELS];

extern u32 SQR1Forward;
extern u32 SQR2Forward;
extern u32 SQR3Forward;
extern u32 SQR1Reverse;
extern u32 SQR2Reverse;
extern u32 SQR3Reverse;


/**
 * This function programms the watchdog to be
 * triggered by TI CH3. It also selects the 
 * channel to be guarded in respect to the
 * direction the motor turns.
 */
void configureWatchdog(vu8 dir);

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
