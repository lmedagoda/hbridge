#ifndef ENCODER_ADC_H
#define ENCODER_ADC_H
#include <stdint.h>

void encoderInitADC();
void setTicksPerTurnADC(uint32_t ticks, uint8_t tickDivider);
uint32_t getTicksADC(void);
void encoderDeInitADC(void);

#endif
