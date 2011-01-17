#ifndef ENCODER_ADC_H
#define ENCODER_ADC_H
#include "inc/stm32f10x_type.h"

void encoderInitADC();
void setTicksPerTurnADC(u32 ticks, u8 tickDivider);
u32 getTicksADC(void);
void encoderDeInitADC(void);

#endif
