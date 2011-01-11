#ifndef ENCODER_ADC_H
#define ENCODER_ADC_H
#include "inc/stm32f10x_type.h"

void encoderInitADC();
u32 getTicksPerTurnADC();
void setTicksPerTurnADC(u32 ticks, u8 tickDivider);
u32 getTicksADC(void);
u16 getDividedTicksADC(void);
void encoderDeInitADC(void);

#endif