#ifndef __ENCODER_H
#define __ENCODER_H

#include "inc/stm32f10x_type.h"

void encoderInit();

void setTicksPerTurn(u32 ticks);

u32 getTicks();

void externalEncoderInit();
void setTicksPerTurnExtern(u32 ticks);
u32 getExternalEncoderTicks();

#endif
