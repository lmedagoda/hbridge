#ifndef __ENCODER_H
#define __ENCODER_H

#include "inc/stm32f10x_type.h"

void encoderInit();
void setTicksPerTurn(u32 ticks, u8 tickDivider);
u32 getTicks();
u16 getDividedTicks();

void encoderInitExtern();
void setTicksPerTurnExtern(u32 ticks, u8 tickDivider);
u32 getTicksExtern();
u16 getDividedTicksExtern();

u8 encodersConfigured();
#endif
