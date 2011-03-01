#ifndef __ENCODER_H
#define __ENCODER_H

#include "inc/stm32f10x_type.h"

enum encoderStates {
    STATE_00,
    STATE_01,
    STATE_10,
    STATE_11,
};

enum encoderInputs {
    INPUT_00,
    INPUT_01,
    INPUT_10,
    INPUT_11,
};

void encoderInit();
void setTicksPerTurn(u16 ticks, u8 tickDivider);
u32 getTicks();
u16 getDividedTicks();

void encoderInitExtern();
void setTicksPerTurnExtern(u32 ticks, u8 tickDivider);
u32 getTicksExtern();
u16 getDividedTicksExtern();

u8 encodersConfigured();
#endif
