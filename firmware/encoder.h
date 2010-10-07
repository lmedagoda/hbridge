#ifndef __ENCODER_H
#define __ENCODER_H

#include "inc/stm32f10x_type.h"
#include "protocol.h"

struct EncoderInterface
{
    void (*encoderInit) (void);
    void (*setTicksPerTurn) (u32 ticks, u8 tickDivider);
    u32 (*getTicks) (void);
    u16 (*getDividedTicks) (void);
    u32 (*getTicksPerTurn) (void);
    void (*encoderDeInit) (void);
};

void setExternalEncoder(enum encoderTypes type);
u32 getTicks(enum encoderTypes type);
u16 getDividedTicks(enum encoderTypes type);
u32 getTicksPerTurn(enum encoderTypes type);
void setTicksPerTurn(enum encoderTypes type, u32 ticks, u8 tickDivider);

void encodersInit();

u8 encodersConfigured();

#endif
