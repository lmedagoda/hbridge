#ifndef __ENCODER_H
#define __ENCODER_H

#include "inc/stm32f10x_type.h"
#include "protocol.h"

struct encoderData {
    u32 ticksPerTurn;
    u8 tickDivider;
    u8 configured;
};

struct EncoderInterface
{
    struct encoderData encoderConfig;
    void (*encoderInit) (void);
    void (*setTicksPerTurn) (u32 ticks, u8 tickDivider);
    u32 (*getTicks) (void);
    void (*encoderDeInit) (void);
};

void setExternalEncoder(enum encoderTypes type);
u32 getTicks(enum encoderTypes type);
u16 getDividedTicks(enum encoderTypes type);
u8 getTickDivider(enum encoderTypes type);
u32 getTicksPerTurn(enum encoderTypes type);
void setTicksPerTurn(enum encoderTypes type, u32 ticks, u8 tickDivider);
void initEncoder(enum encoderTypes type);
void deinitEncoder(enum encoderTypes type);

void encodersInit();

u8 encoderConfigured(enum encoderTypes type);

#endif
