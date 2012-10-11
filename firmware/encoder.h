#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>
#include "protocol.h"

struct encoderData {
    uint32_t ticksPerTurn;
    uint8_t tickDivider;
    uint8_t configured;
};

struct EncoderInterface
{
    struct encoderData encoderConfig;
    void (*encoderInit) (void);
    void (*setTicksPerTurn) (uint32_t ticks, uint8_t tickDivider);
    uint32_t (*getTicks) (void);
    void (*encoderDeInit) (void);
};

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

void setExternalEncoder(enum encoderTypes type);
uint32_t getTicks(enum encoderTypes type);
uint16_t getDividedTicks(enum encoderTypes type);
uint8_t getTickDivider(enum encoderTypes type);
uint32_t getTicksPerTurn(enum encoderTypes type);
void setTicksPerTurn(enum encoderTypes type, uint32_t ticks, uint8_t tickDivider);
void initEncoder(enum encoderTypes type);
void deinitEncoder(enum encoderTypes type);

void encodersInit();

uint8_t encoderConfigured(enum encoderTypes type);

#endif
