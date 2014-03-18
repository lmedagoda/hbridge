#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>
#include "../common/packets.h"

struct encoderData {
    uint32_t ticksPerTurn;
    uint32_t leapTickValue;
    
    //current value of the encoder
    //corrected by leap ticks
    uint32_t curTickValue;
    //last value reported by the sensor
    uint32_t lastRawValue;
    int32_t leapTickCounter;
    
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

void encoder_defaultStructInit(struct EncoderInterface *encoder);
void encoder_setImplementation(enum encoderTypes type, struct EncoderInterface impl);

void encoder_initEncoder(enum encoderTypes type);
void encoder_deinitEncoder(enum encoderTypes type);
void encoder_setTicksPerTurn(enum encoderTypes type, uint32_t ticks, uint8_t tickDivider, uint32_t leapTicks);
void encoder_getTicksPerTurn(enum encoderTypes type, uint32_t *ticks, uint8_t *tickDivider, uint32_t *leapTicks);




void setExternalEncoder(enum encoderTypes type);
void encoder_sampleTicks(enum encoderTypes type);
uint32_t getTicks(enum encoderTypes type);
uint16_t getDividedTicks(enum encoderTypes type);
uint8_t getTickDivider(enum encoderTypes type);
uint32_t getTicksPerTurn(enum encoderTypes type);

uint8_t encoderConfigured(enum encoderTypes type);

/**
 * Initializes the internal data structures.
 * */
void encoder_init();


#endif
