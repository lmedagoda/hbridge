#include <stdlib.h>
#include "printf.h"
#include "../interfaces/encoder.h"
#include "protocol.h"

struct EncoderInterface encoders[NUM_ENCODERS];

void defaultEncoderInit(void) {}
void defaultSetTicksPerTurn(uint32_t ticks, uint8_t tickDivider) {}
uint32_t defaultGetTicks(void) {return 0;}
void defaultEncoderDeInit(void) {}

void encoder_defaultStructInit(struct EncoderInterface *encoder)
{
    encoder->encoderConfig.configured = 0;
    encoder->encoderConfig.tickDivider = 1;
    encoder->encoderConfig.ticksPerTurn = 0;
    encoder->encoderInit = defaultEncoderInit;
    encoder->getTicks = defaultGetTicks;
    encoder->setTicksPerTurn = defaultSetTicksPerTurn;
    encoder->encoderDeInit = defaultEncoderDeInit;
}

void encoder_setImplementation(enum encoderTypes type, struct EncoderInterface impl)
{
    encoders[type] = impl;
}

void encoder_init()
{
    int i;
    for(i = 0; i < NUM_ENCODERS; i++)
    {
    	encoder_defaultStructInit(encoders + i);
    }

    //the non existing encoder is allways configured
    encoders[NO_ENCODER].encoderConfig.configured = 1;
}


uint32_t getTicks(enum encoderTypes type)
{
    return encoders[type].getTicks();
}

uint16_t getDividedTicks(enum encoderTypes type)
{
    if(encoders[type].encoderConfig.tickDivider != 0)
        return encoders[type].getTicks() / encoders[type].encoderConfig.tickDivider;
    else
        return 0;
}

void encoder_setTicksPerTurn(enum encoderTypes type, uint32_t ticks, uint8_t tickDivider)
{
    encoders[type].encoderConfig.configured = 1;

    //do not bother encoder code with anything if the config didn't change
    if((encoders[type].encoderConfig.ticksPerTurn == ticks * tickDivider) && (encoders[type].encoderConfig.tickDivider == tickDivider))
        return;
 
    encoders[type].encoderConfig.tickDivider = tickDivider;
    encoders[type].encoderConfig.ticksPerTurn = ticks * tickDivider;
    encoders[type].setTicksPerTurn(ticks * tickDivider, tickDivider);
}

uint32_t getTicksPerTurn(enum encoderTypes type)
{
    return encoders[type].encoderConfig.ticksPerTurn;
}

uint8_t getTickDivider(enum encoderTypes type)
{
    if(encoders[type].encoderConfig.tickDivider == 0)
	return 1;
    return encoders[type].encoderConfig.tickDivider;
}

uint8_t encoderConfigured(enum encoderTypes type)
{
    return encoders[type].encoderConfig.configured;
}


void encoder_initEncoder(enum encoderTypes type) 
{
    encoders[type].encoderInit();
    if(type != NO_ENCODER)
        encoders[type].encoderConfig.configured = 0;
}

void encoder_deinitEncoder(enum encoderTypes type) 
{
    encoders[type].encoderDeInit();
}
