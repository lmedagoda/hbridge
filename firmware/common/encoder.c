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
    encoder->encoderConfig.ticksPerTurn = 0;
    encoder->encoderConfig.leapTickValue = 0;
    encoder->encoderConfig.curTickValue = 0;
    encoder->encoderConfig.lastRawValue = 0;
    encoder->encoderConfig.leapTickCounter = 0;
    encoder->encoderConfig.tickDivider = 1;
    encoder->encoderConfig.configured = 0;
    
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


void encoder_sampleTicks(enum encoderTypes type)
{
    //get value from sensor
    uint32_t curValue = encoders[type].getTicks();
    const int32_t leapTickValue = encoders[type].encoderConfig.leapTickValue;
    
    if(!leapTickValue)
    {
	encoders[type].encoderConfig.curTickValue = curValue;
	return;
    }

    const uint32_t oldValue = encoders[type].encoderConfig.lastRawValue;
    encoders[type].encoderConfig.lastRawValue = curValue;
    
    const uint32_t ticksPerTurn = encoders[type].encoderConfig.ticksPerTurn;
    
    //compute diff between values
    int32_t diff = oldValue - curValue;
    if(abs(diff) > ticksPerTurn / 2.0)
    {
	if(oldValue < curValue)
	{
	    //forward
	    diff += ticksPerTurn;
	}
	else
	{
	    //backwards
	    diff -= ticksPerTurn;
	}
    }
    
    int32_t leapTickCounter = encoders[type].encoderConfig.leapTickCounter;
    leapTickCounter += diff;
    
    //compute output tick value
    int32_t tickValue = encoders[type].encoderConfig.curTickValue;    
    if(leapTickCounter < -leapTickValue)
    {
	leapTickCounter += leapTickValue;
	tickValue -= 1;
    }
	
    if( leapTickCounter > leapTickCounter)
    {
	leapTickCounter -= leapTickValue;
	tickValue += 1;
    }    
    tickValue += diff;

    
    encoders[type].encoderConfig.leapTickCounter = leapTickCounter;
    encoders[type].encoderConfig.curTickValue = tickValue;
}

uint32_t getTicks(enum encoderTypes type)
{
    return encoders[type].encoderConfig.curTickValue;
}

uint16_t getDividedTicks(enum encoderTypes type)
{
    if(encoders[type].encoderConfig.tickDivider != 0)
        return encoders[type].getTicks() / encoders[type].encoderConfig.tickDivider;
    else
        return 0;
}

void encoder_setTicksPerTurn(enum encoderTypes type, uint32_t ticks, uint8_t tickDivider, uint32_t leapTicks)
{
    encoders[type].encoderConfig.configured = 1;

 
    encoders[type].encoderConfig.leapTickValue = leapTicks;
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
