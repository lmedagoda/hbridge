#include "inc/stm32f10x_tim.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_gpio.h"
#include "inc/stm32f10x_exti.h"
#include <stdlib.h>
#include "printf.h"
#include "encoder.h"
#include "protocol.h"
#include "inc/stm32f10x_spi.h"
#include "encoder_adc.h"
#include "encoder_quadrature.h"
#include "encoder_ichaus.h"
#include "encoder_quadrature_exti.h"

struct EncoderInterface encoders[NUM_ENCODERS];

void defaultEncoderInit(void) {}
void defaultSetTicksPerTurn(uint32_t ticks, uint8_t tickDivider) {}
uint32_t defaultGetTicks(void) {return 0;}
void defaultEncoderDeInit(void) {}

void defaultInitEncoder(struct EncoderInterface *encoder)
{
    encoder->encoderConfig.configured = 0;
    encoder->encoderConfig.tickDivider = 1;
    encoder->encoderConfig.ticksPerTurn = 0;
    encoder->encoderInit = defaultEncoderInit;
    encoder->getTicks = defaultGetTicks;
    encoder->setTicksPerTurn = defaultSetTicksPerTurn;
    encoder->encoderDeInit = defaultEncoderDeInit;
}


void encodersInit()
{
    int i;
    for(i = 0; i < NUM_ENCODERS; i++)
    {
    	defaultInitEncoder(encoders + i);
    }

    //the non existing encoder is allways configured
    encoders[NO_ENCODER].encoderConfig.configured = 1;

    encoders[QUADRATURE].encoderInit = encoderInitQuadrature;
    encoders[QUADRATURE].getTicks = getTicksQuadrature;
    encoders[QUADRATURE].setTicksPerTurn = setTicksPerTurnQuadrature;

    encoders[QUADRATURE_WITH_ZERO].encoderInit = encoderInitQuadratureWithZero;
    encoders[QUADRATURE_WITH_ZERO].getTicks = getTicksQuadratureWithZero;
    encoders[QUADRATURE_WITH_ZERO].setTicksPerTurn = setTicksPerTurnQuadratureWithZero;

//     encoders[BMMV30_SSI].encoderInit = encoderInitBMMV;
//     encoders[BMMV30_SSI].getTicks = getTicksBMMV;
//     encoders[BMMV30_SSI].setTicksPerTurn = setTicksPerTurnBMMV;

    encoders[IC_HOUSE_MH_Y].encoderInit = encoderInitIcHaus;
    encoders[IC_HOUSE_MH_Y].getTicks = getTicksIcHaus;
    encoders[IC_HOUSE_MH_Y].setTicksPerTurn = setTicksPerTurnIcHaus;
    
    encoders[ANALOG_VOLTAGE].encoderInit = encoderInitADC;
    encoders[ANALOG_VOLTAGE].encoderDeInit = encoderDeInitADC;
    encoders[ANALOG_VOLTAGE].getTicks = getTicksADC;
    encoders[ANALOG_VOLTAGE].setTicksPerTurn = setTicksPerTurnADC;
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

void setTicksPerTurn(enum encoderTypes type, uint32_t ticks, uint8_t tickDivider)
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


void initEncoder(enum encoderTypes type) 
{
    encoders[type].encoderInit();
    if(type != NO_ENCODER)
        encoders[type].encoderConfig.configured = 0;
}

void deinitEncoder(enum encoderTypes type) 
{
    encoders[type].encoderDeInit();
}
