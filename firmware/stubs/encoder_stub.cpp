extern "C" {
#include "interfaces/encoder.h"
}


void encoderInitIcHaus(){}

void setTicksPerTurnIcHaus(uint32_t ticks, uint8_t tickDivider){}
uint32_t getTicksIcHaus(void){
    return 0;
}

uint16_t getDividedTicksIcHaus(void){    
    return 0;
}


void encoderInitQuadrature(){}
uint32_t getTicksQuadrature(){    
    return 0;
}

void setTicksPerTurnQuadrature(uint32_t ticks, uint8_t tickDivider){}

void encoderInitQuadratureV2(){}
uint32_t getTicksQuadratureV2(){
        return 0;
}
void setTicksPerTurnQuadratureV2(uint32_t ticks, uint8_t tickDivider){}

void encoderInitQuadratureWithZeroV2(){}
uint32_t getTicksQuadratureWithZeroV2(){
            return 0;
}
void setTicksPerTurnQuadratureWithZeroV2(uint32_t ticks, uint8_t tickDivider){}

void encoderInitQuadratureWithZero(){}
void setTicksPerTurnQuadratureWithZero(uint32_t ticks, uint8_t tickDivider){}
uint32_t getTicksQuadratureWithZero(){
            return 0;
}

void encoderInitADC(){}
void setTicksPerTurnADC(uint32_t ticks, uint8_t tickDivider){}
uint32_t getTicksADC(void){
            return 0;
}
void encoderDeInitADC(void){}

void encodersStubInit()
{
    struct EncoderInterface encoder;
    encoder_defaultStructInit(&encoder);

    encoder.encoderInit = encoderInitQuadrature;
    encoder.getTicks = getTicksQuadrature;
    encoder.setTicksPerTurn = setTicksPerTurnQuadrature;
    encoder_setImplementation(QUADRATURE, encoder);

    encoder.encoderInit = encoderInitQuadratureWithZero;
    encoder.getTicks = getTicksQuadratureWithZero;
    encoder.setTicksPerTurn = setTicksPerTurnQuadratureWithZero;
    encoder_setImplementation(QUADRATURE_WITH_ZERO, encoder);

    encoder.encoderInit = encoderInitIcHaus;
    encoder.getTicks = getTicksIcHaus;
    encoder.setTicksPerTurn = setTicksPerTurnIcHaus;
    encoder_setImplementation(IC_HOUSE_MH_Y, encoder);
    
    encoder.encoderInit = encoderInitADC;
    encoder.encoderDeInit = encoderDeInitADC;
    encoder.getTicks = getTicksADC;
    encoder.setTicksPerTurn = setTicksPerTurnADC;
    encoder_setImplementation(ANALOG_VOLTAGE, encoder);
}
