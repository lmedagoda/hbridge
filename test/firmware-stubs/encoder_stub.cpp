extern "C" {
#include "encoder_adc.h"
#include "encoder_bmmv.h"
#include "encoder_ichaus.h"
#include "encoder_quadrature.h"
#include "encoder_quadrature_exti.h"
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