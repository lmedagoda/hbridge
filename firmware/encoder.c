#include "inc/stm32f10x_tim.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_gpio.h"
#include "inc/stm32f10x_exti.h"
#include "inc/stm32f10x_nvic.h"
#include <stdlib.h>
#include "printf.h"
#include "encoder.h"
#include "protocol.h"

struct EncoderInterface encoders[NUM_ENCODERS];

enum encoderTypes externalEncoderType;

struct encoderData {
    u16 ticksPerTurn;
    u8 tickDivider;
};

vs32 externalEncoderValue = 0;

static vu8 configured = 0;
static vu8 foundZero = 0;

struct encoderData internalEncoderConfig;
struct encoderData externalEncoderConfig;

void defaultEncoderInit(void) {}
void defaultSetTicksPerTurn(u32 ticks, u8 tickDivider) {}
u32 defaultGetTicks(void) {return 0;}
u16 defaultGetDividedTicks(void) {return 0;}
u32 defaultGetTicksPerTurn(void) {return 0;}
void defaultEncoderDeInit(void) {}

void defaultInitEncoder(struct EncoderInterface *encoder)
{
    encoder->encoderInit = defaultEncoderInit;
    encoder->getTicks = defaultGetTicks;
    encoder->getDividedTicks = defaultGetDividedTicks;
    encoder->getTicksPerTurn = defaultGetTicksPerTurn;
    encoder->setTicksPerTurn = defaultSetTicksPerTurn;
    encoder->encoderDeInit = defaultEncoderDeInit;
}

u8 encodersConfigured() {
    return configured;
}
// incremental encoder
void encoderInitQuadrature() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;

    //get default GPIO config
    GPIO_StructInit(&GPIO_InitStructure);
    
      //configure Timer4 ch1 (PB6) as encoder input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    //configure Timer4 ch2 (PB7) as encoder input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);


    //turn on timer hardware
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    // Time base configuration 
    TIM_TimeBaseStructure.TIM_Period = 65000;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    //configure TIM4 as encoder interface
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,
				TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);    

    //only take edges into account where the 
    //signal is stable for at least 8 clock cycles
    TIM4->CCMR1 |= (3<<4) | (3<<12);
    TIM4->CCMR2 |= (3<<4) | (3<<12);

    TIM_SetCounter(TIM4, 0);

    // TIM enable counter
    TIM_Cmd(TIM4, ENABLE);    
    
    internalEncoderConfig.tickDivider = 1;
    internalEncoderConfig.ticksPerTurn = 0;    
}

void setTicksPerTurnQuadrature(u32 ticks, u8 tickDivider) {
    if(configured && internalEncoderConfig.ticksPerTurn == ticks && internalEncoderConfig.tickDivider == tickDivider)
	return;
    
    if(ticks == 0)
	ticks = 1;
    
    internalEncoderConfig.tickDivider = tickDivider;
    internalEncoderConfig.ticksPerTurn = ticks;
    TIM_Cmd(TIM4, DISABLE);    
 
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    // Time base configuration 
    TIM_TimeBaseStructure.TIM_Period = ticks - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    //configure value to reload after wrap around
    TIM_SetAutoreload(TIM4, ticks - 1);
    
    TIM_SetCounter(TIM4, 0);
    
    TIM_Cmd(TIM4, ENABLE);
    foundZero = 0;
    configured = 1;
}

u32 getTicksQuadrature()
{
  static s32 lastEncoderValue = 0;
  static s16 wrapCounter = 0;

  //get encoder
  s32 encoderValue = TIM_GetCounter(TIM4);
  
  u32 wheelPos = encoderValue;
  s32 diff = encoderValue - lastEncoderValue;
  //we got a wraparound
  if(abs(diff) > internalEncoderConfig.ticksPerTurn / 2) {
      //test if we wrapped "forward" or "backwards"
      if(diff < 0)  {
	  //forward
	  wrapCounter++;
	  if(wrapCounter >= internalEncoderConfig.tickDivider)
	      wrapCounter -= internalEncoderConfig.tickDivider;
      } else {
	  //backward
	  wrapCounter--;
	  if(wrapCounter < 0)
	      wrapCounter += internalEncoderConfig.tickDivider;
      }
  }

  wheelPos += wrapCounter * internalEncoderConfig.ticksPerTurn;

  lastEncoderValue = encoderValue;

  return wheelPos;
}

u16 getDividedTicksQuadrature() {
    u32 ticks = getTicksQuadrature();
    return ticks / internalEncoderConfig.tickDivider;
}

void setTicksPerTurnQuadratureWithZero(u32 ticks, u8 tickDivider) {
    if(configured && externalEncoderConfig.ticksPerTurn == ticks && externalEncoderConfig.tickDivider == tickDivider)
	return;
    
    externalEncoderConfig.tickDivider = tickDivider;
    externalEncoderConfig.ticksPerTurn = ticks;
    configured = 1;
    foundZero = 0;
}

vu32 ainIT = 0;
vu32 binIT = 0;
vs32 zeroIT = 0;

u32 getTicksQuadratureWithZero() {
    if(zeroIT != -1) {
	printf("Z: %li \n", zeroIT );
	zeroIT = -1;
    }
/*    printf("A: %lu, B:%lu, Z: %lu\n", ainIT, binIT, zeroIT);
    printf("Enc : %l \n", externalEncoderValue);*/
    if(!foundZero)
	return 0;

    return externalEncoderValue;
}

u16 getDividedTicksQuadratureWithZero() {
    if(!foundZero)
	return 0;
    return externalEncoderValue / externalEncoderConfig.tickDivider;
}


void EXTI15_10_IRQHandler(void)
{
    u16 ain = GPIOB->IDR & GPIO_Pin_13; //GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13);
    u16 bin = GPIOB->IDR & GPIO_Pin_14; //GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14);

   
    if(EXTI_GetITStatus(EXTI_Line13) != RESET) {
	ainIT++;
	//impicit ain != lastAin
	if(ain) {
	    if(bin)
		externalEncoderValue++;
	    else
		externalEncoderValue--;      
	} else { //!ain
	    if(bin)
		externalEncoderValue--;
	    else
		externalEncoderValue++;	    
	}
	EXTI_ClearITPendingBit(EXTI_Line13);
    }

    if(EXTI_GetITStatus(EXTI_Line14) != RESET) {
	binIT++;
	if(bin) {
	    if(ain)
		externalEncoderValue--;
	    else
		externalEncoderValue++;      
	} else {
	    if(ain)
		externalEncoderValue++;
	    else
		externalEncoderValue--;	    
	}      
	EXTI_ClearITPendingBit(EXTI_Line14);
    }


    if(externalEncoderValue < 0)
	externalEncoderValue += externalEncoderConfig.ticksPerTurn;

    if(externalEncoderValue > externalEncoderConfig.ticksPerTurn)
	externalEncoderValue -= externalEncoderConfig.ticksPerTurn;

    if(EXTI_GetITStatus(EXTI_Line12) != RESET) {
	if(zeroIT == -1)
	    zeroIT = externalEncoderValue;
	externalEncoderValue = 0;
	if(configured)
	    foundZero = 1;
	EXTI_ClearITPendingBit(EXTI_Line12);
    }

}

// incremental encoder with zero signal
void encoderInitQuadratureWithZero() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;

    //get default GPIO config
    GPIO_StructInit(&GPIO_InitStructure);

    //Configure GPIO pins: AIN BIN and Zero
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);    

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14);

    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line12 | EXTI_Line13 | EXTI_Line14;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line13;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line14;
    EXTI_Init(&EXTI_InitStructure);

    //programm encoder interrutps to highest priority
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_StructInit(&NVIC_InitStructure);    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    externalEncoderValue = 0;
    externalEncoderConfig.ticksPerTurn = 0;
    externalEncoderConfig.tickDivider = 1;
}

void encodersInit()
{
    int i;
    for(i = 0; i < NUM_ENCODERS; i++)
    {
    	defaultInitEncoder(encoders + i);
    }

    encoders[QUADRATURE].encoderInit = encoderInitQuadrature;
    encoders[QUADRATURE].getTicks = getTicksQuadrature;
    encoders[QUADRATURE].getDividedTicks = getDividedTicksQuadrature;
    encoders[QUADRATURE].setTicksPerTurn = setTicksPerTurnQuadrature;

    encoders[QUADRATURE_WITH_ZERO].encoderInit = encoderInitQuadratureWithZero;
    encoders[QUADRATURE_WITH_ZERO].getTicks = getTicksQuadratureWithZero;
    encoders[QUADRATURE_WITH_ZERO].getDividedTicks = getDividedTicksQuadratureWithZero;
    encoders[QUADRATURE_WITH_ZERO].setTicksPerTurn = setTicksPerTurnQuadratureWithZero;
}


u32 getTicks(enum encoderTypes type)
{
    return encoders[type].getTicks();
}

u16 getDividedTicks(enum encoderTypes type)
{
    return encoders[type].getDividedTicks();
}

void setTicksPerTurn(enum encoderTypes type, u32 ticks, u8 tickDivider)
{
    encoders[type].setTicksPerTurn(ticks, tickDivider);
}

u32 getTicksPerTurn(enum encoderTypes type)
{
    return encoders[type].getTicksPerTurn();
}
