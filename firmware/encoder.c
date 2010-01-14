#include "inc/stm32f10x_tim.h"
#include "inc/stm32f10x_rcc.h"

struct encoderData {
    u16 ticksPerTurn;
    u8 ticksBiggerThanU16;
};

struct encoderData internalEncoder;

void encoderInit() {
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

    TIM_ARRPreloadConfig(TIM4, ENABLE);

    //signal needs to be 8 clock cycles stable
    TIM4->CCMR1 |= (3<<4) | (3<<12);
    TIM4->CCMR2 |= (3<<4) | (3<<12);

    // TIM enable counter
    TIM_Cmd(TIM4, ENABLE);    
}

void setTicksPerTurn(u32 ticks) {
    if(ticks > (1<<15)) {
	assert_param(ticks / 2 < (1<<15));
	internalEncoder.ticksBiggerThanU16 = 1;
	//configure value to reload after wrap around
	TIM_SetAutoreload(TIM4, ticks / 2);
    } else {
	//configure value to reload after wrap around
	TIM_SetAutoreload(TIM4, ticks);
    }
    internalEncoder.ticksPerTurn = ticks;
}

u32 getTicks()
{
  static u16 lastEncoderValue = 0;
  static u8 wheelHalfTurned = 0;

  //get encoder
  u16 encoderValue = TIM_GetCounter(TIM4);
  
  u32 wheelPos = encoderValue;
  
  if(internalEncoder.ticksBiggerThanU16) {
    //calculate correct half wheel position
    if(abs( ((s32) lastEncoderValue) - encoderValue) > internalEncoder.ticksPerTurn / 2) {
	if(wheelHalfTurned)
	    wheelHalfTurned = 0;
	else
	    wheelHalfTurned = 1;
    }

    wheelPos += wheelHalfTurned * internalEncoder.ticksPerTurn / 2;
  }
  
  lastEncoderValue = encoderValue;

  return wheelPos;
}

