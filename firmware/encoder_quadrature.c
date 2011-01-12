#include "encoder.h"
#include "inc/stm32f10x_tim.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_nvic.h"
#include "inc/stm32f10x_gpio.h"
#include <math.h>
#include "printf.h"

u32 maxWraps = 0;
u32 maxTicksPerTurnIntern = 1;    

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

    //disable for configuration
    TIM_Cmd(TIM4, DISABLE);
    
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

    //do not enable timer, so that getCount returns zero.
    //Timer will be enabled as soon as ticks are set
    TIM_Cmd(TIM4, DISABLE);    
}

void setTicksPerTurnQuadrature(u32 ticks, u8 tickDivider) 
{
    if(ticks == 0)
        ticks = 1;
    
    //calcualte how often ticks fits into 16 bits
    maxWraps = ticks / (1<<16) + 1;
    maxTicksPerTurnIntern = ticks / maxWraps;    
    
    printf("ticks: %lu, ", ticks);
    printf("max Wraps: %lu, maxTicksPerTurnIntern:%lu \n", maxWraps, maxTicksPerTurnIntern);
    
    TIM_Cmd(TIM4, DISABLE);    
 
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    // Time base configuration 
    TIM_TimeBaseStructure.TIM_Period = maxTicksPerTurnIntern - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    //configure value to reload after wrap around
    TIM_SetAutoreload(TIM4, maxTicksPerTurnIntern - 1);
    
    TIM_SetCounter(TIM4, 0);
    
    TIM_Cmd(TIM4, ENABLE);
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
  if(abs(diff) > maxTicksPerTurnIntern / 2) {
      //test if we wrapped "forward" or "backwards"
      if(diff < 0)  {
          //forward
          wrapCounter++;
          if(wrapCounter >= maxWraps)
              wrapCounter -= maxWraps;
      } else {
          //backward
          wrapCounter--;
          if(wrapCounter < 0)
              wrapCounter += maxWraps;
      }
  }

  wheelPos += wrapCounter * maxTicksPerTurnIntern;

  lastEncoderValue = encoderValue;

  return wheelPos;
}
