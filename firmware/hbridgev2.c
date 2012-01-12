#include "inc/stm32f10x_tim.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_adc.h"
#include "inc/stm32f10x_gpio.h"
#include "hbridge.h"
#include <stdlib.h>

vu16 leftHighCC;
vu16 rightHighCC;
vu16 leftLowCC;
vu16 rightLowCC;



void initHbridgeTimers()
{

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    //we want to have a signal of 25us lenght
    //as centeraligend mode counts down and up
    //we need to half the period
    //(TIM2CLK = 72 MHz / 40 kHz = Period = 1800)
    //therefore the resulting period is 900
    TIM_TimeBaseStructure.TIM_Period = 899;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    //only generate an update event when downcounting
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3;

    //timer used for PWM generation
    //turn on timer hardware
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    //init timer to 40khz
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // Prescaler configuration 
    TIM_PrescalerConfig(TIM1, 0, TIM_PSCReloadMode_Immediate); 

    //All channels use buffered registers
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_OCInitTypeDef ocstruct;
    TIM_OCStructInit(&ocstruct);
    
    ocstruct->TIM_OCMode = TIM_OCMode_PWM1;
    ocstruct->TIM_OutputState = TIM_OutputState_Enable;
    ocstruct->TIM_OCPolarity = TIM_OCPolarity_High;
    //no pulse initally
    ocstruct->TIM_Pulse = 0;
    
    //high side have high polarity
    TIM_OC1Init(TIM1, ocstruct);
    TIM_OC3Init(TIM1, ocstruct);

    //low sides have low polarity
    ocstruct->TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC2Init(TIM1, ocstruct);
    TIM_OC4Init(TIM1, ocstruct);

    // TIM1 enable counter
    TIM_Cmd(TIM1, ENABLE);
    
    
    //force update event to occure on underflow
    u8 dir = TIM1->CR1 & (1<<4);
    u8 lastDir = dir;
    
    const u8 down = 1;
    const u8 up = 0;
    
    while(1)
    {
	dir = TIM1->CR1 & (1<<4);
	if(dir == up && lastDir == down)
	{
	    //this rewrites the internal repition counter with
	    //the value from the RCR register, thus moving the 
	    //update event to the correct spot
	    TIM_GenerateEvent(TIM1, TIM_EventSource_Update);
	    break;
	}
	lastDir = dir;
    }
    

}

void hbridgeGPIOConfig() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    //get default GPIO config
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

    //TIM1 channel 1 pin (PA8)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //TIM1 channel 2 pin (PA9)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //TIM1 channel 3 pin (PA10)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //TIM1 channel 4 pin (PA11)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void hbridgeInit() {
    //init gpios
    hbridgeGPIOConfig();
    
    //init all timers
    initHbridgeTimers();

    //set pwm to zero
    setNewPWM(0, 0);
    
}

void setNewPWM(const s16 value2, u8 useOpenLoop) {
  s16 value = value2;

  //truncate to max of 95% PWM, needed to charge boost circuit
  const s16 maxValue = 855;
  
  if(value < -maxValue)
  {
    value = -maxValue;
  }
  else
  {
    if(value > maxValue)
    {
        value = maxValue;
    }
  }  

  u16 dutyTime = abs(value);
  
  const u16 low_allways_on = 0;
  const u16 hight_allways_on = 900;
  const u16 low_allways_off = 900;
  const u16 high_allways_off = 0;
  
  //TODO get real value
  const u16 dead_time = 10;  

  u8 desiredDirection = 0;
  
  if(value != 0) {
    desiredDirection = value > 0;
  }
  
  if(desiredDirection) {
	//forward case
	leftHighCC = dutyTime;
	rightHighCC = high_allways_off;
	rightLowCC = low_allways_on;

    if(useOpenLoop) {
	leftLowCC = low_allways_off;
    } else {
        //closed loop
	leftLowCC = dutyTime + dead_time;
    }    
  } else {
	//backward case
	leftHighCC = high_allways_off;
	rightHighCC = dutyTime;
	leftLowCC = low_allways_on;

    if(useOpenLoop) {
	rightLowCC = low_allways_off;
    } else {
        //closed loop
	rightLowCC = dutyTime + dead_time;
    }    
  }


///not needed, done by repetition counter
//   //activate interrupt to write values to CC
//   //registers at a sane time
//   TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
}

///not needed, done by repetition counter
// void TIM1_UP_IRQHandler(void)
// {
//     //Clear TIM1 update interrupt pending bit
//     TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
//     
//     //only write values if we are downcounting
//     if(TIM_GetCounter(TIM1) > 700)
//     {
// 	//update values
// 	//note as this code is executed
// 	//near to an update event
// 	//we can be relativly shure that
// 	//no quirk happens and all values
// 	//get written atomar
// 	TIM1->CCR1 = leftHighCC;
// 	TIM1->CCR2 = leftLowCC;
// 	TIM1->CCR3 = rightHighCC;
// 	TIM1->CCR4 = rightLowCC;
// 	
// 	//out job is done stop wasting cpu time
// 	//and disable it
// 	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
//     }
// };

