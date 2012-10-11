#include "inc/stm32f10x_tim.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_adc.h"
#include "inc/stm32f10x_gpio.h"
#include "hbridge.h"
#include <stdlib.h>

volatile uint8_t lastDirection = 0;


void initHbridgeTimers()
{
    //timer used for PWM generation
    //turn on timer hardware
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

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
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3;

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
    
    ocstruct.TIM_OCMode = TIM_OCMode_PWM2;
    ocstruct.TIM_OutputState = TIM_OutputState_Enable;
    ocstruct.TIM_OCPolarity = TIM_OCPolarity_Low;
    //no pulse initally
    ocstruct.TIM_Pulse = 0;
    
    //high side have high polarity
    TIM_OC1Init(TIM1, &ocstruct);
    TIM_OC3Init(TIM1, &ocstruct);

    //low sides have low polarity
    ocstruct.TIM_OCMode = TIM_OCMode_PWM2;
    ocstruct.TIM_OCPolarity = TIM_OCPolarity_High;
    //initally low
    ocstruct.TIM_Pulse = 900;
    TIM_OC2Init(TIM1, &ocstruct);
    TIM_OC4Init(TIM1, &ocstruct);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    
    TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
    
    //output a trigger event on ever update event for adc synchronisation
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);

    // TIM1 enable counter
    TIM_Cmd(TIM1, ENABLE);

    //set RCR AFTER timer was enables, 
    //so that update event occurs on underflow 
    TIM1->RCR = 1;
    
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

void setNewPWM(const int16_t value2, uint8_t useOpenLoop) {
    int16_t value = value2 / 2;
    uint16_t leftHighCC = 0;
    uint16_t rightHighCC = 0;
    uint16_t leftLowCC = 0;
    uint16_t rightLowCC = 0;

    //set dead time to 500ns
    const int16_t dead_time = 36;  

    const uint16_t low_allways_on = 0;
    const uint16_t low_allways_off = 900;
    const uint16_t high_allways_off = 0;

    //time low side needs to be high 
    //so that boost capacitor get's recharged
    const uint16_t half_recharge_time = 30;

    const int16_t maxValue = 900;

    int16_t dutyTime = abs(value);

    //limit duty cycle to maximum possilbe pwm
    if(dutyTime + dead_time + half_recharge_time > maxValue)
    {
	dutyTime = maxValue - dead_time - half_recharge_time;
    }

    uint8_t desiredDirection = 0;

    if(value != 0) {
	desiredDirection = value > 0;
    } else {
	desiredDirection = lastDirection;
    }

    if(desiredDirection) {
	//forward case
	leftHighCC = dutyTime;
	rightHighCC = high_allways_off;
	rightLowCC = low_allways_on;

	if(useOpenLoop) {
	    leftLowCC = low_allways_off - half_recharge_time;
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
	    rightLowCC = low_allways_off - half_recharge_time;
	} else {
	    //closed loop
	    rightLowCC = dutyTime + dead_time;
	}    
    }

    TIM_UpdateDisableConfig(TIM1, ENABLE);

    TIM1->CCR1 = leftHighCC;
    TIM1->CCR2 = leftLowCC;
    TIM1->CCR3 = rightHighCC;
    TIM1->CCR4 = rightLowCC;

    TIM_UpdateDisableConfig(TIM1, DISABLE);

    lastDirection = desiredDirection;
}
