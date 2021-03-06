#include "encoder.h"
#include "inc/stm32f10x_tim.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_gpio.h"
#include "core_cm3.h"
#include <stdlib.h>
#include "printf.h"

struct TimerQuadratureEncoderData
{
    uint32_t maxWraps;
    uint32_t maxTicksPerTurnIntern;
    int32_t lastEncoderValue;
    int16_t wrapCounter;
    uint8_t usesZero;
    volatile uint8_t hasZero;
};

struct TimerQuadratureEncoderData tqeTim2Data;
struct TimerQuadratureEncoderData tqeTim3Data;
struct TimerQuadratureEncoderData tqeTim4Data;

void TIM2_IRQHandler(void){
    //Clear TIM2 Capture compare interrupt pending bit
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
    TIM_SetCounter(TIM2, 0);
  
    tqeTim2Data.hasZero = 1;
}  

void timerQuadratureEncoderEnableZero(TIM_TypeDef *timer, uint8_t irq_channel)
{
    TIM_ICInitTypeDef  TIM_ICInitStructure;

    //Initialise IC Initstruct with default values
    TIM_ICStructInit(&TIM_ICInitStructure);

    //configure as input compare
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0F;//0x08; //0 to 0x0F
    TIM_ICInit(timer, &TIM_ICInitStructure);    

    //Enable for timer the CC3 Interrupt Request 
    TIM_ITConfig(timer, TIM_IT_CC3, ENABLE);

    //programm encoder interrutps to highest priority
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = irq_channel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void timerQuadratureEncoderDisableZero(TIM_TypeDef *timer, uint8_t irq_channel)
{
    TIM_ITConfig(timer, TIM_IT_CC3, DISABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = irq_channel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);

}

void timerQuadratureEncoderDeInit(TIM_TypeDef *timer, struct TimerQuadratureEncoderData *data)
{
    //disable first
    TIM_Cmd(timer, DISABLE);
    
    if(data->hasZero)
	//FIXME only works for Timer 2
	timerQuadratureEncoderDisableZero(timer, TIM2_IRQn);
    
    TIM_DeInit(timer);
}

void timerQuadratureEncoderInit(TIM_TypeDef *timer, struct TimerQuadratureEncoderData *data, int withZero)
{
    //default init
    data->maxWraps = 0; 
    data->maxTicksPerTurnIntern = 1;    
    data->lastEncoderValue = 0;
    data->wrapCounter = 0;
    data->usesZero = withZero;
    data->hasZero = !withZero;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    //get default GPIO config
    GPIO_StructInit(&GPIO_InitStructure);

    if(timer == TIM4)
    {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//turn on timer hardware
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	    
	//configure Timer4 ch1 (PB6) as encoder input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//configure Timer4 ch2 (PB7) as encoder input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	if(withZero)
	{
	    //not implemented, bail out
	    assert_param(0);
	}
    } else
	if(timer == TIM2)
    {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//turn on timer hardware
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	
	    
	//configure Timer2 ch1 (PA0) as encoder input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//configure Timer2 ch2 (PA1) as encoder input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	if(withZero)
	{
	    //configure Timer2 ch2 (PA2) as encoder input
	    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	    GPIO_Init(GPIOA, &GPIO_InitStructure);
	    
	    timerQuadratureEncoderEnableZero(timer, TIM2_IRQn);
	}
    } else
	if(timer == TIM3)
    {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//turn on timer hardware
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	    
	//configure Timer2 ch1 (PA0) as encoder input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//configure Timer2 ch2 (PA1) as encoder input
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	if(withZero)
	{
	    //not implemented, bail out
	    assert_param(0);
	}
    } else {
	//not implemented, bail out
	assert_param(0);
    }
    
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    //disable for configuration
    TIM_Cmd(timer, DISABLE);
    
    // Time base configuration 
    TIM_TimeBaseStructure.TIM_Period = 65000;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);

    //configure TIM4 as encoder interface
    TIM_EncoderInterfaceConfig(timer, TIM_EncoderMode_TI12,
                                TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);    

    //only take edges into account where the 
    //signal is stable for at least 8 clock cycles
    timer->CCMR1 |= (3<<4) | (3<<12);
    timer->CCMR2 |= (3<<4) | (3<<12);

    TIM_SetCounter(timer, 0);

    //do not enable timer, so that getCount returns zero.
    //Timer will be enabled as soon as ticks are set
    TIM_Cmd(timer, DISABLE);    
}

void timerQuadratureEncoderSetTicksPerTurn(TIM_TypeDef *timer, struct TimerQuadratureEncoderData *data, uint32_t ticks, uint8_t tickDivider) 
{
    if(ticks == 0)
        ticks = 1;
    
    //calcualte how often ticks fits into 16 bits
    data->maxWraps = ticks / (1<<16) + 1;
    data->maxTicksPerTurnIntern = ticks / data->maxWraps;    
    
    printf("ticks: %lu, ", ticks);
    printf("max Wraps: %lu, maxTicksPerTurnIntern:%lu \n", data->maxWraps, data->maxTicksPerTurnIntern);
    
    TIM_Cmd(timer, DISABLE);    
 
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

    // Time base configuration 
    TIM_TimeBaseStructure.TIM_Period = data->maxTicksPerTurnIntern - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);
    
    //configure value to reload after wrap around
    TIM_SetAutoreload(timer, data->maxTicksPerTurnIntern - 1);
    
    TIM_SetCounter(timer, 0);
    
    TIM_Cmd(timer, ENABLE);
    
    data->hasZero = !data->usesZero;
}

uint32_t timerQuadratureEncoderGetTicks(TIM_TypeDef *timer, struct TimerQuadratureEncoderData *data)
{
    //if have never seen a zero mark, return the 'magic' value zero
    if(!data->hasZero)
	return 0;
    
    //get encoder value
    int32_t encoderValue = TIM_GetCounter(timer);
    
    uint32_t wheelPos = encoderValue;
    int32_t diff = encoderValue - data->lastEncoderValue;
    //we got a wraparound
    if(abs(diff) > data->maxTicksPerTurnIntern / 2) {
	//test if we wrapped "forward" or "backwards"
	if(diff < 0)  {
	    //forward
	    (data->wrapCounter)++;
	    if(data->wrapCounter >= data->maxWraps)
		data->wrapCounter -= data->maxWraps;
	} else {
	    //backward
	    (data->wrapCounter)--;
	    if(data->wrapCounter < 0)
		data->wrapCounter += data->maxWraps;
	}
    }

    wheelPos += data->wrapCounter * data->maxTicksPerTurnIntern;

    data->lastEncoderValue = encoderValue;

    return wheelPos;
}


// incremental encoder
void encoderInitQuadrature() {
    timerQuadratureEncoderInit(TIM4, &tqeTim4Data, 0);
}

void setTicksPerTurnQuadrature(uint32_t ticks, uint8_t tickDivider) 
{
    timerQuadratureEncoderSetTicksPerTurn(TIM4, &tqeTim4Data, ticks, tickDivider);
}

uint32_t getTicksQuadrature()
{
    return timerQuadratureEncoderGetTicks(TIM4, &tqeTim4Data);
}

void encoderInitQuadratureV2()
{
    timerQuadratureEncoderInit(TIM3, &tqeTim3Data, 0);
}

void encoderDeInitQuadratureV2()
{
    timerQuadratureEncoderDeInit(TIM3, &tqeTim3Data);
}


void setTicksPerTurnQuadratureV2(uint32_t ticks, uint8_t tickDivider)
{
    timerQuadratureEncoderSetTicksPerTurn(TIM3, &tqeTim3Data, ticks, tickDivider);
}

uint32_t getTicksQuadratureV2()
{
    return timerQuadratureEncoderGetTicks(TIM3, &tqeTim3Data);
}

void encoderInitQuadratureWithZeroV2()
{
    timerQuadratureEncoderInit(TIM2, &tqeTim2Data, 1);
}

void encoderDeInitQuadratureWithZeroV2()
{
    timerQuadratureEncoderDeInit(TIM2, &tqeTim2Data);
}

void setTicksPerTurnQuadratureWithZeroV2(uint32_t ticks, uint8_t tickDivider)
{
    timerQuadratureEncoderSetTicksPerTurn(TIM2, &tqeTim2Data, ticks, tickDivider);
}

uint32_t getTicksQuadratureWithZeroV2()
{
    return timerQuadratureEncoderGetTicks(TIM2, &tqeTim2Data);
}



