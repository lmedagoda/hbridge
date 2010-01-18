#include "inc/stm32f10x_tim.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_gpio.h"
#include "inc/stm32f10x_exti.h"
#include "inc/stm32f10x_nvic.h"


struct encoderData {
    u16 ticksPerTurn;
    u8 ticksBiggerThanU16;
};

vs32 externalEncoderValue = 0;
vu32 ticksPerTurnExtern; 

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

void setTicksPerTurnExtern(u32 ticks) {
    ticksPerTurnExtern = ticks;
}

vu32 ainIT = 0;
vu32 binIT = 0;
vu32 zeroIT = 0;

u32 getExternalEncoderTicks() {
    printf("A: %lu, B:%lu, Z: %lu\n", ainIT, binIT, zeroIT);
    printf("Enc : %l \n", externalEncoderValue);
    return externalEncoderValue;
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

void EXTI15_10_IRQHandler(void)
{
    u16 ain = GPIOB->IDR & GPIO_Pin_14; //GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14);
    u16 bin = GPIOB->IDR & GPIO_Pin_15; //GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15);
    
    
    if(EXTI_GetITStatus(EXTI_Line14) != RESET) {
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
	EXTI_ClearITPendingBit(EXTI_Line14);
    }

    if(EXTI_GetITStatus(EXTI_Line15) != RESET) {
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
	EXTI_ClearITPendingBit(EXTI_Line15);
    }


    if(externalEncoderValue < 0)
	externalEncoderValue += ticksPerTurnExtern;

    if(externalEncoderValue > ticksPerTurnExtern)
	externalEncoderValue -= ticksPerTurnExtern;

    if(EXTI_GetITStatus(EXTI_Line13) != RESET) {
	zeroIT = externalEncoderValue;
	externalEncoderValue = 0;
	EXTI_ClearITPendingBit(EXTI_Line13);
    }

}


void externalEncoderInit() {
    GPIO_InitTypeDef GPIO_InitStructure;

    //get default GPIO config
    GPIO_StructInit(&GPIO_InitStructure);

    //Configure GPIO pins: AIN BIN and Zero
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);    

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource15);

    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line13 | EXTI_Line14 | EXTI_Line15;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line14;
    EXTI_Init(&EXTI_InitStructure);
    EXTI_InitStructure.EXTI_Line = EXTI_Line15;
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
    
}
