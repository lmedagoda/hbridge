#include "encoder.h"
#include "inc/stm32f10x_tim.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_gpio.h"
#include "core_cm3.h"
#include <stdint.h>

uint8_t gotZeroPos = 0;

/*******************************************************************************
* Function Name  : Encoder_Configuration
* Description    : This function configures TIM3 in encoder mode and an EXTI Reset
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void JointEncoder_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //get default GPIO config
    GPIO_StructInit(&GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);

    //configure enc3 Zero
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //enc3 A/B
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /*--------------- TIMER 3-----------------*/

    // init structs
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef  TIM_ICInitStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_ICStructInit(&TIM_ICInitStructure);
    
    //turn on timer hardware
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // Time base configuration 
    TIM_TimeBaseStructure.TIM_Period = AUTORELOAD_VALUE; // TIM->ARR
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    //configure TIM3 as encoder interface
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,
                                TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_ARRPreloadConfig(TIM3, ENABLE);

    TIM_ICInitStructure.TIM_ICFilter = FILTER_VALUE; //ICx_FILTER;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    // TIM 3 enable counter
    TIM_Cmd(TIM3, ENABLE);

    /*--------------- EXTERNAL IRQ-----------------*/
            
    EXTI_InitTypeDef EXTI_InitStructure;
            
    // Connect Key Button EXTI Line to ENC3_0 GPIO Pin (PC2) 
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource2);

    // Configure ENC3_0 EXTI Line to generate an interrupt on rising edge  
    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Generate software interrupt: simulate a rising edge applied on EXTI2 line 
    EXTI_GenerateSWInterrupt(EXTI_Line2);
}



/*******************************************************************************
* Function Name  : getEncoderReading
* Description    : This function returns the actual encoder value (raw)
* Input          : None
* Output         : None
* Return         : Encoder Value
*******************************************************************************/
int getEncoderReading()
{
    if(!gotZeroPos)
        return 0;

    return TIM3->CNT;
}



/*******************************************************************************
* Function Name  : EXTI2_IRQHandler
* Description    : This function handles External line 2 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI2_IRQHandler()
{
    if(EXTI_GetITStatus(EXTI_Line2) != RESET)
    {
        gotZeroPos = 1;
        // reset timer 3 value		
        TIM3->CNT = 0;

        // Clear the Key Button EXTI line pending bit
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}
