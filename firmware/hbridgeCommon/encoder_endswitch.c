#include "encoder_endswitch.h"
#include "inc/stm32f10x_rcc.h"
#include "inc/stm32f10x_gpio.h"
#include "core_cm3.h"

void endSwitchEncoder_Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    //get default GPIO config
    GPIO_StructInit(&GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
    //configure PB6  / PB 7 as end switch input
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6, GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);	
}

void endSwitchEncoder_setTicksPerTurn(uint32_t ticks, uint8_t tickDivider)
{
    
}

uint32_t endSwitchEncoder_getTicks(void )
{
    static uint32_t lastRet = 0;
    uint32_t ret = 1;
    if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6))
	ret = 0;
    if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7))
	ret = 2;
    
    if(ret != lastRet)
    {
	lastRet = ret;
	printf("End switch changed to %i \n", ret);
    }
    return ret;
}

void endSwitchEncoder_encoderDeInit(void )
{
    //nothing to do
    //switching off GPIOB RCC may influence other devices
    //therefore we don't do it
}
