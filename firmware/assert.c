#include "printf.h"
#include "inc/stm32f10x_lib.h"
#include "inc/stm32f10x_gpio.h"
#include "inc/stm32f10x_rcc.h"
#include "usart.h"
#include "assert.h"

GPIO_TypeDef* assertGpio;
u16 assertGpioPin;
enum ASSERT_UASRT assertUsart;


void Assert_Init(GPIO_TypeDef* GPIOx, u16 GPIO_Pin, enum ASSERT_UASRT used_usart)
{
    assertGpio = GPIOx;
    assertGpioPin = GPIO_Pin;
    assertUsart = used_usart;
}


/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{
    switch(assertUsart)
    {
	case USE_USART1:
	    USART1_DeInit();
	    //do not use interrupts in assert case
	    USART1_Init(DISABLE);
	    break;
	case USE_USART3:
	    USART3_DeInit();
	    //do not use interrupts in assert case
	    USART3_Init(DISABLE);
	    break;
    }
    
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    printf("Wrong parameters value: file %s on line %lu\n", file, line);

    GPIO_InitTypeDef GPIO_InitStructure;
  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA, ENABLE);
    
    // Configure PC.12 as output push-pull (LED)
    GPIO_WriteBit(assertGpio,assertGpioPin,Bit_SET);
    GPIO_InitStructure.GPIO_Pin =  assertGpioPin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(assertGpio, &GPIO_InitStructure);
    
    volatile int delay;
    int waittime = 500000;
    
    while(1)
    {  
	GPIO_SetBits(assertGpio, assertGpioPin);
	delay = waittime;
	while(delay) {
	    delay--;
	}
	
	GPIO_ResetBits(assertGpio, assertGpioPin);
	delay = waittime;
	while(delay) {
	    delay--;
	}
    }
}

#define printDebug(debugString) \
    switch(assertUsart) \
    {\
	case USE_USART1:\
	    USART1_DeInit();\
	    USART1_Init(DISABLE); \
	    break;\
	case USE_USART3:\
	    USART3_DeInit();\
	    USART3_Init(DISABLE);\
	    break;\
    }\
    print(#debugString "\n");

void NMIException(void)
{
    printDebug(NMIException)
    assert_failed((u8 *)__FILE__, __LINE__);
}

void HardFaultException(void)
{
    printDebug(HardFaultException)
    assert_failed((u8 *)__FILE__, __LINE__);
}

void MemManageException(void)
{
    printDebug(MemManageException)
    assert_failed((u8 *)__FILE__, __LINE__);
}

void BusFaultException(void)
{
    printDebug(BusFaultException)
    assert_failed((u8 *)__FILE__, __LINE__);
}

void UsageFaultException(void)
{
    printDebug(UsageFaultException)
    assert_failed((u8 *)__FILE__, __LINE__);
}


