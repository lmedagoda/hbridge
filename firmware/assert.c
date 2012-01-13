#include "printf.h"
#include "inc/stm32f10x_lib.h"
#include "inc/stm32f10x_gpio.h"
#include "inc/stm32f10x_rcc.h"
#include "usart.h"

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
    USART3_DeInit();
    //do not use interrupts in assert case
    USART3_Init(DISABLE);
    
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    printf("Wrong parameters value: file %s on line %lu\n", file, line);

    GPIO_InitTypeDef GPIO_InitStructure;
  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA, ENABLE);
    
    // Configure PC.12 as output push-pull (LED)
    GPIO_WriteBit(GPIOA,GPIO_Pin_12,Bit_SET);
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    volatile int delay;
    int waittime = 500000;
    
    while(1)
    {  
	GPIO_SetBits(GPIOA, GPIO_Pin_12);
	delay = waittime;
	while(delay) {
	    delay--;
	}
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_12);
	delay = waittime;
	while(delay) {
	    delay--;
	}
    }
}