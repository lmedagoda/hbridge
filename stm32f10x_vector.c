/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_vector.c
* Author             : MCD Application Team
* Version            : V2.0
* Date               : 05/23/2008
* Description        : STM32F10x vector table for RIDE7 toolchain.
*                      This module performs:
*                      - Set the initial SP
*                      - Set the initial PC == Reset_Handler,
*                      - Set the vector table entries with the exceptions ISR address,
*                      - Configure external SRAM mounted on STM3210E-EVAL board
*                       to be used as data memory (optional, to be enabled by user)
*                      - Branches to main in the C library (which eventually
*                        calls main()).
*                      After Reset the Cortex-M3 processor is in Thread mode,
*                      priority is Privileged, and the Stack is set to Main.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED 
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_it.h"

/* Private typedef -----------------------------------------------------------*/
typedef void( *intfunc )( void );
typedef union { intfunc __fun; void * __ptr; } intvec_elem;

/* Private macro -------------------------------------------------------------*/
#define NVIC_CCR ((volatile unsigned long *)(0xE000ED14))

/* end address for the .text section. defined in linker script */		
extern unsigned long _etext;

/* start address for the .data section. defined in linker script */		
extern unsigned long _data;

/* end address for the .data section. defined in linker script */		
extern unsigned long _edata;
		
/* start address for the .bss section. defined in linker script */
extern unsigned long _bss;

/* end address for the .bss section. defined in linker script */			
extern unsigned long _ebss;	
		
/* init value for the stack pointer. defined in linker script */
extern void _estack;	
	
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Reset_Handler(void) __attribute__((__interrupt__));
extern int main(void);
/* Private functions ---------------------------------------------------------*/

__attribute__ ((section(".vectors")))
void (* const g_pfnVectors[])(void) =
{
  &_estack,            /* The initial stack pointer*/
  Reset_Handler,             /* The reset handler*/
  NMIException,
  HardFaultException,
  MemManageException,
  BusFaultException,
  UsageFaultException,
  0, 0, 0, 0,            /* Reserved */ 
  SVCHandler,
  DebugMonitor,
  0,                      /* Reserved */
  PendSVC,
  SysTickHandler,
  WWDG_IRQHandler,
  PVD_IRQHandler,
  TAMPER_IRQHandler,
  RTC_IRQHandler,
  FLASH_IRQHandler,
  RCC_IRQHandler,
  EXTI0_IRQHandler,
  EXTI1_IRQHandler,
  EXTI2_IRQHandler,
  EXTI3_IRQHandler,
  EXTI4_IRQHandler,
  DMA1_Channel1_IRQHandler,
  DMA1_Channel2_IRQHandler,
  DMA1_Channel3_IRQHandler,
  DMA1_Channel4_IRQHandler,
  DMA1_Channel5_IRQHandler,
  DMA1_Channel6_IRQHandler,
  DMA1_Channel7_IRQHandler,
  ADC1_2_IRQHandler,
  USB_HP_CAN_TX_IRQHandler,
  USB_LP_CAN_RX0_IRQHandler,
  CAN_RX1_IRQHandler,
  CAN_SCE_IRQHandler,
  EXTI9_5_IRQHandler,
  TIM1_BRK_IRQHandler,
  TIM1_UP_IRQHandler,
  TIM1_TRG_COM_IRQHandler,
  TIM1_CC_IRQHandler,
  TIM2_IRQHandler,
  TIM3_IRQHandler,
  TIM4_IRQHandler,
  I2C1_EV_IRQHandler,
  I2C1_ER_IRQHandler,
  I2C2_EV_IRQHandler,
  I2C2_ER_IRQHandler,
  SPI1_IRQHandler,
  SPI2_IRQHandler,
  USART1_IRQHandler,
  USART2_IRQHandler,
  USART3_IRQHandler,
  EXTI15_10_IRQHandler,
  RTCAlarm_IRQHandler,
  USBWakeUp_IRQHandler,
  TIM8_BRK_IRQHandler,
  TIM8_UP_IRQHandler,
  TIM8_TRG_COM_IRQHandler,
  TIM8_CC_IRQHandler,
  ADC3_IRQHandler,
  FSMC_IRQHandler,
  SDIO_IRQHandler,
  TIM5_IRQHandler,
  SPI3_IRQHandler,
  UART4_IRQHandler,
  UART5_IRQHandler,
  TIM6_IRQHandler,
  TIM7_IRQHandler,
  DMA2_Channel1_IRQHandler,
  DMA2_Channel2_IRQHandler,
  DMA2_Channel3_IRQHandler,
  DMA2_Channel4_5_IRQHandler,  
};



void cpu_init()
{
  *NVIC_CCR = *NVIC_CCR | 0x200;	/* Set STKALIGN in NVIC */
  
  //RCC system reset(for debug purpose)
  RCC_DeInit();


  //Turn on crystal oscillator
  RCC_HSEConfig(RCC_HSE_ON);
  while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);

  //Configure flash timing
  FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
  FLASH_SetLatency(FLASH_Latency_2);	// 2 wait states

  //Configure clocks
  RCC_HCLKConfig(RCC_SYSCLK_Div1);	// HCLK = SYSCLK
  RCC_PCLK1Config(RCC_HCLK_Div2);	// PCLK1 = HCLK/2
  RCC_PCLK2Config(RCC_HCLK_Div1);	// PCLK2 = HCLK

  //Configure PLL for 72 MHz
  RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); // 8 MHz * 9 = 72 MHz
  RCC_PLLCmd(ENABLE);
  while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

  //Select PLL for SYSCLK source
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  while(RCC_GetSYSCLKSource() != 0x08);
}

/*******************************************************************************
* Function Name  : Reset_Handler
* Description    : This is the code that gets called when the processor first
*                  starts execution following a reset event. Only the absolutely
*                  necessary set is performed, after which the application
*                  supplied main() routine is called. 
* Input          :
* Output         :
* Return         :
*******************************************************************************/
void Reset_Handler(void)
{
unsigned long *pulSrc, *pulDest;

/* Copy the data segment initializers from flash to SRAM */
    pulSrc = &_etext;
    for(pulDest = &_data; pulDest < &_edata; )
    {
        *(pulDest++) = *(pulSrc++);
    }
/* Zero fill the bss segment.  */
    for(pulDest = &_bss; pulDest < &_ebss; )
    {
        *(pulDest++) = 0;
    }

/* init cpu at 72 mhz */
    cpu_init();

/* Call the application's entry point.*/
    main();
}


/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/


