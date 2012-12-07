#include "stm32f10x.h"
#include "stm32f10x_rcc.h"

uint32_t SystemCoreClock;

void SetSysClock();

/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */ 
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x0 /*!< Vector Table base offset field. 
                                  This value must be a multiple of 0x200. */

void SystemInit (void)
{
    //RCC system reset(for debug purpose)
    RCC_DeInit();


    //Turn on crystal oscillator
    RCC_HSEConfig(RCC_HSE_ON);
    while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
	;

    //Configure flash timing
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    FLASH_SetLatency(FLASH_Latency_2);    // 2 wait states

    //Configure clocks
    RCC_HCLKConfig(RCC_SYSCLK_Div1);      // HCLK = SYSCLK
    RCC_PCLK1Config(RCC_HCLK_Div2);       // PCLK1 = HCLK/2
    RCC_PCLK2Config(RCC_HCLK_Div1);       // PCLK2 = HCLK
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);     // ADCCLK = PCLK2/6 = 12Mhz

    //Configure PLL for 72 MHz
    RCC_PLLConfig(RCC_PLLSource_HSE_Div2, RCC_PLLMul_9); // 16 MHz / 2 * 9 = 72 MHz
    RCC_PLLCmd(ENABLE);
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	;

    //Select PLL for SYSCLK source
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    while(RCC_GetSYSCLKSource() != 0x08)
	;
}


void SetSysClock()
{
    /* Select HSI as clock source */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_HSI);

    /* Enable external crystal oscillator*/
    CLEAR_BIT(RCC->CR, RCC_CR_HSEON);
    SET_BIT(RCC->CR, RCC_CR_HSEON);
    while(! (RCC->CR & RCC_CR_HSERDY));

    /* Enable Prefetch Buffer */
    SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);
    /* Flash 2 wait state */
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2);

    /* set divider of HSE input if used as PLL source */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLXTPRE, RCC_CFGR_PLLXTPRE_HSE_Div2);
    /* set AHB prescaler to generate HCLK from SYSCLK */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);
    /* APB2 prescaler: PCLK2 = HCLK */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV1);
    /* APB1 prescaler: PCLK1 may not exceed 36 MHz! */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV2);
    /* ADC prescaler: ADCCLK may not exceed 12 MHz! */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_ADCPRE, RCC_CFGR_ADCPRE_DIV6);


    /* PLL disable before configuration */
    CLEAR_BIT(RCC->CR, RCC_CR_PLLON);
    /* PLL source: external clock (HSE) */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLSRC, RCC_CFGR_PLLSRC_HSE);
    /* PLL-Output := (HSE_VALUE / 2) * 12 */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PLLMULL, RCC_CFGR_PLLMULL9);

    /* enable PLL and wait for stable output */
    SET_BIT(RCC->CR, RCC_CR_PLLON);
    while(! (RCC->CR & RCC_CR_PLLRDY));
    /* select pll output as system clock source */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_CFGR_SW_PLL);

    while((RCC->CFGR & 0x0000000C) != 0x08)

    /* store frequency of system clock in global variable */
    SystemCoreClock = (HSE_VALUE / 2) * 9;

}
