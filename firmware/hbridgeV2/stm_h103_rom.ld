

MEMORY
{
  RAM 	(rwx) : ORIGIN = 0x20000000, LENGTH = 20K
  FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 128K
}

/* inital stack pointer at end of memory */
_estack =  0x20005000;
SECTIONS
{
    .  = 0x00000000;          /* From 0x00000000 */
    .isr_vector :
    {
	. = ALIGN(4);
        KEEP(*(.isr_vector))            /* Startup code */
	. = ALIGN(4);
    } >FLASH
/*  */
  .text : {
    . = ALIGN(4);
    *(.text)        /* Program code */
    *(.text.*)                   /* remaining code */
    *(.rodata)      /* Read only data */
    *(.rodata*)      /* Read only data */
    . = ALIGN(4);
    _etext = .;
    _sidata = _etext;
    } > FLASH

    .data  : AT ( _sidata )
    {
	. = ALIGN(4);
        /* This is used by the startup in order to initialize the .data secion */
        _sdata = . ;
        
        *(.data)
        *(.data.*)

	. = ALIGN(4);
	/* This is used by the startup in order to initialize the .data secion */
	_edata = . ;
    } >RAM

    /* This is the uninitialized data section */
    .bss :
    {
	    . = ALIGN(4);
        /* This is used by the startup in order to initialize the .bss secion */
        _sbss = .;
        
        *(.bss)
        *(COMMON)
        
	    . = ALIGN(4);
	    /* This is used by the startup in order to initialize the .bss secion */
   	 _ebss = . ;
    } >RAM
 }  

/*  Libraries to link against. */

/*INPUT( -lstm32fw )*/

/*  Provide a default vector for any unhandled interrupts. */

PROVIDE(NMIException = _default_handler);
PROVIDE(HardFaultException = _default_handler);
PROVIDE(MemManageException = _default_handler);
PROVIDE(BusFaultException = _default_handler);
PROVIDE(UsageFaultException = _default_handler);
PROVIDE(DebugMonitor = _default_handler);
PROVIDE(SVCHandler = _default_handler);
PROVIDE(PendSVC = _default_handler);
PROVIDE(SysTickHandler = _default_handler);
PROVIDE(WWDG_IRQHandler = _default_handler);
PROVIDE(PVD_IRQHandler = _default_handler);
PROVIDE(TAMPER_IRQHandler = _default_handler);
PROVIDE(RTC_IRQHandler = _default_handler);
PROVIDE(FLASH_IRQHandler = _default_handler);
PROVIDE(RCC_IRQHandler = _default_handler);
PROVIDE(EXTI0_IRQHandler = _default_handler);
PROVIDE(EXTI1_IRQHandler = _default_handler);
PROVIDE(EXTI2_IRQHandler = _default_handler);
PROVIDE(EXTI3_IRQHandler = _default_handler);
PROVIDE(EXTI4_IRQHandler = _default_handler);
PROVIDE(DMA1_Channel1_IRQHandler = _default_handler);
PROVIDE(DMA1_Channel2_IRQHandler = _default_handler);
PROVIDE(DMA1_Channel3_IRQHandler = _default_handler);
PROVIDE(DMA1_Channel4_IRQHandler = _default_handler);
PROVIDE(DMA1_Channel5_IRQHandler = _default_handler);
PROVIDE(DMA1_Channel6_IRQHandler = _default_handler);
PROVIDE(DMA1_Channel7_IRQHandler = _default_handler);
PROVIDE(ADC1_2_IRQHandler = _default_handler);
PROVIDE(USB_HP_CAN_TX_IRQHandler = _default_handler);
PROVIDE(USB_LP_CAN_RX0_IRQHandler = _default_handler);
PROVIDE(CAN_RX1_IRQHandler = _default_handler);
PROVIDE(CAN_SCE_IRQHandler = _default_handler);
PROVIDE(EXTI9_5_IRQHandler = _default_handler);
PROVIDE(TIM1_BRK_IRQHandler = _default_handler);
PROVIDE(TIM1_UP_IRQHandler = _default_handler);
PROVIDE(TIM1_TRG_COM_IRQHandler = _default_handler);
PROVIDE(TIM1_CC_IRQHandler = _default_handler);
PROVIDE(TIM2_IRQHandler = _default_handler);
PROVIDE(TIM3_IRQHandler = _default_handler);
PROVIDE(TIM4_IRQHandler = _default_handler);
PROVIDE(I2C1_EV_IRQHandler = _default_handler);
PROVIDE(I2C1_ER_IRQHandler = _default_handler);
PROVIDE(I2C2_EV_IRQHandler = _default_handler);
PROVIDE(I2C2_ER_IRQHandler = _default_handler);
PROVIDE(SPI1_IRQHandler = _default_handler);
PROVIDE(SPI2_IRQHandler = _default_handler);
PROVIDE(USART1_IRQHandler = _default_handler);
PROVIDE(USART2_IRQHandler = _default_handler);
PROVIDE(USART3_IRQHandler = _default_handler);
PROVIDE(EXTI15_10_IRQHandler = _default_handler);
PROVIDE(RTCAlarm_IRQHandler = _default_handler);
PROVIDE(USBWakeUp_IRQHandler = _default_handler);
PROVIDE(TIM8_BRK_IRQHandler = _default_handler);
PROVIDE(TIM8_UP_IRQHandler = _default_handler);
PROVIDE(TIM8_TRG_COM_IRQHandler = _default_handler);
PROVIDE(TIM8_CC_IRQHandler = _default_handler);
PROVIDE(ADC3_IRQHandler = _default_handler);
PROVIDE(FSMC_IRQHandler = _default_handler);
PROVIDE(SDIO_IRQHandler = _default_handler);
PROVIDE(TIM5_IRQHandler = _default_handler);
PROVIDE(SPI3_IRQHandler = _default_handler);
PROVIDE(UART4_IRQHandler = _default_handler);
PROVIDE(UART5_IRQHandler = _default_handler);
PROVIDE(TIM6_IRQHandler = _default_handler);
PROVIDE(TIM7_IRQHandler = _default_handler);
PROVIDE(DMA2_Channel1_IRQHandler = _default_handler);
PROVIDE(DMA2_Channel2_IRQHandler = _default_handler);
PROVIDE(DMA2_Channel3_IRQHandler = _default_handler);
PROVIDE(DMA2_Channel4_5_IRQHandler = _default_handler);

EXTERN(g_pfnVectors _default_handler);

/*========== end of file ==========*/
