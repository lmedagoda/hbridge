#include "interfaces/thread.h"
#include "stm32f10x_conf.h"

void (*thread_func)(void);

void SysTick_Handler(void) 
{    
    thread_func();
}

void startHardPeriodicThread(int frequency, void (*func)(void))
{
    thread_func = func;
    if (SysTick_Config(SystemCoreClock / frequency))
    { 
	assert_failed((uint8_t *)__FILE__, __LINE__);
    }    
}
