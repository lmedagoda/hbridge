#include "interfaces/thread.h"
#include "stm32f10x_conf.h"

void (*thread_func)(void);

void SysTickHandler(void) {
    thread_func();
}

void startHardPeriodicThread(int frequency, void (*func)(void))
{
    if (SysTick_Config(SystemCoreClock / 1000))
    { 
	assert_failed((uint8_t *)__FILE__, __LINE__);
    }    
}
