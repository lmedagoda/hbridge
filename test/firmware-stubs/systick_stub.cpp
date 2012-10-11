#include <pthread.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>

extern "C"
{
    void SysTick_Init(void);
    void SysTickHandler(void);
}

pthread_t systickThread;
void *thread_func(void *)
{
    while(true)
    {
	SysTickHandler();
	//not exact but ok
	usleep(1000);
    }
    return NULL;
}


void SysTick_Init(void)
{
    if(pthread_create(&systickThread, NULL, thread_func, NULL))
    {
	std::cout << "Error Failed to create Systick thread" << std::endl;
	exit(1);
    }

}
