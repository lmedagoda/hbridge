#include <pthread.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>

extern "C"
{
#include "../interfaces/thread.h"
}

void (*systickFunc)(void);
int frequency;
pthread_t systickThread;
void *thread_func(void *)
{
    while(true)
    {
	systickFunc();
	
	//not exact but ok
	usleep(1000000 / frequency);
    }
    return NULL;
}

void startHardPeriodicThread(int freq, void (*func)(void))
{
    systickFunc = func;
    frequency = freq;

    if(pthread_create(&systickThread, NULL, thread_func, NULL))
    {
	std::cout << "Error Failed to create Systick thread" << std::endl;
	exit(1);
    }
}