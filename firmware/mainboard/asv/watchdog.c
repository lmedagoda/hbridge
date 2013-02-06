/**
 * Watchdog implementation into the SystickInterrupt of the STM32
 */

#include "watchdog.h"

//watchdog Counter, will be increased if a systick arives
static unsigned int counter = 0;
static unsigned int max_counter = 0;

//Will be called, when watchdog_reset() was not called for > MAX_TIME
static void (*exception_func)(void) = 0;


static void exception_wrapper() {
	printf("Watchdog Exception!!!\n");
	if(exception_func != 0) {
		exception_func();
	}
}

//Init function for setting the max counter value where the watchdog exception func
//gets called
void watchdog_init(unsigned int timer_max) {
	max_counter = timer_max;
}

//Connect to Systick_Handler for a clock tick
void watchdog_tick() {
	++counter;
	if(counter > max_counter)
	{
		exception_wrapper();
	}
}

//Set the function which gets called, when counter > MAX_TIME
void watchdog_registerWatchDogFunc(void (*except_func)(void)) {
	exception_func = except_func;
}

//Reset the systick counter to zero
void watchdog_reset() {
	counter = 0;
}

