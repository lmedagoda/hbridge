#ifndef WATCHDOG_H
#define WATCHDOG_H

//Connect to Systick_Handler for a clock tick
void watchdog_tick();

//Init function for setting the max counter value where the watchdog exception func
//gets called
void watchdog_init(unsigned int timer_max);

void watchdogHandler();

//Set the function which gets called, when counter > MAX_TIME
void watchdog_registerWatchDogFunc(void (*except_func)(void));

//Reset the watchdog counter to zero
void watchdog_reset();

#endif /* end of include guard: WATCHDOG_H */
