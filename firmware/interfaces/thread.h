#ifndef THREAD_H
#define THREAD_H

/**
 * This function must be implemented by the platform.
 * 
 * It should start a hard periodic thread, that 
 * calls the given function 'func' in the given
 * frequency.
 * 
 * @param frequency the frequency in which func should be called in herz
 * */
void startHardPeriodicThread(int frequency, void (*func)(void));


#endif