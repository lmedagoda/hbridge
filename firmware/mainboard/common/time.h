#ifndef TIME_H
#define TIME_H

/**
 * Returns the time in ms since startup
 **/
unsigned int time_getTimeInMs();

/**
 * Increases the time by one ms.
 * */
void time_msPassed();

#endif