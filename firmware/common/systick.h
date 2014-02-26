#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "protocol.h"

/**
 * Initializes and allocates all internal data structures
 * This function MUST be called first
 * */
void baseInit();


void platformInit();

void run();



#endif
