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

/**
 * Sends out an error message
 * containing the source of the error
 */
void systick_sendErrorMessage();


#endif
