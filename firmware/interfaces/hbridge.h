#ifndef __HBRIDGE_H
#define __HBRIDGE_H

#include <stdint.h>

/**
 * Initializes the hardware and the internal data structures
 * */
void hbridgeInit();

/**
 * Applies the giben PWM (in the range from -1800 to 1800) to
 * the motor PWM generator.
 *
 * The second parameter determines if (in the case of a 2 phase 
 * DC motor) the lower side of the hbridge should be closed during
 * the off phase of the PWM.
 *  
 * */
void setNewPWM(const int16_t value2, uint8_t useOpenCircuit);

#endif
