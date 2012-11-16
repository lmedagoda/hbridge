#ifndef __CURRENT_MEASUREMENT_H
#define __CURRENT_MEASUREMENT_H

#include <stdint.h>

/**
 * Initializes the hardware and the internal data structures.
 * */
void currentMeasurementInit();

/**
 * Returns the latest current measurement.
 * */
uint32_t getCurrentMeasurement();


#endif
