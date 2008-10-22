#ifndef __ADC_H
#define __ADC_H

#define USED_REGULAR_ADC_CHANNELS 15

extern vu16 adc_values[USED_REGULAR_ADC_CHANNELS];


/**
 * This function programms the watchdog to be
 * triggered by TI CH3. It also selects the 
 * channel to be guarded in respect to the
 * direction the motor turns.
 */
void configureWatchdog(vu8 dir);

/**
 * This function configures the channel from which
 * the current measurement is taken, in respect of
 * the direction in which the motor turns.
 */
void configureCurrentMeasurement(vu8 dir);


#endif
