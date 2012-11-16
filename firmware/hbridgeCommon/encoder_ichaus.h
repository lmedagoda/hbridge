#ifndef ENCODER_ICHAUS_H
#define	ENCODER_ICHAUS_H

#include <stdint.h>

void encoderInitIcHaus();

void setTicksPerTurnIcHaus(uint32_t ticks, uint8_t tickDivider);
uint32_t getTicksIcHaus(void);
uint16_t getTickint16_tIcHaus(void);
uint32_t from16BitIcHaus(uint16_t const ticks);
uint16_t getDividedTicksIcHaus(void);

#endif	/* ENCODER_IcHaus_H */

