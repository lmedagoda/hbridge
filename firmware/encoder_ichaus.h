#ifndef ENCODER_ICHAUS_H
#define	ENCODER_ICHAUS_H

#include "inc/stm32f10x_type.h"

void encoderInitIcHaus();

void setTicksPerTurnIcHaus(u32 ticks, u8 tickDivider);
u32 getTicksIcHaus(void);
u16 getTicks16IcHaus(void);
u32 from16BitIcHaus(u16 const ticks);
u16 getDividedTicksIcHaus(void);

#endif	/* ENCODER_IcHaus_H */

