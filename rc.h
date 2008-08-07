#ifndef __RC_H
#define __RC_H

#include "stm32f10x_type.h"


#define EDGETIMEBUFFER_SIZE 256

extern vu16 edgeTimeBuffer[EDGETIMEBUFFER_SIZE];
extern vu16 etbRead;
extern vu16 etbWrite;

#define MAX_RC_CHANNELS 8

extern volatile s32 rc_values[MAX_RC_CHANNELS];

void updateRCValues();

#endif
