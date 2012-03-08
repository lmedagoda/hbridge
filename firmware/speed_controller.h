#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include <inc/stm32f10x_type.h>
#include "protocol.h"


struct ControllerData;

/**
 * This function is meant to set special configurations for this controller.
 * Also one should note, that this function may be called from a different
 * thread context.
 * */
void speedControllerSetControllerConfiguration(volatile struct posControllerData* data);
void speedControllerInit(void);
void speedControllerDeInit(void);
void speedControllerReset(s32 wheelPos);
void speedControllerSetDebugActive(u8 debugActive);

s32 speedControllerStep(s32 targetPos, s32 wheelPos, u32 ticksPerTurn);
void speedControllerSetConfiguration(s32 p, s32 i, s32 d, s32 minMax);



#endif