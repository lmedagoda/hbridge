#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <inc/stm32f10x_type.h>
#include "protocol.h"


struct ControllerData;

/**
 * This function is meant to set special configurations for this controller.
 * Also one should note, that this function may be called from a different
 * thread context.
 * */
void positionControllerSetControllerConfiguration(volatile struct posControllerData* data);
void positionControllerInit(void);
void positionControllerDeInit(void);
void positionControllerReset(s32 wheelPos);
void positionControllerSetDebugActive(u8 debugActive);

s32 positionControllerStep(s32 targetPos, s32 wheelPos, u32 ticksPerTurn);
void positionControllerSetConfiguration(s32 p, s32 i, s32 d, s32 minMax);



#endif