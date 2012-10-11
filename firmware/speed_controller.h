#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include <stdint.h>
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
void speedControllerReset(int32_t wheelPos);
void speedControllerSetDebugActive(uint8_t debugActive);

int32_t speedControllerStep(int32_t targetPos, int32_t wheelPos, uint32_t ticksPerTurn);
void speedControllerSetConfiguration(int32_t p, int32_t i, int32_t d, int32_t minMax);



#endif