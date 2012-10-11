#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <stdint.h>
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
void positionControllerReset(int32_t wheelPos);
void positionControllerSetDebugActive(uint8_t debugActive);

int32_t positionControllerStep(int32_t targetPos, int32_t wheelPos, uint32_t ticksPerTurn);
void positionControllerSetConfiguration(int32_t p, int32_t i, int32_t d, int32_t minMax);

int32_t cascadedPositionController(int32_t targetPos, int32_t wheelPos, uint32_t ticksPerTurn);


#endif