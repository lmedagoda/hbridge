#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <stdint.h>
#include "protocol.h"
#include "../interfaces/controllers.h"

void positionControllerInit(void);
void positionControllerDeInit(void);
void positionControllerReset(int32_t wheelPos);
uint8_t positionControllerIsConfigured();

int32_t positionControllerStep(struct ControllerTargetData *targetData, int32_t wheelPos, uint32_t ticksPerTurn);

int32_t cascadedPositionController(struct ControllerTargetData *targetData, int32_t wheelPos, uint32_t ticksPerTurn);


#endif