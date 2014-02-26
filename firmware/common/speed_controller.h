#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include <stdint.h>
#include "protocol.h"
#include "../interfaces/controllers.h"

void speedControllerInit(void);
void speedControllerDeInit(void);
void speedControllerReset(int32_t wheelPos);
uint8_t speedControllerIsConfigured();

int32_t speedControllerStep(struct ControllerTargetData *targetData, int32_t wheelPos, uint32_t ticksPerTurn);



#endif