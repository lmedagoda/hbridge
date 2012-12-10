#ifndef PWM_CONTROLLER_H
#define PWM_CONTROLLER_H

#include <stdint.h>
#include "../interfaces/controllers.h"

void pwmControllerInit(void);
void pwmControllerDeInit(void);
void pwmControllerReset(int32_t wheelPos);
uint8_t pwmControllerIsConfigured();

int32_t pwmControllerStep(struct ControllerTargetData *targetData, int32_t wheelPos, uint32_t ticksPerTurn);



#endif