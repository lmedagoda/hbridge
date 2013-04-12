#include "pwm_controller.h"
#include <stdlib.h>
#include "printf.h"
#include <limits.h>

uint8_t pwmControllerIsConfigured()
{
    return 1;
}

void pwmControllerInit(void )
{
}

void pwmControllerDeInit(void )
{

}

void pwmControllerReset(int32_t wheelPos)
{
}

int32_t pwmControllerStep(struct ControllerTargetData *targetData, int32_t wheelPos, uint32_t ticksPerTurn)
{
    int16_t *pwmValue = (int16_t *) targetData->data;
    //printf("Setting pwm value %hi\n", *pwmValue);
    return *pwmValue;
}
