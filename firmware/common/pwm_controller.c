#include "pwm_controller.h"
#include <stdlib.h>
#include "printf.h"

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
    int32_t pwmValue = *((int16_t *) targetData->data);
//     printf("PWM !!! Got value %i \n", pwmValue);
    return pwmValue;
}