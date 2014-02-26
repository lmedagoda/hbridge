#include "protocol.h"
#include <stdlib.h>
#include "state.h"
#include "position_controller.h"
#include "speed_controller.h"
#include "pwm_controller.h"
#include "../interfaces/controllers.h"

struct ControllerInterface controllers[NUM_CONTROLLERS];

void defaultControllerInit(void )
{
}

void defaultControllerDeInit(void )
{
}

void defaultControllerReset(int32_t wheelPos)
{
}

uint8_t defaultControllerIsConfigured()
{
    return 1;
}

int32_t defaultControllerStep(struct ControllerTargetData *targetData, int32_t wheelPos, uint32_t ticksPerTurn)
{
    return 0;
}

void defaultInit(struct ControllerInterface *cont)
{
    cont->init = defaultControllerInit;
    cont->reset = defaultControllerReset;
    cont->isConfigured = defaultControllerIsConfigured;
    cont->step = defaultControllerStep;
    cont->deInit = defaultControllerDeInit;
}

void controllers_init()
{
    int i;
    for(i = 0; i < NUM_CONTROLLERS; i++)
    {
	defaultInit(controllers + i);
    }
    
    controllers[CONTROLLER_MODE_PWM].init = pwmControllerInit;
    controllers[CONTROLLER_MODE_PWM].reset = pwmControllerReset;
    controllers[CONTROLLER_MODE_PWM].isConfigured = pwmControllerIsConfigured;
    controllers[CONTROLLER_MODE_PWM].step = pwmControllerStep;
    controllers[CONTROLLER_MODE_PWM].deInit = pwmControllerDeInit;
    
    controllers[CONTROLLER_MODE_POSITION].init = positionControllerInit;
    controllers[CONTROLLER_MODE_POSITION].reset = positionControllerReset;
    controllers[CONTROLLER_MODE_POSITION].isConfigured = positionControllerIsConfigured;
    controllers[CONTROLLER_MODE_POSITION].step = positionControllerStep;
    controllers[CONTROLLER_MODE_POSITION].deInit = positionControllerDeInit;    
    
    controllers[CONTROLLER_MODE_SPEED].init = speedControllerInit;
    controllers[CONTROLLER_MODE_SPEED].reset = speedControllerReset;
    controllers[CONTROLLER_MODE_SPEED].isConfigured = speedControllerIsConfigured;
    controllers[CONTROLLER_MODE_SPEED].step = speedControllerStep;
    controllers[CONTROLLER_MODE_SPEED].deInit = speedControllerDeInit;            
    
    //init pid structs with sane values
    for(i = 0; i < NUM_CONTROLLERS; i++)
    {
	controllers[i].init();
    }
}

uint8_t controllers_areControllersConfigured()
{
    int i;
    for(i = 0; i < NUM_CONTROLLERS; i++)
    {
	if(!controllers[i].isConfigured())
	    return 0;
    }
    return 1;
}

void controllers_resetControllers(int32_t wheelPos)
{
    int i;
    for(i = 0; i < NUM_CONTROLLERS; i++)
    {
	controllers[i].reset(wheelPos);
    }
}

const struct ControllerInterface *controllers_getController(uint8_t ctrlID)
{
    if(ctrlID >= NUM_CONTROLLERS)
	return NULL;
    
    return &(controllers[ctrlID]);
}
