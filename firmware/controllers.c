#include "pid.h"
#include "protocol.h"
#include <stdlib.h>
#include "inc/stm32f10x_type.h"
#include "inc/stm32f10x_can.h"
#include "encoder.h"
#include "state.h"
#include "position_controller.h"
#include "speed_controller.h"
#include "controllers.h"

struct ControllerInterface controllers[NUM_CONTROLLERS];

void defaultControllerInit(void )
{
}

void defaultControllerDeInit(void )
{
}

void defaultControllerReset(s32 wheelPos)
{
}

s32 defaultControllerStep(s32 targetSpeed, s32 wheelPos, u32 ticksPerTurn)
{
    return 0;
}

void defaultInit(struct ControllerInterface *cont)
{
    cont->init = defaultControllerInit;
    cont->reset = defaultControllerReset;
    cont->step = defaultControllerStep;
    cont->deInit = defaultControllerDeInit;
}

void initControllers()
{
    int i;
    for(i = 0; i < NUM_CONTROLLERS; i++)
    {
	defaultInit(controllers + i);
    }
    
    controllers[CONTROLLER_MODE_POSITION].init = positionControllerInit;
    controllers[CONTROLLER_MODE_POSITION].reset = positionControllerReset;
    controllers[CONTROLLER_MODE_POSITION].step = positionControllerStep;
    controllers[CONTROLLER_MODE_POSITION].deInit = positionControllerDeInit;    
    
    controllers[CONTROLLER_MODE_SPEED].init = speedControllerInit;
    controllers[CONTROLLER_MODE_SPEED].reset = speedControllerReset;
    controllers[CONTROLLER_MODE_SPEED].step = speedControllerStep;
    controllers[CONTROLLER_MODE_SPEED].deInit = speedControllerDeInit;            
    
    //init pid structs with sane values
    for(i = 0; i < NUM_CONTROLLERS; i++)
    {
	controllers[i].init();
    }
}

void resetControllers(s32 wheelPos)
{
    int i;
    for(i = 0; i < NUM_CONTROLLERS; i++)
    {
	controllers[i].reset(wheelPos);
    }
}

