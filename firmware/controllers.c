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

void defaultControllerSetConfiguration(s32 p, s32 i, s32 d, s32 minMax)
{
}

void defaultControllerSetDebugActive(u8 debugActive)
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
    cont->setDebugActive = defaultControllerSetDebugActive;
    cont->setConfiguration = defaultControllerSetConfiguration;
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
    controllers[CONTROLLER_MODE_POSITION].setDebugActive = positionControllerSetDebugActive;
    controllers[CONTROLLER_MODE_POSITION].setConfiguration = positionControllerSetConfiguration;
    controllers[CONTROLLER_MODE_POSITION].step = positionControllerStep;
    controllers[CONTROLLER_MODE_POSITION].deInit = positionControllerDeInit;    
    
    controllers[CONTROLLER_MODE_SPEED].init = speedControllerInit;
    controllers[CONTROLLER_MODE_SPEED].reset = speedControllerReset;
    controllers[CONTROLLER_MODE_SPEED].setDebugActive = speedControllerSetDebugActive;
    controllers[CONTROLLER_MODE_SPEED].setConfiguration = speedControllerSetConfiguration;
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

void setNewPosPIDValues(s32 p, s32 i, s32 d, s32 minMax) {
    controllers[CONTROLLER_MODE_POSITION].setConfiguration(p,i,d,minMax);
}

void setNewSpeedPIDValues(s32 p, s32 i, s32 d, s32 minMax) {
    controllers[CONTROLLER_MODE_SPEED].setConfiguration(p,i,d,minMax);
}

s32 cascadedPositionController(s32 targetPos, s32 wheelPos, u32 ticksPerTurn, u8 debug) {
    controllers[CONTROLLER_MODE_POSITION].setDebugActive(debug);
    s32 pwmValue = controllers[CONTROLLER_MODE_POSITION].step(targetPos, wheelPos, ticksPerTurn);
    controllers[CONTROLLER_MODE_SPEED].setDebugActive(debug);
    pwmValue = controllers[CONTROLLER_MODE_SPEED].step(pwmValue / 10, wheelPos, ticksPerTurn);
   
    return pwmValue;
}
