#ifndef __CONTROLLERS_H
#define __CONTROLLERS_H

#include "inc/stm32f10x_type.h"
#include "pid.h"

struct ControllerData {
    struct pid_data pidData;
    s32 lastWheelPos;
    u8 debugActive;
};

struct ControllerInterface {
    void (*init) (void);
    void (*reset) (s32 wheelPos);
    void (*setDebugActive) (u8 debugActive);
    void (*setConfiguration) (s32 p, s32 i, s32 d, s32 minMax);
    s32 (*step) (s32 targetPos, s32 wheelPos, u32 ticksPerTurn);
    void (*deInit) (void);
};

#define NUM_CONTROLLERS 3

extern struct ControllerInterface controllers[NUM_CONTROLLERS];

/**
* This function initializes internal values of the controllers
**/
void initControllers();

/**
* This function sould be called, if the h-bridge goes into an
* error state. In this function the internal data of the 
* controllers should be reset, so that no quirks occure on
* reactivation, but configuration data should be kept.
**/
void resetControllers(s32 wheelPos);


void setNewSpeedPIDValues(s32 p, s32 i, s32 d, s32 minMax);
void setNewPosPIDValues(s32 p, s32 i, s32 d, s32 minMax);
s32 positionController(s32 targetPos, s32 wheelPos, u32 ticksPerTurn, u8 debug);
s32 speedController(s32 targetSpeed, s32 wheelPos, u32 ticksPerTurn, u8 debug);
s32 cascadedPositionController(s32 targetPos, s32 wheelPos, u32 ticksPerTurn, u8 debug);

#endif
