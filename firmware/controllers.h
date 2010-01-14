#ifndef __CONTROLLERS_H
#define __CONTROLLERS_H

#include "inc/stm32f10x_type.h"

/**
* This function initalizes internal values of the controllers
**/
void initControllers();

/**
* This function sould be called, if the h-bridge goes into an
* error state. In this function the internal data of the 
* controllers should be reseted, so that no quirks occure on
* reactivation, but configurationd data should be kept.
**/
void resetControllers();


void setNewSpeedPIDValues(s32 p, s32 i, s32 d, s32 minMax);
void setNewPosPIDValues(s32 p, s32 i, s32 d, s32 minMax);
s32 positionController(s32 targetPos, s32 wheelPos, u32 ticksPerTurn, u8 debug);
s32 speedController(s32 targetSpeed, s32 wheelPos, u32 ticksPerTurn, u8 debug);
s32 cascadedPositionController(s32 targetPos, s32 wheelPos, u32 ticksPerTurn, u8 debug);

#endif
