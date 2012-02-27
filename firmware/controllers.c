#include "pid.h"
#include "protocol.h"
#include <stdlib.h>
#include "inc/stm32f10x_type.h"
#include "inc/stm32f10x_can.h"
#include "encoder.h"
#include "state.h"
#include "position_controller.h"
#include "controllers.h"

extern volatile enum hostIDs ownHostId;
struct pid_data speedPidData;
volatile s32 lastWheelPos = 0;



struct ControllerInterface controllers[3];


void initControllers()
{
    controllers[CONTROLLER_MODE_POSITION].init = positionControllerInit;
    controllers[CONTROLLER_MODE_POSITION].reset = positionControllerReset;
    controllers[CONTROLLER_MODE_POSITION].setDebugActive = positionControllerSetDebugActive;
    controllers[CONTROLLER_MODE_POSITION].setConfiguration = positionControllerSetConfiguration;
    controllers[CONTROLLER_MODE_POSITION].step = positionControllerStep;
    controllers[CONTROLLER_MODE_POSITION].deInit = positionControllerDeInit;    
    
    //init pid structs with sane values
    initPIDStruct(&(speedPidData));
    controllers[CONTROLLER_MODE_POSITION].init();
}

void resetControllers(s32 wheelPos)
{
    lastWheelPos = wheelPos;
    resetPIDStruct(&(speedPidData));
}


void setNewSpeedPIDValues(s32 p, s32 i, s32 d, s32 minMax) {
    setKp(&(speedPidData), p);
    setKi(&(speedPidData), i);
    setKd(&(speedPidData), d);
    setMinMaxCommandVal(&(speedPidData), -minMax, minMax);   
}

void setNewPosPIDValues(s32 p, s32 i, s32 d, s32 minMax) {
    controllers[CONTROLLER_MODE_POSITION].setConfiguration(p,i,d,minMax);
}


s32 speedController(s32 targetSpeed, s32 wheelPos, u32 ticksPerTurn, u8 debug) {
    s32 pwmValue = 0;

    s32 curSpeed = wheelPos - lastWheelPos;

    //this assumes, that the motor will never turn faster than
    //a quarter wheel turn (or 12.5 motor turns) in a ms 
    if(abs(curSpeed) > ticksPerTurn / 4) {
	//wheel is turning backward
	if(lastWheelPos < wheelPos) {
	    curSpeed -= ticksPerTurn;
	}

	//wheel is turning forward
	if(lastWheelPos > wheelPos) {
	    curSpeed += ticksPerTurn;
	}
    }

    //calculate PID value
    //use input from PC-Side
    setTargetValue(&(speedPidData), targetSpeed);
    pwmValue = pid(&(speedPidData), curSpeed);

    if(debug) {
	u8 tickDivider = getTickDivider(activeCState->controllerInputEncoder);

	CanTxMsg speedDbgMessage;
        CanTxMsg pidMessageSpeed;
        speedDbgMessage.StdId= PACKET_ID_SPEED_DEBUG + ownHostId;
	speedDbgMessage.RTR=CAN_RTR_DATA;
	speedDbgMessage.IDE=CAN_ID_STD;
	speedDbgMessage.DLC= sizeof(struct speedDebugData);

        struct speedDebugData *sdbgdata = (struct speedDebugData *) speedDbgMessage.Data;
        
        sdbgdata->targetVal = targetSpeed;
	sdbgdata->pwmVal = pwmValue;
	sdbgdata->encoderVal = wheelPos / tickDivider;
	sdbgdata->speedVal = curSpeed;

	//send status message over CAN
	pidMessageSpeed.StdId= PACKET_ID_PID_DEBUG_SPEED + ownHostId;
	pidMessageSpeed.RTR=CAN_RTR_DATA;
	pidMessageSpeed.IDE=CAN_ID_STD;
	pidMessageSpeed.DLC= sizeof(struct pidDebugData);

	struct pidDebugData *sdata = (struct pidDebugData *) pidMessageSpeed.Data;
	getInternalPIDValues(&(sdata->pPart), &(sdata->iPart), &(sdata->dPart));
	sdata->minMaxPidOutput = speedPidData.max_command_val;
	
	while(CAN_Transmit(&pidMessageSpeed) == CAN_NO_MB){
	    ;
	}
	
	//send speed status message
	while(CAN_Transmit(&speedDbgMessage) == CAN_NO_MB) {
	    ;
	}
    }    

    lastWheelPos = wheelPos;
    return pwmValue;
}

s32 cascadedPositionController(s32 targetPos, s32 wheelPos, u32 ticksPerTurn, u8 debug) {
    controllers[CONTROLLER_MODE_POSITION].setDebugActive(debug);
    s32 pwmValue = controllers[CONTROLLER_MODE_POSITION].step(targetPos, wheelPos, ticksPerTurn);
    pwmValue = speedController(pwmValue / 10, wheelPos, ticksPerTurn, debug);
   
    return pwmValue;
}
