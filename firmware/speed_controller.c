#include "speed_controller.h"
#include "pid.h"
#include "controllers.h"
#include "encoder.h"
#include "state.h"
#include <stdlib.h>

extern volatile enum hostIDs ownHostId;

struct SpeedControllerConfig {
};

struct ControllerData speedControllerData;

void speedControllerInit(void )
{
    speedControllerData.debugActive = 0;
    speedControllerReset(0);
}

void speedControllerDeInit(void )
{

}

void speedControllerReset(s32 wheelPos)
{
    speedControllerData.lastWheelPos = wheelPos;
    resetPIDStruct(&(speedControllerData.pidData));
}

void speedControllerSetConfiguration(s32 p, s32 i, s32 d, s32 minMax)
{
    setKp(&(speedControllerData.pidData), p);
    setKi(&(speedControllerData.pidData), i);
    setKd(&(speedControllerData.pidData), d);
    setMinMaxCommandVal(&(speedControllerData.pidData), -minMax, minMax);
}

void speedControllerSetDebugActive(u8 debugActive)
{
    speedControllerData.debugActive = debugActive;
}

s32 speedControllerStep(s32 targetSpeed, s32 wheelPos, u32 ticksPerTurn)
{
    s32 pwmValue = 0;

    s32 curSpeed = wheelPos - speedControllerData.lastWheelPos;

    //this assumes, that the motor will never turn faster than
    //a quarter wheel turn (or 12.5 motor turns) in a ms 
    if(abs(curSpeed) > ticksPerTurn / 4) {
	//wheel is turning backward
	if(speedControllerData.lastWheelPos < wheelPos) {
	    curSpeed -= ticksPerTurn;
	}

	//wheel is turning forward
	if(speedControllerData.lastWheelPos > wheelPos) {
	    curSpeed += ticksPerTurn;
	}
    }

    //calculate PID value
    //use input from PC-Side
    setTargetValue(&(speedControllerData.pidData), targetSpeed);
    pwmValue = pid(&(speedControllerData.pidData), curSpeed);

    if(speedControllerData.debugActive) {
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
	sdata->minMaxPidOutput = speedControllerData.pidData.max_command_val;
	
	while(CAN_Transmit(&pidMessageSpeed) == CAN_NO_MB){
	    ;
	}
	
	//send speed status message
	while(CAN_Transmit(&speedDbgMessage) == CAN_NO_MB) {
	    ;
	}
    }    

    speedControllerData.lastWheelPos = wheelPos;
    return pwmValue;
}