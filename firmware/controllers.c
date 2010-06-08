#include "pid.h"
#include "protocol.h"
#include <stdlib.h>
#include "inc/stm32f10x_type.h"
#include "inc/stm32f10x_can.h"

extern volatile enum hostIDs ownHostId;
struct pid_data posPidData;
struct pid_data speedPidData;
static s32 lastWheelPos = 0;

void initControllers()
{
    //init pid structs with sane values
    initPIDStruct(&(posPidData));
    initPIDStruct(&(speedPidData));
}

void resetControllers(s32 wheelPos)
{
    lastWheelPos = wheelPos;
    resetPIDStruct(&(speedPidData));
    resetPIDStruct(&(posPidData));
}


void setNewSpeedPIDValues(s32 p, s32 i, s32 d, s32 minMax) {
    setKp(&(speedPidData), p);
    setKi(&(speedPidData), i);
    setKd(&(speedPidData), d);
    setMinMaxCommandVal(&(speedPidData), -minMax, minMax);   
}

void setNewPosPIDValues(s32 p, s32 i, s32 d, s32 minMax) {
    setKp(&(posPidData), p);
    setKi(&(posPidData), i);
    setKd(&(posPidData), d);
    setMinMaxCommandVal(&(posPidData), -minMax, minMax);
}


s32 positionController(s32 targetPos, s32 wheelPos, u32 ticksPerTurn, u8 debug) {
    CanTxMsg pidMessagePos;
    CanTxMsg posDbgMessage;
    
    s32 pwmValue = 0;
    s32 curVal = wheelPos;

    //correct wraparounds
    if(abs((targetPos) - curVal) > ticksPerTurn / 2) {
    if(curVal < ticksPerTurn / 2)
	curVal += ticksPerTurn;
    else 
	curVal -= ticksPerTurn;
    }

    //calculate PID value
    setTargetValue(&(posPidData), targetPos);
    pwmValue = pid(&(posPidData), curVal);

    if(debug) {
	//send status message over CAN
	pidMessagePos.StdId= PACKET_ID_PID_DEBUG_POS + ownHostId;
	pidMessagePos.RTR=CAN_RTR_DATA;
	pidMessagePos.IDE=CAN_ID_STD;
	pidMessagePos.DLC= sizeof(struct pidDebugData);

	struct pidDebugData *sdata = (struct pidDebugData *) pidMessagePos.Data;
	getInternalPIDValues(&(sdata->pPart), &(sdata->iPart), &(sdata->dPart));
	posDbgMessage.StdId= PACKET_ID_POS_DEBUG + ownHostId;
	posDbgMessage.RTR=CAN_RTR_DATA;
	posDbgMessage.IDE=CAN_ID_STD;
	posDbgMessage.DLC= sizeof(struct posDebugData);

	struct posDebugData *pdbgdata = (struct posDebugData *) posDbgMessage.Data;
	pdbgdata->targetVal = targetPos;
	pdbgdata->pwmVal = pwmValue;
	pdbgdata->encoderVal = wheelPos;
	pdbgdata->posVal = (curVal / 4);
	
	while(CAN_Transmit(&pidMessagePos) == CAN_NO_MB){
	    ;
	}
	
	while(CAN_Transmit(&posDbgMessage) == CAN_NO_MB) {
	    ;
	}
    }

    return pwmValue;
}


s32 speedController(s32 targetSpeed, s32 wheelPos, u32 ticksPerTurn, u8 debug) {
    s32 pwmValue = 0;
    CanTxMsg speedDbgMessage;
    CanTxMsg pidMessageSpeed;

    s32 curSpeed = wheelPos - lastWheelPos;

    //this assumes, that the motor will never turn faster than
    //a quarter wheel turn (or 12.5 motor turns) in a ms 
    if(abs(curSpeed) > ticksPerTurn / 4) {
	//wheel ist turning backward
	if(lastWheelPos < wheelPos) {
	    curSpeed -= ticksPerTurn;
	}

	//wheel is turning forward
	if(lastWheelPos > wheelPos) {
	    curSpeed += ticksPerTurn;
	}
    }

    if(debug) {
	speedDbgMessage.StdId= PACKET_ID_SPEED_DEBUG + ownHostId;
	speedDbgMessage.RTR=CAN_RTR_DATA;
	speedDbgMessage.IDE=CAN_ID_STD;
	speedDbgMessage.DLC= sizeof(struct speedDebugData);
    }

    struct speedDebugData *sdbgdata = (struct speedDebugData *) speedDbgMessage.Data;

    //calculate PID value
    //use input from PC-Side
    setTargetValue(&(speedPidData), targetSpeed);
    pwmValue = pid(&(speedPidData), curSpeed);
    sdbgdata->targetVal = targetSpeed;

    if(debug) {
	sdbgdata->pwmVal = pwmValue;
	sdbgdata->encoderVal = wheelPos;
	sdbgdata->speedVal = curSpeed;

	//send status message over CAN
	pidMessageSpeed.StdId= PACKET_ID_PID_DEBUG_SPEED + ownHostId;
	pidMessageSpeed.RTR=CAN_RTR_DATA;
	pidMessageSpeed.IDE=CAN_ID_STD;
	pidMessageSpeed.DLC= sizeof(struct pidDebugData);

	struct pidDebugData *sdata = (struct pidDebugData *) pidMessageSpeed.Data;
	getInternalPIDValues(&(sdata->pPart), &(sdata->iPart), &(sdata->dPart));
	
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
    s32 pwmValue = positionController(targetPos, wheelPos, ticksPerTurn, debug);
    pwmValue = speedController(pwmValue / 10, wheelPos, ticksPerTurn, debug);
   
    return pwmValue;
}
