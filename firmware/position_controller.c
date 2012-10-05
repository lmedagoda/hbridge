#include "position_controller.h"
#include "pid.h"
#include "controllers.h"
#include "encoder.h"
#include "state.h"
#include <stdlib.h>

extern volatile enum hostIDs ownHostId;

struct PositionControllerConfig {
    u16 minHysteresisDistance;
    u16 maxHysteresisDistance;
    u8 maxOverDistanceCount;
    u8 hysteresisActive;
    u8 allowWrapArround;
};

struct ControllerData posContData;

volatile struct PositionControllerConfig *activePosConfig;
volatile struct PositionControllerConfig *inActivPosConfig;

volatile struct PositionControllerConfig conf1;
volatile struct PositionControllerConfig conf2;

u8 inHystesis;
u8 hystLeavingCount;

void initPositionControllerConfig(volatile struct PositionControllerConfig *conf)
{
    //got safe, do not allow the initally
    conf->allowWrapArround = 0;
    conf->hysteresisActive = 0;
    conf->maxHysteresisDistance = 0;
    conf->minHysteresisDistance = 0;
    conf->maxOverDistanceCount = 1;
}

void positionControllerSetControllerConfiguration(volatile struct posControllerData* data)
{
    inActivPosConfig->hysteresisActive = data->hysteresisActive;
    inActivPosConfig->maxHysteresisDistance = data->maxHystDist;
    inActivPosConfig->minHysteresisDistance = data->minHystDist;
    inActivPosConfig->maxOverDistanceCount = data->overDistCount;
    inActivPosConfig->allowWrapArround = data->allowWrapAround;
    volatile struct PositionControllerConfig *tmp = activePosConfig;

    //atomar pointer switch passing the whole struct to the other thread
    //contrext at once without locking
    activePosConfig = inActivPosConfig;
    inActivPosConfig = tmp;
}

void positionControllerSetDebugActive(u8 debugActive)
{
    posContData.debugActive = debugActive;
}

void positionControllerInit()
{
    initPositionControllerConfig(&conf1);
    initPositionControllerConfig(&conf2);
    activePosConfig = &conf1;
    inActivPosConfig = &conf2;
    posContData.debugActive = 0;
    positionControllerReset(0);
}

void positionControllerDeInit()
{

}

void positionControllerReset(s32 wheelPos)
{
    inHystesis = 0;
    hystLeavingCount = 0;
    posContData.lastWheelPos = wheelPos;
    initPIDStruct(&(posContData.pidData));
}

void positionControllerSetConfiguration(s32 p, s32 i, s32 d, s32 minMax)
{
    setKp(&(posContData.pidData), p);
    setKi(&(posContData.pidData), i);
    setKd(&(posContData.pidData), d);
    setMinMaxCommandVal(&(posContData.pidData), -minMax, minMax);   
}

s32 positionControllerStep(s32 targetPos, s32 wheelPos, u32 ticksPerTurn)
{
    s32 pwmValue = 0;
    s32 curVal = wheelPos;

    //correct wraparounds
    if(activePosConfig->allowWrapArround && abs((targetPos) - curVal) > ticksPerTurn / 2) {
	if(curVal < ticksPerTurn / 2)
	    curVal += ticksPerTurn;
	else 
	    curVal -= ticksPerTurn;
    }

    //calculate PID value
    setTargetValue(&(posContData.pidData), targetPos);
    pwmValue = pid(&(posContData.pidData), curVal);
    
    if(activePosConfig->hysteresisActive)
    {
	if(inHystesis)
	{
	    //suppress the pwm generation
	    pwmValue = 0;
	    
	    //check if we left the corridor
	    if(abs(targetPos - curVal) > activePosConfig->maxHysteresisDistance)
	    {
		hystLeavingCount++;
	    } else {
		hystLeavingCount = 0;
	    }
	    
	    if(hystLeavingCount > activePosConfig->maxOverDistanceCount)
	    {
		inHystesis = 0;

		//reset pid to avoid funny behaviour
		resetPIDStruct(&(posContData.pidData));

		//generate correct pwm
		setTargetValue(&(posContData.pidData), targetPos);
		pwmValue = pid(&(posContData.pidData), curVal);
	    }
	} else {	    
	    //test if we enterd the corridor
	    if(abs(targetPos - curVal) <= activePosConfig->minHysteresisDistance)
	    {
		inHystesis = 1;
		hystLeavingCount = 0;
		//we entered the corridor, turn of pwm
		pwmValue = 0;
	    }
	}
    }

    if(posContData.debugActive) {
	u8 tickDivider = getTickDivider(activeCState->controllerInputEncoder);
	
        CanTxMsg pidMessagePos;
        CanTxMsg posDbgMessage;

	//send status message over CAN
	pidMessagePos.StdId= PACKET_ID_PID_DEBUG_POS + ownHostId;
	pidMessagePos.RTR=CAN_RTR_DATA;
	pidMessagePos.IDE=CAN_ID_STD;
	pidMessagePos.DLC= sizeof(struct pidDebugData);

	struct pidDebugData *sdata = (struct pidDebugData *) pidMessagePos.Data;
	getInternalPIDValues(&(sdata->pPart), &(sdata->iPart), &(sdata->dPart));
	sdata->minMaxPidOutput = posContData.pidData.max_command_val;
	posDbgMessage.StdId= PACKET_ID_POS_DEBUG + ownHostId;
	posDbgMessage.RTR=CAN_RTR_DATA;
	posDbgMessage.IDE=CAN_ID_STD;
	posDbgMessage.DLC= sizeof(struct posDebugData);

	struct posDebugData *pdbgdata = (struct posDebugData *) posDbgMessage.Data;
	pdbgdata->targetVal = targetPos / tickDivider;
	pdbgdata->pwmVal = pwmValue;
	pdbgdata->encoderVal = wheelPos / tickDivider;
	pdbgdata->posVal = curVal / tickDivider;
	
	while(CAN_SendMessage(&pidMessagePos)){
	    ;
	}
	
	while(CAN_SendMessage(&posDbgMessage)) {
	    ;
	}
    }

    return pwmValue;
}


