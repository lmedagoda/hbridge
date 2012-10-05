#include "position_controller.h"
#include "pid.h"
#include "controllers.h"
#include "encoder.h"
#include "state.h"
#include <stdlib.h>
#include "can.h"

extern volatile enum hostIDs ownHostId;

struct PositionControllerConfig {
    u16 minHysteresisDistance;
    u16 maxHysteresisDistance;
    u8 maxOverDistanceCount;
    u8 hysteresisActive;
    u8 allowWrapArround;

    struct pid_data pidData;
    s32 lastWheelPos;
    u8 debugActive;
    u8 inHystesis;
    u8 hystLeavingCount;
};

volatile struct PositionControllerConfig posContData;


void initPositionControllerConfig(volatile struct PositionControllerConfig *conf)
{
    //got safe, do not allow the initally
    conf->allowWrapArround = 0;
    conf->hysteresisActive = 0;
    conf->maxHysteresisDistance = 0;
    conf->minHysteresisDistance = 0;
    conf->maxOverDistanceCount = 1;
}

void positionControllerProtocolHandler(int id, unsigned char *idata, unsigned short size)
{
    switch(id)
    {
	case PACKET_ID_POS_CONTROLLER_DATA:
	{
	    if(size != sizeof(struct posControllerData))
	    {
		print("Error PosCtrlData has wrong size");
	    }
	    struct posControllerData* data = (struct posControllerData* )(idata);
	    posContData.hysteresisActive = data->hysteresisActive;
	    posContData.maxHysteresisDistance = data->maxHystDist;
	    posContData.minHysteresisDistance = data->minHystDist;
	    posContData.maxOverDistanceCount = data->overDistCount;
	    posContData.allowWrapArround = data->allowWrapAround;
	    posContData.debugActive = data->debugActive;
	    break;
	}
	case PACKET_ID_SET_PID_POS:
	{
	    struct setPidData *data = (struct setPidData *) idata;
	    setKp(&(posContData.pidData), data->kp);
	    setKi(&(posContData.pidData), data->ki);
	    setKd(&(posContData.pidData), data->kd);
	    setMinMaxCommandVal(&(posContData.pidData), -data->minMaxPidOutput, data->minMaxPidOutput);
	    break;
	}
    }
    
    protocol_ackPacket(id);
}

void positionControllerInit()
{
    initPositionControllerConfig(&posContData);
    posContData.debugActive = 0;
    positionControllerReset(0);
    protocol_registerHandler(PACKET_ID_POS_CONTROLLER_DATA, positionControllerProtocolHandler);
    protocol_registerHandler(PACKET_ID_SET_PID_POS, positionControllerProtocolHandler);
}

void positionControllerDeInit()
{

}

void positionControllerReset(s32 wheelPos)
{
    posContData.inHystesis = 0;
    posContData.hystLeavingCount = 0;
    posContData.lastWheelPos = wheelPos;
    resetPIDStruct(&(posContData.pidData));
}

s32 positionControllerStep(s32 targetPos, s32 wheelPos, u32 ticksPerTurn)
{
    s32 pwmValue = 0;
    s32 curVal = wheelPos;

    //correct wraparounds
    if(posContData.allowWrapArround && abs((targetPos) - curVal) > ticksPerTurn / 2) {
	if(curVal < ticksPerTurn / 2)
	    curVal += ticksPerTurn;
	else 
	    curVal -= ticksPerTurn;
    }

    //calculate PID value
    setTargetValue(&(posContData.pidData), targetPos);
    pwmValue = pid(&(posContData.pidData), curVal);
    
    if(posContData.hysteresisActive)
    {
	if(posContData.inHystesis)
	{
	    //suppress the pwm generation
	    pwmValue = 0;
	    
	    //check if we left the corridor
	    if(abs(targetPos - curVal) > posContData.maxHysteresisDistance)
	    {
		posContData.hystLeavingCount++;
	    } else {
		posContData.hystLeavingCount = 0;
	    }
	    
	    if(posContData.hystLeavingCount > posContData.maxOverDistanceCount)
	    {
		posContData.inHystesis = 0;

		//reset pid to avoid funny behaviour
		resetPIDStruct(&(posContData.pidData));

		//generate correct pwm
		setTargetValue(&(posContData.pidData), targetPos);
		pwmValue = pid(&(posContData.pidData), curVal);
	    }
	} else {	    
	    //test if we enterd the corridor
	    if(abs(targetPos - curVal) <= posContData.minHysteresisDistance)
	    {
		posContData.inHystesis = 1;
		posContData.hystLeavingCount = 0;
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

s32 cascadedPositionController(s32 targetPos, s32 wheelPos, u32 ticksPerTurn) {
    s32 pwmValue = controllers[CONTROLLER_MODE_POSITION].step(targetPos, wheelPos, ticksPerTurn);
    pwmValue = controllers[CONTROLLER_MODE_SPEED].step(pwmValue / 10, wheelPos, ticksPerTurn);
   
    return pwmValue;
}

