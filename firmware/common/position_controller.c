#include "position_controller.h"
#include "pid.h"
#include "interfaces/controllers.h"
#include "encoder.h"
#include "state.h"
#include <stdlib.h>
#include "printf.h"
#include "protocol.h"
#include <limits.h>
struct PositionControllerConfig {
    uint16_t minHysteresisDistance;
    uint16_t maxHysteresisDistance;
    uint8_t maxOverDistanceCount;
    uint8_t hysteresisActive;
    uint8_t allowWrapArround;

    struct pid_data pidData;
    int32_t lastWheelPos;
    uint8_t debugActive;
    uint8_t inHystesis;
    uint8_t hystLeavingCount;
    unsigned gotPIDValues:1;
    unsigned gotControllerData:1;
};

struct PositionControllerConfig posContData;


void initPositionControllerConfig(volatile struct PositionControllerConfig *conf)
{
    conf->allowWrapArround = 0;
    conf->hysteresisActive = 0;
    conf->maxHysteresisDistance = 0;
    conf->minHysteresisDistance = 0;
    conf->maxOverDistanceCount = 1;
    conf->gotControllerData = 0;
    conf->gotPIDValues = 0;
}

void positionControllerProtocolHandler(int senderId, int receiverId, int id, unsigned char *idata, unsigned short size)
{
    switch(id)
    {
	case PACKET_ID_SET_POS_CONTROLLER_DATA:
	{
	    if(size != sizeof(struct posControllerData))
	    {
		printf("Error PosCtrlData has wrong size");
	    }
	    struct posControllerData* data = (struct posControllerData* )(idata);
	    posContData.hysteresisActive = data->hysteresisActive;
	    posContData.maxHysteresisDistance = data->maxHystDist;
	    posContData.minHysteresisDistance = data->minHystDist;
	    posContData.maxOverDistanceCount = data->overDistCount;
	    posContData.allowWrapArround = data->allowWrapAround;
	    posContData.debugActive = data->debugActive;
	    posContData.gotControllerData = 1;
	    setPidConfiguration(&(posContData.pidData), &(data->pidData));
	    posContData.gotPIDValues = 1;

	    break;
	}
    }
    
    protocol_ackPacket(id, senderId);
}

uint8_t positionControllerIsConfigured()
{
    return posContData.gotPIDValues && posContData.gotControllerData;
}

void positionControllerInit()
{
    initPositionControllerConfig(&posContData);
    posContData.debugActive = 0;
    positionControllerReset(0);
    protocol_registerHandler(PACKET_ID_SET_POS_CONTROLLER_DATA, positionControllerProtocolHandler);
}

void positionControllerDeInit()
{

}

void positionControllerReset(int32_t wheelPos)
{
    posContData.inHystesis = 0;
    posContData.hystLeavingCount = 0;
    posContData.lastWheelPos = wheelPos;
    resetPIDStruct((struct pid_data *) &(posContData.pidData));
}

int32_t positionControllerStep(struct ControllerTargetData *targetData, int32_t wheelPos, uint32_t ticksPerTurn)
{
    uint16_t *pos_p = (uint16_t *) (targetData->data);
    
    uint32_t posScaled = (((ticksPerTurn * 2000) / USHRT_MAX) * *pos_p) / 2000;
    
    int32_t targetPos = posScaled;
    int32_t pwmValue = 0;
    int32_t curVal = wheelPos;

    //correct wraparounds
    if(posContData.allowWrapArround && abs((targetPos) - curVal) > ticksPerTurn / 2) {
	if(curVal < ticksPerTurn / 2)
	    curVal += ticksPerTurn;
	else 
	    curVal -= ticksPerTurn;
    }

    //calculate PID value
    setTargetValue((struct pid_data *) &(posContData.pidData), targetPos);
    pwmValue = pid((struct pid_data *) &(posContData.pidData), curVal);
    
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
		resetPIDStruct((struct pid_data *) &(posContData.pidData));

		//generate correct pwm
		setTargetValue((struct pid_data *) &(posContData.pidData), targetPos);
		pwmValue = pid((struct pid_data *) &(posContData.pidData), curVal);
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
	struct posDebugData debugData;
	
	getInternalPIDValues(&(debugData.pidData.pPart), &(debugData.pidData.iPart), &(debugData.pidData.dPart));
	debugData.pidData.minMaxPidOutput = posContData.pidData.max_command_val;
	debugData.targetVal = targetPos;
	debugData.pwmVal = pwmValue;
	debugData.encoderVal = wheelPos;
	debugData.posVal = curVal;
	
	protocol_sendData(RECEIVER_ID_ALL, PACKET_ID_POS_CONTROLLER_DEBUG, (uint8_t *) &debugData, sizeof(struct posDebugData));
    }

    return pwmValue;
}

int32_t cascadedPositionController(struct ControllerTargetData *targetData, int32_t wheelPos, uint32_t ticksPerTurn) {
//     int32_t targetPos = *((int32_t *) (targetData->data));
//     int32_t pwmValue = controllers[CONTROLLER_MODE_POSITION].step(targetPos, wheelPos, ticksPerTurn);
//     pwmValue = controllers[CONTROLLER_MODE_SPEED].step(pwmValue / 10, wheelPos, ticksPerTurn);
   
    return 0; //pwmValue;
}

