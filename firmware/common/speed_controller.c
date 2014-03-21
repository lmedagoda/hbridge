#include "speed_controller.h"
#include "pid.h"
#include "controllers.h"
#include "encoder.h"
#include "state.h"
#include <stdlib.h>
#include <limits.h>
#include "printf.h"

struct ControllerData speedControllerData;

void speedControllerProtocolHandler(int senderId, int receiverId, int id, unsigned char *idata, unsigned short size)
{
    switch(id)
    {
        case PACKET_ID_SET_SPEED_CONTROLLER_DATA: {
	    struct speedControllerData *data = (struct speedControllerData *) idata;
	    setPidConfiguration(&(speedControllerData.pidData), &(data->pidData));
            speedControllerData.debugActive = data->debugActive;
	    speedControllerData.isConfigured = 1;
	    //todo add speedControllerData.debugEveryXMs
	    break;
	}
    }
    protocol_ackPacket(id, senderId);
}

uint8_t speedControllerIsConfigured()
{
    return speedControllerData.isConfigured;
}

void speedControllerInit(void )
{
    speedControllerData.debugActive = 0;
    speedControllerData.isConfigured = 0;
    speedControllerData.debugCounter = 0;
    speedControllerData.debugEveryXMs = 20;
    speedControllerReset(0);
    initPIDStruct(&(speedControllerData.pidData));
    protocol_registerHandler(PACKET_ID_SET_SPEED_CONTROLLER_DATA, speedControllerProtocolHandler);
}

void speedControllerDeInit(void )
{

}

void speedControllerReset(int32_t wheelPos)
{
    speedControllerData.lastWheelPos = wheelPos;
    resetPIDStruct(&(speedControllerData.pidData));
}

void speedControllerSetDebugActive(uint8_t debugActive)
{
    speedControllerData.debugActive = debugActive;
}

int32_t speedControllerStep(struct ControllerTargetData *targetData, int32_t wheelPos, uint32_t ticksPerTurn)
{
    if(targetData->dataSize != sizeof(int16_t))
    {
	printf("Error, got unexpected ControllerTargetData\n");
	return 0;
    }
    
    int16_t *speed_p = (int16_t *) (targetData->data);
    int32_t targetSpeed = *speed_p;
    
    //input is turns per second * 100
    targetSpeed = targetSpeed * ((int32_t) ticksPerTurn) / (1000 * 100);

    int32_t pwmValue = 0;

    int32_t curSpeed = wheelPos - speedControllerData.lastWheelPos;

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
	if(speedControllerData.debugCounter >= speedControllerData.debugEveryXMs)
	{
	    struct speedDebugData sdbgdata;
        
	    sdbgdata.targetVal = targetSpeed;
	    sdbgdata.pwmVal = pwmValue;
	    sdbgdata.encoderVal = wheelPos;
	    sdbgdata.speedVal = curSpeed;
	    getPidDebugData(&(speedControllerData.pidData), &(sdbgdata.pidData));
	    
	    protocol_sendData(RECEIVER_ID_PC, PACKET_ID_SPEED_CONTROLLER_DEBUG, (unsigned char *) &sdbgdata, sizeof(struct speedDebugData));
	    speedControllerData.debugCounter = 0;
	}
	speedControllerData.debugCounter++;
    }    

    speedControllerData.lastWheelPos = wheelPos;
    return pwmValue;
}
