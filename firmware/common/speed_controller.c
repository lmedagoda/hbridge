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
	    setKp(&(speedControllerData.pidData), data->pidData.kp);
	    setKi(&(speedControllerData.pidData), data->pidData.ki);
	    setKd(&(speedControllerData.pidData), data->pidData.kd);
	    setMinMaxCommandVal(&(speedControllerData.pidData), -data->pidData.minMaxPidOutput, data->pidData.minMaxPidOutput);
            speedControllerData.debugActive = data->debugActive;
	    speedControllerData.isConfigured = 1;
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
    speedControllerReset(0);
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

void speedControllerSetConfiguration(int32_t p, int32_t i, int32_t d, int32_t minMax)
{
    setKp(&(speedControllerData.pidData), p);
    setKi(&(speedControllerData.pidData), i);
    setKd(&(speedControllerData.pidData), d);
    setMinMaxCommandVal(&(speedControllerData.pidData), -minMax, minMax);
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
        struct speedDebugData sdbgdata;
        
        sdbgdata.targetVal = targetSpeed;
	sdbgdata.pwmVal = pwmValue;
	sdbgdata.encoderVal = wheelPos;
	sdbgdata.speedVal = curSpeed;
        getInternalPIDValues(&(sdbgdata.pidData.pPart), &(sdbgdata.pidData.iPart), &(sdbgdata.pidData.dPart));
        sdbgdata.pidData.minMaxPidOutput = speedControllerData.pidData.max_command_val;
	
        protocol_sendData(RECEIVER_ID_ALL, PACKET_ID_SPEED_CONTROLLER_DEBUG, (unsigned char *) &sdbgdata, sizeof(struct speedDebugData));
    }    

    speedControllerData.lastWheelPos = wheelPos;
    return pwmValue;
}