#include "speed_controller.h"
#include "pid.h"
#include "controllers.h"
#include "encoder.h"
#include "state.h"
#include <stdlib.h>
#include <limits.h>
#include "printf.h"

struct SpeedControllerConfig {
};

struct ControllerData speedControllerData;

void speedControllerProtocolHandler(int senderId, int receiverId, int id, unsigned char *idata, unsigned short size)
{
    switch(id)
    {
        case PACKET_ID_SET_SPEED_CONTROLLER_DATA: {
	    struct setPidData *data = (struct setPidData *) idata;
	    setKp((struct pid_data *) &(speedControllerData.pidData), data->kp);
	    setKi((struct pid_data *) &(speedControllerData.pidData), data->ki);
	    setKd((struct pid_data *) &(speedControllerData.pidData), data->kd);
	    setMinMaxCommandVal((struct pid_data *) &(speedControllerData.pidData), -data->minMaxPidOutput, data->minMaxPidOutput);
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
    
    int32_t targetSpeed = *((int16_t *) (targetData->data));
    //TODO normalize somehow to turnsPerSecond(?)

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

//     if(speedControllerData.debugActive) {
// 	uint8_t tickDivider = getTickDivider(activeCState->controllerInputEncoder);
// 
// 	CanTxMsg speedDbgMessage;
//         CanTxMsg pidMessageSpeed;
//         speedDbgMessage.StdId= PACKET_ID_SPEED_DEBUG + ownHostId;
// 	speedDbgMessage.RTR=CAN_RTR_DATA;
// 	speedDbgMessage.IDE=CAN_ID_STD;
// 	speedDbgMessage.DLC= sizeof(struct speedDebugData);
// 
//         struct speedDebugData *sdbgdata = (struct speedDebugData *) speedDbgMessage.Data;
//         
//         sdbgdata->targetVal = targetSpeed;
// 	sdbgdata->pwmVal = pwmValue;
// 	sdbgdata->encoderVal = wheelPos / tickDivider;
// 	sdbgdata->speedVal = curSpeed;
// 
// 	//send status message over CAN
// 	pidMessageSpeed.StdId= PACKET_ID_PID_DEBUG_SPEED + ownHostId;
// 	pidMessageSpeed.RTR=CAN_RTR_DATA;
// 	pidMessageSpeed.IDE=CAN_ID_STD;
// 	pidMessageSpeed.DLC= sizeof(struct pidDebugData);
// 
// 	struct pidDebugData *sdata = (struct pidDebugData *) pidMessageSpeed.Data;
// 	getInternalPIDValues(&(sdata->pPart), &(sdata->iPart), &(sdata->dPart));
// 	sdata->minMaxPidOutput = speedControllerData.pidData.max_command_val;
// 	
// 	while(CAN_SendMessage(&pidMessageSpeed)){
// 	    ;
// 	}
// 	
// 	//send speed status message
// 	while(CAN_SendMessage(&speedDbgMessage)) {
// 	    ;
// 	}
//     }    

    speedControllerData.lastWheelPos = wheelPos;
    return pwmValue;
}