#include "protocol.h"
#include "state.h"
#include "../interfaces/controllers.h"
#include "../interfaces/hbridge.h"
#include "../interfaces/encoder.h"
#include "../interfaces/current_measurement.h"
#include "../interfaces/temperature_sensor.h"
#include "../interfaces/thread.h"
#include "printf.h"
#include <stdlib.h>

#define MIN_S16 (-(1<<15))
#define MAX_S16 ((1<<15) -1)

extern volatile enum hostIDs ownHostId;

unsigned int statusMsgCounter;
static int32_t currentPwmValue = 0;
static uint16_t index = 0;
static uint16_t overCurrentCounter = 0;
static uint16_t timeoutCounter = 0;
static int32_t temperature = 0;
static uint32_t overTempCounter = 0;
static int32_t motorTemperature = 0;
static uint32_t overMotorTempCounter = 0;

volatile struct TemperatureInterface pcbTempSensor;
volatile struct TemperatureInterface motorTempSensor;

void checkMotorTemperature();
void checkTimeout();
void checkTemperature();
void checkOverCurrent();

void sendStatusMessage(uint32_t pwmValue, uint32_t currentValue, int32_t temperature, int32_t motorTemperature, uint32_t index);
void sendErrorMessage(int32_t temperature, int32_t motorTemperature, uint32_t index);

void systick_step();

void resetCounters()
{
    overCurrentCounter = 0;
    timeoutCounter = 0;
    overTempCounter = 0;
    overMotorTempCounter = 0;
}

void checkTimeout()
{	
    //check for timeout and go into error state
    if(activeCState->actuatorConfig.timeout && (timeoutCounter > activeCState->actuatorConfig.timeout)) {
	state_getErrorState()->timeout = 1;
    }
}

void checkOverCurrent(uint32_t currentValue)
{
    //check for overcurrent
    if(currentValue > activeCState->actuatorConfig.maxCurrent) {
	overCurrentCounter++;

	if(overCurrentCounter > activeCState->actuatorConfig.maxCurrentCount) {
	    state_getErrorState()->overCurrent = 1;
	}
    } else {
	overCurrentCounter = 0;
    }
}

void checkMotorTemperature(uint32_t temperature)
{
    if(!activeCState->sensorConfig.useExternalTempSensor)
	return;
	
    //check for overcurrent
    if(motorTemperature > activeCState->actuatorConfig.maxMotorTemp) {
	overMotorTempCounter++;

	if(overMotorTempCounter > activeCState->actuatorConfig.maxMotorTempCount) {
	    state_getErrorState()->motorOverheated = 1;
	}
    } else {
	overMotorTempCounter = 0;
    }
}

void checkTemperature(uint32_t temperature)
{
    //check for overcurrent
    if(temperature > activeCState->actuatorConfig.maxBoardTemp) {
	overTempCounter++;

	if(overTempCounter > activeCState->actuatorConfig.maxBoardTempCount) {
	    state_getErrorState()->boardOverheated = 1;
	}
    } else {
	overTempCounter = 0;
    }
}

void baseInit()
{
    statusMsgCounter = 0;
    currentPwmValue = 0;
    index = 0;
    overCurrentCounter = 0;
    timeoutCounter = 0;
    temperature = 0;
    overTempCounter = 0;
    motorTemperature = 0;
    overCurrentCounter = 0;

    protocol_init();
    state_init();
    encoder_init();
    temperatureSensors_init();
    controllers_init();
    
    //clear all errors, btw initialize error state struct
    state_clearErrors();
}

void platformInit()
{
    hbridge_init();
    currentMeasurement_init();

    pcbTempSensor = *temperatureSensors_getSensorHandle(POSITION_PCB);
    motorTempSensor = *temperatureSensors_getSensorHandle(POSITION_MOTOR);
    
    pcbTempSensor.sensorInit();
    motorTempSensor.sensorInit();
    
    startHardPeriodicThread(1000, systick_step);
    
}

void run()
{
    // 	uint16_t errorDbg = inErrorState();
// 	if((curMsg->StdId != PACKET_ID_SET_VALUE14 && curMsg->StdId != PACKET_ID_SET_VALUE58)  || cnt == 50) {
// 	    cnt = 0;  
// 	    printf("Error is %hu \n", errorDbg);
// 	    printErrorState();
// 	    print("ActiveCstate: ");
// 	    printStateDebug(activeCState);
// 	    print(" LastActiveCstate: ");
// 	    printStateDebug(lastActiveCState);
// 	} else {
// 	    cnt++;
// 	}
//     }

    protocol_processPackages();
}

void systick_unconfiguredState(uint32_t index)
{
    //be shure motor is off
    hbridge_setNewPWM(0, activeCState->actuatorConfig.useOpenLoop);
    
    resetCounters();
}

void systick_runningState(uint32_t index)
{
    int32_t pwmValue = 0;
    int32_t wheelPos = 0;
    uint32_t ticksPerTurn = 0;
    
    switch(activeCState->controllerInputEncoder) {
        case INTERNAL:
            wheelPos = getTicks(activeCState->sensorConfig.internalEncoder);
            ticksPerTurn = getTicksPerTurn(activeCState->sensorConfig.internalEncoder);
            break;
        case EXTERNAL:
            wheelPos = getTicks(activeCState->sensorConfig.externalEncoder);
            ticksPerTurn = getTicksPerTurn(activeCState->sensorConfig.externalEncoder);
            break;
    }

    uint32_t currentValue = currentMeasurement_getValue();

    //reset timeout, if "userspace" requested it
    if(activeCState->resetTimeoutCounter) {
	activeCState->resetTimeoutCounter = 0;
	timeoutCounter = 0;
    } else{
	//increase timeout
	timeoutCounter++;
    }

    //change state to error if error is set
    if(state_inErrorState()) {
	activeCState->internalState = STATE_ERROR;
	return;
    }

    checkTimeout();
    checkTemperature(temperature);
    checkMotorTemperature(motorTemperature);
    checkOverCurrent(currentValue);
	
    //calculate pwm value
    const struct ControllerInterface *curCtrl = controllers_getController(activeCState->controllMode);
    pwmValue =  curCtrl->step((struct ControllerTargetData *) &(activeCState->targetData), wheelPos, ticksPerTurn);

    //trunkcate to int16_t
    if(pwmValue > MAX_S16) 
	pwmValue = MAX_S16;
    if(pwmValue < MIN_S16)
	pwmValue = MIN_S16;

    if(abs(currentPwmValue - pwmValue) < activeCState->actuatorConfig.pwmStepPerMillisecond) {
	currentPwmValue = pwmValue;
    } else {
	if(currentPwmValue - pwmValue < 0) {
	    currentPwmValue += activeCState->actuatorConfig.pwmStepPerMillisecond;
	} else {
	    currentPwmValue -= activeCState->actuatorConfig.pwmStepPerMillisecond;      
	}
    }

    sendStatusMessage(currentPwmValue, currentValue, temperature, motorTemperature, index);
	
    //set pwm
    hbridge_setNewPWM(currentPwmValue, activeCState->actuatorConfig.useOpenLoop);
}

void systick_errorState(uint32_t index)
{
    if(index % activeCState->sensorConfig.statusEveryMs == 0)
	sendErrorMessage(temperature, motorTemperature, index);

    //be shure motor is off
    hbridge_setNewPWM(0, activeCState->actuatorConfig.useOpenLoop);

    resetCounters();
}

void systick_step()
{
    ++statusMsgCounter;
    
    index++;
    if(index >= (1<<10))
    {
	index = 0;
	state_printDebug(activeCState);
    }
    
    // get temperature
    pcbTempSensor.getTemperature(&temperature);

    if(activeCState->sensorConfig.useExternalTempSensor)
    {
	motorTempSensor.getTemperature(&motorTemperature);
    }
    
    switch(activeCState->internalState)
    {
	case STATE_UNCONFIGURED:
	    systick_unconfiguredState(index);
	    break;
	case STATE_SENSORS_CONFIGURED:
	    break;
	case STATE_ACTUATOR_CONFIGURED:
	    break;
	case STATE_CONTROLLER_CONFIGURED:
	    break;
	case STATE_RUNNING:
	    systick_runningState(index);
	    break;
	case STATE_ERROR:
	    systick_errorState(index);
	    break;
    }

}

void sendErrorMessage(int32_t temperature, int32_t motorTemperature, uint32_t index)
{
    struct errorData edata;

    volatile struct ErrorState *es = state_getErrorState();

    if(temperature < 0)
	temperature = 0;

    edata.temperature = temperature;
    edata.position = getDividedTicks(activeCState->sensorConfig.internalEncoder);
    edata.index = index;
    edata.externalPosition = getDividedTicks(activeCState->sensorConfig.externalEncoder);
    edata.motorOverheated = es->motorOverheated;
    edata.boardOverheated = es->boardOverheated;
    edata.overCurrent = es->overCurrent;
    edata.timeout = es->timeout;
    edata.badConfig = es->badConfig;
    edata.encodersNotInitalized = es->encodersNotInitalized;

    protocol_sendData(PACKET_ID_ERROR, (unsigned char *) &edata, sizeof(struct errorData));
}

void sendStatusMessage(uint32_t pwmValue, uint32_t currentValue, int32_t temperature, int32_t motorTemperature, uint32_t index)
{
    if(statusMsgCounter < activeCState->sensorConfig.statusEveryMs || !activeCState->sensorConfig.statusEveryMs)
	return;
    
    statusMsgCounter = 0;
    
    struct statusData sdata;
    
    sdata.pwm = currentPwmValue;
    sdata.externalPosition = getDividedTicks(activeCState->sensorConfig.externalEncoder);
    sdata.position = getDividedTicks(activeCState->sensorConfig.internalEncoder);
    sdata.currentValue = currentValue;
    sdata.index = index;

    protocol_sendData(PACKET_ID_STATUS,(unsigned char *) &sdata, sizeof(struct statusData));
    
    static unsigned int extendedStatusCounter = 0;
    extendedStatusCounter += activeCState->sensorConfig.statusEveryMs;
    
    //not this is not exact every 100 ms, but good enough
    if(extendedStatusCounter > 100)
    {
	extendedStatusCounter = 0;
	
	struct extendedStatusData esdata;
	
	if(temperature < 0)
	    temperature = 0;
	esdata.temperature = temperature;
	    
	if(motorTemperature < 0)
	    motorTemperature = 0;
	esdata.motorTemperature = motorTemperature;
	
	protocol_sendData(PACKET_ID_EXTENDED_STATUS, (unsigned char *) &esdata, sizeof(struct extendedStatusData));
    }
}
