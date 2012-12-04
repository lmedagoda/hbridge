#include "state.h"
#include "printf.h"
#include "protocol.h"
#include "packets.h"
#include "controllers.h"
#include "encoder.h"

volatile struct GlobalState state1;
volatile struct GlobalState state2;
volatile struct GlobalState *activeCState = &state1;
volatile struct GlobalState *lastActiveCState = &state2;

volatile struct ErrorState state_errorState;

volatile struct ControllerTargetData controllerTargetValueData1;
volatile struct ControllerTargetData controllerTargetValueData2;
volatile struct ControllerTargetData *activeControllerTargetValueData;
volatile struct ControllerTargetData *inactiveControllerTargetValueData;

void state_sensorConfigHandler(int id, unsigned char *data, unsigned short size);
void state_setActuatorLimitHandler(int id, unsigned char *data, unsigned short size);
void state_setActiveControllerHandler(int id, unsigned char *data, unsigned short size);
void state_setTargetValueHandler(int id, unsigned char *data, unsigned short size);

void state_init()
{
    state_initStruct(&state1);
    state_initStruct(&state2);
    
    activeControllerTargetValueData = &controllerTargetValueData1;
    inactiveControllerTargetValueData = &controllerTargetValueData2;

    protocol_registerHandler(PACKET_ID_SET_SENSOR_CONFIG, state_sensorConfigHandler);
    protocol_registerHandler(PACKET_ID_SET_ACTUATOR_CONFIG, state_setActuatorLimitHandler);
    protocol_registerHandler(PACKET_ID_SET_ACTIVE_CONTROLLER, state_setActiveControllerHandler);
    protocol_registerHandler(PACKET_ID_SET_VALUE, state_setTargetValueHandler);
    protocol_registerHandler(PACKET_ID_SET_VALUE14, state_setTargetValueHandler);
    protocol_registerHandler(PACKET_ID_SET_VALUE58, state_setTargetValueHandler);
}

void state_switchState(uint8_t forceSynchronisation)
{
    //this is concurrency proof, as this code can not run, while
    //systick Handler is active !
    volatile struct GlobalState *tempstate = activeCState;

    //we go an new packet, so reset the timeout
    lastActiveCState->resetTimeoutCounter = 1;
    
    //this is atomar as only the write is relevant!
    activeCState = lastActiveCState;

    lastActiveCState = tempstate;
    
    *lastActiveCState = *activeCState;      
    
    if(forceSynchronisation) {
	while(activeCState->resetTimeoutCounter)
	    ;
    }
}

void state_setEncoder(volatile enum encoderTypes *curEncoder, const struct encoderConfiguration *newConfig)
{
    if(*curEncoder != newConfig->encoderType)
    {
	encoder_deinitEncoder(*curEncoder);
	encoder_initEncoder(newConfig->encoderType);
	encoder_setTicksPerTurn(newConfig->encoderType, newConfig->ticksPerTurn, newConfig->tickDivider);
	*curEncoder = newConfig->encoderType;
    }
}

void state_switchToErrorState()
{
    lastActiveCState->internalState = STATE_ERROR;
    
    //TODO Send error message
    
    state_switchState(1);
    
}


void state_sensorConfigHandler(int id, unsigned char *data, unsigned short size)
{
    if(lastActiveCState->internalState != STATE_UNCONFIGURED)
    {
	state_switchToErrorState();
	protocol_ackPacket(id);
	print("State: Error, got sensor config and state was not unconfigured \n");
	return;
    }
    
    volatile struct SensorConfiguration *sState = &(lastActiveCState->sensorConfig);
    struct sensorConfig *sCfg = (struct sensorConfig *) data;
    
    state_setEncoder(&(sState->internalEncoder), &(sCfg->encoder1Config));
    state_setEncoder(&(sState->externalEncoder), &(sCfg->encoder2Config));

    sState->statusEveryMs = sCfg->statusEveryMs;
    sState->useExternalTempSensor = sCfg->externalTempSensor;

    
    lastActiveCState->internalState = STATE_SENSORS_CONFIGURED;
    
    state_switchState(1);
    protocol_ackPacket(id);
    print("State: Got sensor config switching state to configured \n");
}

void state_setActuatorLimitHandler(int id, unsigned char *data, unsigned short size)
{    
    if(lastActiveCState->internalState != STATE_SENSORS_CONFIGURED)
    {
	state_switchToErrorState();
	protocol_ackPacket(id);
	return;
    }
    
    volatile struct ActuatorConfiguration *aState = &(lastActiveCState->actuatorConfig);
    struct actuatorConfig *aCfg = (struct actuatorConfig *) data;
    
    aState->maxBoardTemp = aCfg->maxBoardTemp;
    aState->useOpenLoop = aCfg->openCircuit;
    aState->maxMotorTemp = aCfg->maxMotorTemp;
    aState->maxMotorTempCount = aCfg->maxMotorTempCount;
    aState->maxBoardTemp = aCfg->maxBoardTemp;
    aState->maxBoardTempCount = aCfg->maxBoardTempCount;
    aState->timeout = aCfg->timeout;
    aState->maxCurrent = aCfg->maxCurrent;
    aState->maxCurrentCount = aCfg->maxCurrentCount;
    aState->pwmStepPerMillisecond = aCfg->pwmStepPerMs;

    protocol_ackPacket(id);   
    state_switchState(1);
}

void state_setActiveControllerHandler(int id, unsigned char *data, unsigned short size)
{
    struct setActiveControllerData *packet = (struct setActiveControllerData *) data;
    
    if(size != (sizeof(struct setActiveControllerData)))
    {
	print("Error, size of expected data structure does not match\n");
	return;
    }
    
    if(activeCState->internalState == STATE_RUNNING)
    {
	print("Error, motor is active, switching of controller is forbidden\n");
	return;	
    }
    
    lastActiveCState->controllMode = packet->controllerId;
    state_switchState(0);
}

void state_setTargetValueHandler(int id, unsigned char *data, unsigned short size)
{
    if(size > MAX_CONTROLLER_DATA_SIZE)
    {
	print("Error, the give controller data is too big\n");
	return;
    }
    
    int ownHostId = protocol_getOwnHostId();
    
    switch(id)
    {
	case PACKET_ID_SET_VALUE:
	{
	    int i;
	    for(i = 0; i < size; i++)
	    {
		inactiveControllerTargetValueData->data[i] = data[i];
	    }
	    inactiveControllerTargetValueData->dataSize = size;
	    break;
	}
	case PACKET_ID_SET_VALUE14:
	case PACKET_ID_SET_VALUE58:
	{
	    struct setValueData *tdata = (struct setValueData *) data;
	    uint16_t value = 0;
	    switch(ownHostId) {
		    case RECEIVER_ID_H_BRIDGE_1:
		    case RECEIVER_ID_H_BRIDGE_5:
			value = tdata->board1Value;
			break;
		    case RECEIVER_ID_H_BRIDGE_2:
		    case RECEIVER_ID_H_BRIDGE_6:
			value = tdata->board2Value;
			break;
		    case RECEIVER_ID_H_BRIDGE_3:
		    case RECEIVER_ID_H_BRIDGE_7:
			value = tdata->board3Value;
			break;
		    case RECEIVER_ID_H_BRIDGE_4:
		    case RECEIVER_ID_H_BRIDGE_8:
			value = tdata->board4Value;
			break;
		}
	    uint16_t *value_p = (uint16_t *) inactiveControllerTargetValueData->data;
	    *value_p = value;
	    inactiveControllerTargetValueData->dataSize = sizeof(uint16_t);
	}
	break;
    }
    
    volatile struct ControllerTargetData *tmp = activeControllerTargetValueData;
    //atomar switch of data structure
    activeControllerTargetValueData = inactiveControllerTargetValueData;
    inactiveControllerTargetValueData = tmp;
}

void state_initStruct(volatile struct GlobalState *cs)
{
    //init cotroller state with sane values
    cs->controllMode = CONTROLLER_MODE_NONE;
    cs->internalState = STATE_UNCONFIGURED;
    
    cs->sensorConfig.internalEncoder = NO_ENCODER;
    cs->sensorConfig.externalEncoder = NO_ENCODER;
    cs->sensorConfig.statusEveryMs = 0;
    cs->sensorConfig.useExternalTempSensor = 0;
    
    cs->actuatorConfig.useOpenLoop = 0;
    cs->actuatorConfig.pwmStepPerMillisecond = 0;
    cs->actuatorConfig.maxCurrent = 0;
    cs->actuatorConfig.maxCurrentCount = 0;
    cs->actuatorConfig.maxMotorTemp = 0;
    cs->actuatorConfig.maxMotorTempCount = 0;
    cs->actuatorConfig.maxBoardTemp = 0;
    cs->actuatorConfig.maxBoardTempCount = 0;
    cs->actuatorConfig.timeout = 1;
    cs->controllerInputEncoder = INTERNAL;
    cs->resetTimeoutCounter = 0;
};

void state_printDebug(volatile struct GlobalState* cs)
{
    char *ctrl_s = "";
    char *int_state_s = "";
    switch(cs->controllMode) {
	case CONTROLLER_MODE_NONE:
	    ctrl_s = "NONE";
	    break;
	case CONTROLLER_MODE_PWM:
	    ctrl_s = "PWM";
	    break;
	case CONTROLLER_MODE_POSITION:
	    ctrl_s = "POSITION";
	    break;
	case CONTROLLER_MODE_SPEED:
	    ctrl_s = "SPEED";
	    break;
	default:
	    ctrl_s = "ERROR INVALID";
	    break;
    }
    switch(cs->internalState) {
	case STATE_UNCONFIGURED:
	    int_state_s = "UNCONFIGURED";
	    break;
	case STATE_SENSORS_CONFIGURED:
	    int_state_s = "SENSORS_CONFIGURED";
	    break;
	case STATE_ACTUATOR_CONFIGURED:
	    int_state_s = "ACTUATOR_CONFIGURED";
	    break;
	case STATE_CONTROLLER_CONFIGURED:
	    int_state_s = "CONTROLLER_CONFIGURED";
	    break;
	case STATE_ERROR:
	    int_state_s = "ERROR";
	    break;
    }
//     printf("ControllMode: %s ,internal State: %s ,targetVal : %li ,openloop:%hi ,pwmstep %hu \n", ctrl_s, int_state_s, cs->targetValue, cs->useOpenLoop, cs->pwmStepPerMillisecond);    
}

uint8_t state_inErrorState() {
    return  state_errorState.motorOverheated ||
            state_errorState.boardOverheated ||
            state_errorState.overCurrent ||
            state_errorState.timeout ||
            state_errorState.badConfig ||
            state_errorState.encodersNotInitalized ||
            state_errorState.controllersNotConfigured ||
            state_errorState.hardwareShutdown;
}

void state_printErrorState()
{
    if(state_errorState.motorOverheated)
	print("Error: Motor overheated\n");
    if(state_errorState.boardOverheated)
	print("Error: Board overheated\n");
    if(state_errorState.overCurrent)
	print("Error: Overcurrent\n");
    if(state_errorState.timeout)
	print("Error: Timeout\n");
    if(state_errorState.badConfig)
	print("Error: Bad Config\n");
    if(state_errorState.encodersNotInitalized)
	print("Error: Encoders not initialized\n");
    if(state_errorState.hardwareShutdown)
	print("Error: Hardware shutdown\n");
}

volatile struct ErrorState *state_getErrorState() {
    return &state_errorState;
}

void state_clearErrors() {
    state_errorState.motorOverheated = 0;
    state_errorState.boardOverheated = 0;
    state_errorState.overCurrent = 0;
    state_errorState.timeout = 0;
    state_errorState.badConfig = 0;
    state_errorState.controllersNotConfigured = 0;
    state_errorState.encodersNotInitalized = 0;
    state_errorState.hardwareShutdown = 0;
}


