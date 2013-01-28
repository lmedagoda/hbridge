#include "state.h"
#include "printf.h"
#include "protocol.h"
#include "packets.h"
#include "controllers.h"
#include "encoder.h"
#include "printf.h"
#include "temperature_sensor.h"

volatile struct GlobalState state1;
volatile struct GlobalState state2;
volatile struct GlobalState *activeCState = &state1;
volatile struct GlobalState *lastActiveCState = &state2;

volatile struct ErrorState state_errorState;

void state_sensorConfigHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size);
void state_setActuatorLimitHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size);
void state_setActiveControllerHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size);
void state_setTargetValueHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size);
void state_setActuatorUnconfiguredHandler(int senderId, int receiverId, int id, unsigned char *data, short unsigned int size);
void state_sensorClearError(int senderId, int receiverId, int id, unsigned char *data, unsigned short size);
void state_actuatorClearError(int senderId, int receiverId, int id, unsigned char *data, unsigned short size);
void state_setUnconfigured(int senderId, int receiverId, int id, unsigned char *data, unsigned short size);
void state_sendStateHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size);
const char *state_getStateString(enum STATES state);

void state_init()
{
    state_initStruct(&state1);
    state_initStruct(&state2);
    
    protocol_registerHandler(PACKET_ID_SET_SENSOR_CONFIG, state_sensorConfigHandler);
    protocol_registerHandler(PACKET_ID_CLEAR_SENSOR_ERROR, state_sensorClearError);
    protocol_registerHandler(PACKET_ID_SET_ACTUATOR_CONFIG, state_setActuatorLimitHandler);
    protocol_registerHandler(PACKET_ID_CLEAR_ACTUATOR_ERROR, state_actuatorClearError);
    protocol_registerHandler(PACKET_ID_SET_ACTUATOR_UNCONFIGURED, state_setActuatorUnconfiguredHandler);
    protocol_registerHandler(PACKET_ID_SET_ACTIVE_CONTROLLER, state_setActiveControllerHandler);
    protocol_registerHandler(PACKET_ID_SET_VALUE, state_setTargetValueHandler);
    protocol_registerHandler(PACKET_ID_SET_VALUE14, state_setTargetValueHandler);
    protocol_registerHandler(PACKET_ID_SET_VALUE58, state_setTargetValueHandler);
    protocol_registerHandler(PACKET_ID_SET_UNCONFIGURED, state_setUnconfigured);
    protocol_registerHandler(PACKET_ID_REQUEST_STATE, state_sendStateHandler);
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

void state_switchToState(enum STATES nextState)
{
    enum STATES newState = STATE_SENSOR_ERROR;

    switch(lastActiveCState->internalState)
    {
	case STATE_UNCONFIGURED:
	    if(nextState == STATE_SENSORS_CONFIGURED 
		|| nextState == STATE_SENSOR_ERROR
		|| nextState == STATE_UNCONFIGURED)
		newState = nextState;
	    break;
	case STATE_SENSORS_CONFIGURED:
	    if(nextState == STATE_UNCONFIGURED || 
		nextState == STATE_SENSOR_ERROR ||
		nextState == STATE_SENSORS_CONFIGURED ||
		nextState == STATE_ACTUATOR_CONFIGURED)
		newState = nextState;
	    break;
	case STATE_ACTUATOR_CONFIGURED:
	    if(	nextState == STATE_ACTUATOR_ERROR ||
		nextState == STATE_UNCONFIGURED ||
		nextState == STATE_SENSOR_ERROR ||
		nextState == STATE_SENSORS_CONFIGURED ||
		nextState == STATE_CONTROLLER_CONFIGURED)
		newState = nextState;
	    break;
	case STATE_CONTROLLER_CONFIGURED:
	    if(	nextState == STATE_ACTUATOR_ERROR ||
		nextState == STATE_UNCONFIGURED ||
		nextState == STATE_SENSOR_ERROR ||
		nextState == STATE_SENSORS_CONFIGURED ||
		nextState == STATE_RUNNING)
		newState = nextState;	    
	    break;
	case STATE_RUNNING:
	    if(	nextState == STATE_ACTUATOR_ERROR ||
		nextState == STATE_UNCONFIGURED ||
		nextState == STATE_SENSOR_ERROR ||
		nextState == STATE_SENSORS_CONFIGURED ||
		nextState == STATE_CONTROLLER_CONFIGURED)
		newState = nextState;
	    
	    if(state_inErrorState())
	    {
		newState = STATE_ACTUATOR_ERROR;
	    }
	    
	    break;
	case STATE_ACTUATOR_ERROR:
	    if(	nextState == STATE_ACTUATOR_CONFIGURED ||
		nextState == STATE_UNCONFIGURED ||
		nextState == STATE_SENSOR_ERROR ||
		nextState == STATE_ACTUATOR_ERROR ||
		nextState == STATE_SENSORS_CONFIGURED)
		
		state_clearErrors();
		
		newState = nextState;
	    break;
	case STATE_SENSOR_ERROR:
	    if(	nextState == STATE_UNCONFIGURED ||
		nextState == STATE_SENSORS_CONFIGURED)
		newState = nextState;

	    break;
    }

    printf("Switching from state %s to state %s \n", state_getStateString(lastActiveCState->internalState), state_getStateString(newState));
    lastActiveCState->internalState = newState;
    
    struct announceStateData data;
    data.curState = nextState;
    
    protocol_sendData(RECEIVER_ID_ALL, PACKET_ID_ANNOUNCE_STATE, (const unsigned char *)&data, sizeof(struct announceStateData));
    
    state_switchState(1);
    
}

void state_checkErrors()
{
    if(activeCState->internalState == STATE_RUNNING)
    {
	if(state_inErrorState())
	{
	    state_switchToState(STATE_ACTUATOR_ERROR);
	}
    }
}

void state_sendStateHandler(int senderId, int receiverId, int id, unsigned char* idata, short unsigned int size)
{
    protocol_ackPacket(id, senderId);
    struct announceStateData data;
    data.curState = activeCState->internalState;
    
    protocol_sendData(RECEIVER_ID_ALL, PACKET_ID_ANNOUNCE_STATE, (const unsigned char *)&data, sizeof(struct announceStateData));    
}

void state_sensorConfigHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size)
{
    protocol_ackPacket(id, senderId);
    
    if(lastActiveCState->internalState != STATE_UNCONFIGURED)
    {
	state_switchToState(STATE_SENSOR_ERROR);
	printf("State: Error, got sensor config and state was not unconfigured \n");
	return;
    }

    volatile struct SensorConfiguration *sState = &(lastActiveCState->sensorConfig);
    struct sensorConfig *sCfg = (struct sensorConfig *) data;
    
    //TODO This may fail, add a way to inform driver
    state_setEncoder(&(sState->internalEncoder), &(sCfg->encoder1Config));
    state_setEncoder(&(sState->externalEncoder), &(sCfg->encoder2Config));

    sState->statusEveryMs = sCfg->statusEveryMs;
    sState->useExternalTempSensor = sCfg->externalTempSensor;

    state_switchToState(STATE_SENSORS_CONFIGURED);
    
    printf("State: Got sensor config switching state to configured \n");
}

void state_sensorClearError(int senderId, int receiverId, int id, unsigned char *data, unsigned short size){
    printf("Got clear Sensor Error \n");
    protocol_ackPacket(id, senderId);
    
    encoder_deinitEncoder(lastActiveCState->sensorConfig.internalEncoder);
    encoder_deinitEncoder(lastActiveCState->sensorConfig.externalEncoder);
    lastActiveCState->sensorConfig.internalEncoder = NO_ENCODER;
    lastActiveCState->sensorConfig.externalEncoder = NO_ENCODER;
    
    struct TemperatureInterface* sTemp = temperatureSensors_getSensorHandle(POSITION_MOTOR);
    sTemp->sensorDeInit();
    
    sTemp = temperatureSensors_getSensorHandle(POSITION_PCB);
    sTemp->sensorDeInit();
    
    state_switchToState(STATE_UNCONFIGURED);
    
    printf("Cleared sensor-config and switching state to unconfigured \n");
}

void state_setActuatorUnconfiguredHandler(int senderId, int receiverId, int id, unsigned char *data, short unsigned int size)
{
    printf("Got actuator unconfigure \n");
    protocol_ackPacket(id, senderId);
    
    state_switchState(STATE_SENSORS_CONFIGURED);    
    printf("Cleared actuator-config and switsching state to unconfigured");
}

void state_actuatorClearError(int senderId, int receiverId, int id, unsigned char *data, unsigned short size){
    protocol_ackPacket(id, senderId);
    
    state_switchToState(STATE_SENSORS_CONFIGURED);
    
    printf("Cleared actuator-config and switsching state to unconfigured");
}

void state_setActuatorLimitHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size)
{    
    protocol_ackPacket(id, senderId);
    
    if(lastActiveCState->internalState != STATE_SENSORS_CONFIGURED)
    {
	state_switchToState(STATE_ACTUATOR_ERROR);
	printf("State: Error, got actuator config and state was not 'sensors configured' \n");
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
    
    printf("Overcurrent: %i\n", aCfg->maxCurrent);

    state_switchToState(STATE_ACTUATOR_CONFIGURED);
}

void state_setActiveControllerHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size)
{
    protocol_ackPacket(id, senderId);
    
    struct setActiveControllerData *packet = (struct setActiveControllerData *) data;
    
    if(size != (sizeof(struct setActiveControllerData)))
    {
	state_switchToState(STATE_ACTUATOR_ERROR);
	printf("Error, size of expected data structure does not match\n");
	return;
    }
    
    if(activeCState->internalState == STATE_RUNNING)
    {
	state_switchToState(STATE_ACTUATOR_ERROR);
	printf("Error, motor is active, switching of controller is forbidden\n");
	return;	
    }
    
    lastActiveCState->controllMode = packet->controllerId;
    
    state_switchToState(STATE_CONTROLLER_CONFIGURED);
}

void state_setTargetValueHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size)
{
    if(activeCState->internalState == STATE_SENSOR_ERROR || activeCState->internalState == STATE_ACTUATOR_ERROR)
    {
	printf("Error, got target value while in error state\n");
	return;
    }
    
    if((activeCState->internalState != STATE_RUNNING) && (activeCState->internalState != STATE_CONTROLLER_CONFIGURED))
    {
	state_switchToState(STATE_ACTUATOR_ERROR);
	printf("Error, got target value and state is not running or controller configured\n");
	return;		
    }
    
    if(size > MAX_CONTROLLER_DATA_SIZE)
    {
	state_switchToState(STATE_ACTUATOR_ERROR);
	printf("Error, the give controller data is too big\n");
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
		lastActiveCState->targetData.data[i] = data[i];
	    }
	    lastActiveCState->targetData.dataSize = size;
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
	    uint16_t *value_p = (uint16_t *) lastActiveCState->targetData.data;
	    *value_p = value;
	    lastActiveCState->targetData.dataSize = sizeof(uint16_t);
	}
	break;
    }
    
    if(activeCState->internalState == STATE_CONTROLLER_CONFIGURED)
	state_switchToState(STATE_RUNNING);
    
    state_switchState(0);
    
    
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

const char *state_getStateString(enum STATES state)
{
    const char *int_state_s = "";
    switch(state) {
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
	case STATE_RUNNING:
	    int_state_s = "RUNNING";
	    break;
	case STATE_SENSOR_ERROR:
	    int_state_s = "SENSOR_ERROR";
	    break;
	case STATE_ACTUATOR_ERROR:
	    int_state_s = "ACTUATOR_ERROR";
	    break;
    }
    return int_state_s;
}

void state_printDebug(const volatile struct GlobalState* cs)
{
    char *ctrl_s = "";
    const char *int_state_s = state_getStateString(cs->internalState);
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
    printf("ControllMode: %s ,internal State: %s ,openloop:%hi ,pwmstep %hu \n", ctrl_s, int_state_s, cs->actuatorConfig.useOpenLoop, cs->actuatorConfig.pwmStepPerMillisecond);    
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
	printf("Error: Motor overheated\n");
    if(state_errorState.boardOverheated)
	printf("Error: Board overheated\n");
    if(state_errorState.overCurrent)
	printf("Error: Overcurrent\n");
    if(state_errorState.timeout)
	printf("Error: Timeout\n");
    if(state_errorState.badConfig)
	printf("Error: Bad Config\n");
    if(state_errorState.encodersNotInitalized)
	printf("Error: Encoders not initialized\n");
    if(state_errorState.hardwareShutdown)
	printf("Error: Hardware shutdown\n");
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


void state_setUnconfigured(int senderId, int receiverId, int id, unsigned char *data, unsigned short size){
    protocol_ackPacket(id, senderId);
    
    //Needs to be done before, to ensure that nobody reads the config
    state_switchToState(STATE_UNCONFIGURED);
    
    state_switchState(1);
    
    encoder_deinitEncoder(lastActiveCState->sensorConfig.internalEncoder);
    encoder_deinitEncoder(lastActiveCState->sensorConfig.externalEncoder);
    lastActiveCState->sensorConfig.internalEncoder = NO_ENCODER;
    lastActiveCState->sensorConfig.externalEncoder = NO_ENCODER;
    
    struct TemperatureInterface* sTemp = temperatureSensors_getSensorHandle(POSITION_MOTOR);
    sTemp->sensorDeInit();
    
    sTemp = temperatureSensors_getSensorHandle(POSITION_PCB);
    sTemp->sensorDeInit();
    
    
    printf("On Userwish: Cleared sensor-config and switching state to unconfigured");
}
