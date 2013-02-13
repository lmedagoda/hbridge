#include "hbridge_cmd.h"
#include "hbridge_cmd2.h"
#include "printf.h"
#include <stdint.h>
#include <stddef.h>
#include "protocol.h"
#include "common/time.h"

#define MAX_HBRIDGES 8

int hbridge_nrHbridges;
uint32_t hbridge_configureTimeout = 30;
uint32_t hbridge_stateReplyTimeout = 30;
struct sensorConfig hbridge_sensorConfig[MAX_HBRIDGES];
struct actuatorConfig hbridge_actuatorConfig[MAX_HBRIDGES];
struct setActiveControllerData hbridge_controllerConfig[MAX_HBRIDGES];

enum DRIVER_STATE
{
    NONE,
    COM_ERROR,
    REQUESTING_STATE,
    GOT_STATE,
    CONFIGURING_SENSORS,
    CONFIGURING_SENSORS_SENT,
    CONFIGURING_SENSORS_DONE,
    CONFIGURING_ACTUATORS,
    CONFIGURING_ACTUATORS_SENT,
    CONFIGURING_ACTUATORS_DONE,
    CONFIGURING_CONTROLLERS,
    CONFIGURING_CONTROLLERS_DONE,
};

struct hbridgeState
{
    enum DRIVER_STATE driverState;
    enum STATES state;
    uint32_t stateTime;
    
    uint8_t pendingAck;
    int pendingAckId;
    
    uint32_t sendTime;
};

struct hbridgeState hbridge_states[MAX_HBRIDGES];

void hbridge_setPendingAck(int hbId, int packetId)
{
    hbridge_states[hbId].pendingAck = 1;
    hbridge_states[hbId].pendingAckId = packetId;
    hbridge_states[hbId].sendTime = time_getTimeInMs();
}

uint8_t hbridge_hasAckTimeout(int hbId, uint32_t timeout)
{
    uint32_t diff = time_getTimeInMs() - hbridge_states[hbId].sendTime;
    if(hbridge_states[hbId].pendingAck && (diff > timeout))
	return 1;

    //no timeout
    return 0;
}

uint8_t hbridge_checkTimeout(uint32_t timeout, char *job)
{
    int ret = 0;
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	if(hbridge_hasAckTimeout(i, hbridge_configureTimeout))
	{
	    printf("hbridge: Motor driver %i had a timeout while %s\n", i, job);
	    ret = 1;
	}
    }
    return ret;
}

uint8_t hbridge_checkAllInState(enum STATES state)
{
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	if(hbridge_states[i].state != state)
	    return 0;
    }
    return 1;
}

uint8_t hbridge_checkAllInDriverState(enum DRIVER_STATE state)
{
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	if(hbridge_states[i].driverState != state)
	    return 0;
    }
    return 1;
}

enum STATES hbridge_getLowestHBState()
{
    int i;
    enum STATES lowestState = STATE_RUNNING;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	if(hbridge_states[i].state < lowestState)
	{
	    lowestState = hbridge_states[i].state;
	}
    }
    return lowestState;
}

void hbridge_ackHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size)
{
    if(!hbridge_states[senderId].pendingAck)
    {
	printf("Warning, got unexpected ack from hb %i\n", senderId);
	return;
    }
    
    if(hbridge_states[senderId].pendingAckId != id)
    {
	printf("Warning, got ack for unexpected packet id %s expected %s, id %i\n", getPacketName(id), getPacketName(hbridge_states[senderId].pendingAckId), senderId);
	return;
    }

    hbridge_states[senderId].pendingAck = 0;
}

void hbridge_stateHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size)
{
    int hbId = senderId - SENDER_ID_H_BRIDGE_1;
    if(hbId < 0 || hbId > hbridge_nrHbridges)
    {
	printf("Error, got state announce for invalid hbridge id\n");
	return;
    }
    //printf("StateHandler\n");
    struct announceStateData *asd = (struct announceStateData*) data;
    hbridge_states[hbId].state = asd->curState;
    hbridge_states[hbId].stateTime = time_getTimeInMs();
    
    if(hbridge_states[hbId].driverState == REQUESTING_STATE)
	hbridge_states[hbId].driverState = GOT_STATE;
}

void hbridge_init(uint16_t numHbridges)
{
    hbridge_nrHbridges = numHbridges;
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	hbridge_states[i].driverState = NONE;
	hbridge_states[i].pendingAck = 0;
    }
    
    protocol_registerHandler(PACKET_ID_ANNOUNCE_STATE, hbridge_stateHandler);
    protocol_registerHandler(PACKET_ID_ACK, hbridge_ackHandler);

    //request initial state of all hbs
    hbridge_requestStates();
}

uint8_t hbridge_getControlledHbridges()
{
    return hbridge_nrHbridges;
}



void hbridge_resetActuators()
{
    printf("hbridge_resetActuators TODO IMPLEMENT ME\n");
}

void hbridge_resetSensors()
{
    printf("hbridge_resetSensors TODO IMPLEMENT ME\n");
}

struct actuatorConfig* getActuatorConfig(uint16_t hbridgeNr)
{
    if(hbridgeNr > hbridge_nrHbridges)
    {
	printf("ERROR, requested actuator config for invalid hbridge id\n");
	return NULL;
    }
    
    return hbridge_actuatorConfig + hbridgeNr;    
}

struct sensorConfig* getSensorConfig(uint16_t hbridgeNr)
{
    if(hbridgeNr > hbridge_nrHbridges)
    {
	printf("ERROR, requested sensor config for invalid hbridge id\n");
	return NULL;
    }
    
    return hbridge_sensorConfig + hbridgeNr;
}

enum STATES hbridge_getState(uint16_t hbridgeNr)
{
    return hbridge_states[hbridgeNr].state;
}

void hbridge_requestStates()
{
    printf("Requesting motor driver state\n");
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	hbridge_requestState(i + RECEIVER_ID_H_BRIDGE_1);
	hbridge_states[i].driverState = REQUESTING_STATE;
	hbridge_states[i].sendTime = time_getTimeInMs();
    }    
}

void hbridge_triggerSensorConfiguration()
{
	
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	if(hbridge_states[i].state != STATE_UNCONFIGURED)
	    hbridge_setUnconfigured(i + RECEIVER_ID_H_BRIDGE_1);
	
	hbridge_states[i].driverState = CONFIGURING_SENSORS;
	hbridge_setPendingAck(i, PACKET_ID_SET_UNCONFIGURED);
    }
}

uint8_t hbridge_configureSensors()
{
    uint8_t error = 0;
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	if(hbridge_states[i].driverState < GOT_STATE)
	{
	    printf("Error, hbridge %i did not send state, can not configure\n", i);
	    error = 1;
	}
    }
    if(error)
	return 0;

    printf("Configuration motor drivers\n");
    hbridge_triggerSensorConfiguration();
    
    while(1)
    {
	if(hbridge_checkTimeout(hbridge_configureTimeout, "configuring sensors"))
	{
	    return 0;
	}
	
	if(hbridge_checkAllInDriverState(CONFIGURING_SENSORS_DONE))
	    return 1;
	
	hbridge_process();
    }
    return 0;
}

void hbridge_triggerActuatorConfiguration()
{
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	if(hbridge_states[i].state < STATE_SENSORS_CONFIGURED)
	{
	    //ERROR
	    printf("Error configuring Actuators\n");
	    return;
	}
    }
    
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	if(hbridge_states[i].state == STATE_ACTUATOR_ERROR)
	{
	    hbridge_sendClearActuatorError(i + RECEIVER_ID_H_BRIDGE_1);
	    hbridge_setPendingAck(i, PACKET_ID_CLEAR_ACTUATOR_ERROR);
	}
	else
	{
	    hbridge_setActuatorUnconfigured(i + RECEIVER_ID_H_BRIDGE_1);
	    hbridge_setPendingAck(i, PACKET_ID_SET_ACTUATOR_UNCONFIGURED);
	}
	
	hbridge_states[i].driverState = CONFIGURING_ACTUATORS;
    }
}


uint8_t hbridge_configureActuators()
{
    enum STATES lowestState = hbridge_getLowestHBState();
    if(lowestState < STATE_SENSORS_CONFIGURED)
    {
	printf("Error, hbridges have lowest state %i, can not configure actuators\n", lowestState);
	return 0;
    }
	
    hbridge_triggerActuatorConfiguration();
    while(1)
    {
	if(hbridge_checkTimeout(hbridge_configureTimeout, "configuring actuators"))
	{
	    return 0;
	}
	
	if(hbridge_checkAllInDriverState(CONFIGURING_ACTUATORS_DONE))
	{
	    return 1;
	}
	
	hbridge_process();
    }
    return 0;
}

uint8_t hbridge_setControllers()
{
    if(!hbridge_checkAllInDriverState(STATE_ACTUATOR_CONFIGURED))
    {
	printf("Error, driver is in wrong state\n");
	return 0;
    }
    
    if(!hbridge_checkAllInState(STATE_ACTUATOR_CONFIGURED))
    {
	printf("Error, motor driver is in wrong state\n");
	return 0;
    }
    
    //TODO SEND CORRECT CONTROLLER DATA
    
    int i;
    for(i = 0; i < hbridge_nrHbridges;i++)
    {
	hbridge_sendControllerConfiguration(i + RECEIVER_ID_H_BRIDGE_1, hbridge_controllerConfig + i);
	hbridge_setPendingAck(i, PACKET_ID_SET_ACTIVE_CONTROLLER);
    }
    
    while(1)
    {
	if(hbridge_checkTimeout(hbridge_configureTimeout, "setting controller"))
	    return 0;
	
	if(hbridge_checkAllInState(STATE_CONTROLLER_CONFIGURED))
	    return 1;
    }
    return 0;
}

void hbrige_processSingle(int hbId)
{
    struct hbridgeState *state = hbridge_states + hbId;
    switch(state->driverState)
    {
	case NONE:
	    break;
	case COM_ERROR:
	    break;
	case REQUESTING_STATE:
	    if(time_getTimeInMs() - state->sendTime > hbridge_stateReplyTimeout)
	    {
		printf("Error, motor driver %i did not reply to state request\n", hbId);
		state->driverState = COM_ERROR;
	    }
	    break;
	case GOT_STATE:
	    //do noting 
	    break;
	case CONFIGURING_SENSORS:
	    if(state->state == STATE_UNCONFIGURED)
	    {
		printf("configuring motor driver sensors\n");
		hbridge_sendSensorConfiguration(hbId + RECEIVER_ID_H_BRIDGE_1, hbridge_sensorConfig + hbId);
		hbridge_setPendingAck(hbId, hbridge_configureTimeout);
		
		state->driverState = CONFIGURING_SENSORS_SENT;
	    }
	    break;
	case CONFIGURING_SENSORS_SENT:
	    if(state->state == STATE_SENSORS_CONFIGURED)
	    {
		printf("motor driver sensors configured\n");
		state->driverState = CONFIGURING_SENSORS_DONE;
	    }
	    break;
	case CONFIGURING_SENSORS_DONE:
	    break;
	case CONFIGURING_ACTUATORS:
	    if(state->state == STATE_SENSORS_CONFIGURED)
	    {
		printf("sending motor driver actuator configuration\n");
		hbridge_sendActuatorConfiguration(hbId + RECEIVER_ID_H_BRIDGE_1, hbridge_actuatorConfig + hbId);
		hbridge_setPendingAck(hbId, hbridge_configureTimeout);
		state->driverState = CONFIGURING_ACTUATORS_SENT;
	    }
	    break;
	case CONFIGURING_ACTUATORS_SENT:	    
	    if(state->state == STATE_ACTUATOR_CONFIGURED)
	    {
		printf("motor driver actuators configured\n");
		state->driverState = CONFIGURING_ACTUATORS_DONE;
	    }
	    break;
	case CONFIGURING_ACTUATORS_DONE:
	    break;
	case CONFIGURING_CONTROLLERS:
	    if(state->state == STATE_CONTROLLER_CONFIGURED)
	    {
		printf("motor driver controller configured\n");
		state->driverState = CONFIGURING_CONTROLLERS_DONE;
	    }
	    break;
	case CONFIGURING_CONTROLLERS_DONE:
	    break;
    }
}

void hbridge_process()
{
    protocol_processPackage();
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	hbrige_processSingle(i);
    }
}

