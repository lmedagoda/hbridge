#include "hbridge_cmd.h"
#include "hbridge_cmd2.h"
#include "printf.h"
#include <stdint.h>
#include <stddef.h>
#include "protocol.h"
#include "time.h"

#define MAX_HBRIDGES 8

int hbridge_nrHbridges;
uint32_t hbridge_configureTimeout = 30;
struct sensorConfig hbridge_sensorConfig[MAX_HBRIDGES];
struct actuatorConfig hbridge_actuatorConfig[MAX_HBRIDGES];

enum DRIVER_STATE
{
    NONE,
    CONFIGURING_SENSORS,
    CONFIGURING_SENSORS_SENT,
    CONFIGURING_SENSORS_DONE,
    CONFIGURING_ACTUATORS,
    CONFIGURING_ACTUATORS_SENT,
    CONFIGURING_ACTUATORS_DONE,
    
};

struct hbridgeState
{
    enum DRIVER_STATE driverState;
    uint8_t gotState;
    enum STATES state;
    uint32_t stateTime;
    
    uint8_t pendingAck;
    uint8_t pendingAckId;
    
    uint32_t sendTime;
};

struct hbridgeState hbridge_states[MAX_HBRIDGES];

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
    hbridge_states[hbId].gotState = 1;
}

void hbridge_init(uint16_t numHbridges)
{
    hbridge_nrHbridges = numHbridges;
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	hbridge_states[i].driverState = NONE;
	hbridge_states[i].gotState = 0;
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

uint8_t hbridge_configureError()
{
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	if(hbridge_states[i].pendingAck && 
	    time_getTimeInMs() - hbridge_states[i].sendTime > hbridge_configureTimeout)
	    return 1;
    }
    
    return 0;
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
	hbridge_states[i].sendTime = time_getTimeInMs();
	hbridge_states[i].pendingAck = 1;
	hbridge_states[i].pendingAckId = PACKET_ID_SET_UNCONFIGURED;
    }
}

void hbridge_triggerActuatorConfiguration()
{
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	if(!hbridge_states[i].gotState ||
	    hbridge_states[i].state < STATE_SENSORS_CONFIGURED)
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
	}
	else
	{
	    hbridge_setActuatorUnconfigured(i + RECEIVER_ID_H_BRIDGE_1);
	}
	
	hbridge_states[i].driverState = CONFIGURING_ACTUATORS;
    }
}


uint8_t hbridge_sensorsConfigured()
{
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	if(hbridge_states[i].driverState != CONFIGURING_SENSORS_DONE)
	    return 0;
    }
    
    return 1;
}


uint8_t hbridge_actuatorsConfigured()
{
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	if(hbridge_states[i].driverState != CONFIGURING_ACTUATORS_DONE)
	    return 0;
    }
    
    return 1;
}

uint8_t hbridge_configureSensors()
{
    printf("Configuration motor drivers\n");
    hbridge_triggerSensorConfiguration();
    
    uint32_t startTime = time_getTimeInMs();
    
   
    while(time_getTimeInMs() - startTime < hbridge_configureTimeout)
    {
	if(hbridge_configureError())
	{
	    return 0;
	}
	
	if(hbridge_sensorsConfigured())
	    return 1;
	
	hbridge_process();
    }
    printf("hbridge: Timeout configuring sensors\n");
    return 0;
}


uint8_t hbridge_configureActuators()
{
    while(1)
    {
	if(hbridge_actuatorsConfigured())
	{
	    return 1;
	}
	
	hbridge_process();
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

	case CONFIGURING_SENSORS:
	    if(state->gotState && state->state == STATE_UNCONFIGURED)
	    {
		printf("configuring motor driver sensors\n");
		hbridge_sendSensorConfiguration(hbId + RECEIVER_ID_H_BRIDGE_1, hbridge_sensorConfig + hbId);
		state->driverState = CONFIGURING_SENSORS_SENT;
	    }
	    break;
	case CONFIGURING_SENSORS_SENT:
	    if(state->gotState && state->state == STATE_SENSORS_CONFIGURED)
	    {
		printf("motor driver sensors configured\n");
		state->driverState = CONFIGURING_SENSORS_DONE;
	    }
	    break;
	case CONFIGURING_SENSORS_DONE:
	    break;
	case CONFIGURING_ACTUATORS:
	    if(state->gotState && state->state == STATE_SENSORS_CONFIGURED)
	    {
		printf("sending motor driver actuator configuration\n");
		hbridge_sendActuatorConfiguration(hbId + RECEIVER_ID_H_BRIDGE_1, hbridge_actuatorConfig + hbId);
		state->driverState = CONFIGURING_ACTUATORS_SENT;
	    }
	    break;
	case CONFIGURING_ACTUATORS_SENT:	    
	    if(state->gotState && state->state == STATE_ACTUATOR_CONFIGURED)
	    {
		printf("motor driver actuators configured\n");
		state->driverState = CONFIGURING_ACTUATORS_DONE;
	    }
	    break;
	case CONFIGURING_ACTUATORS_DONE:
	    break;
    }
}

void hbridge_process()
{
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	hbrige_processSingle(i);
    }
}

