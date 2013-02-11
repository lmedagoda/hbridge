#include "hbridge_cmd.h"
#include "hbridge_cmd2.h"
#include "printf.h"
#include <stdint.h>
#include <stddef.h>
#include "protocol.h"

#define MAX_HBRIDGES 8

int hbridge_nrHbridges;
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
    
    uint8_t pendingAck;
    uint8_t pendingAckId;
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
    hbridge_states[hbId].gotState = 1;
}

void hbridge_init(uint16_t numHbridges)
{
    hbridge_nrHbridges = numHbridges;
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	hbridge_states[i].gotState = 0;
    }
    
    protocol_registerHandler(PACKET_ID_ANNOUNCE_STATE, hbridge_stateHandler);
    protocol_registerHandler(PACKET_ID_ACK, hbridge_ackHandler);
}

uint8_t hbridge_getControlledHbridges()
{
    return hbridge_nrHbridges;
}

uint8_t hbridge_configureError()
{
    printf("TODO IMPLEMENT ME\n");
    return 0;
}

void hbridge_resetActuators()
{
    printf("TODO IMPLEMENT ME\n");
}

void hbridge_resetSensors()
{
    printf("TODO IMPLEMENT ME\n");
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
    hbridge_triggerSensorConfiguration();
    
    while(1)
    {
	if(hbridge_configureError())
	{
	    return 0;
	}
	
	if(hbridge_actuatorsConfigured())
	    return 1;
	
	hbridge_process();
    }
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
		hbridge_sendSensorConfiguration(hbId + RECEIVER_ID_H_BRIDGE_1, hbridge_sensorConfig + hbId);
		state->driverState = CONFIGURING_SENSORS_SENT;
	    }
	    break;
	case CONFIGURING_SENSORS_SENT:
	    if(state->gotState && state->state == STATE_SENSORS_CONFIGURED)
	    {
		state->driverState = CONFIGURING_SENSORS_DONE;
	    }
	    break;
	case CONFIGURING_SENSORS_DONE:
	    break;
	case CONFIGURING_ACTUATORS:
	    if(state->gotState && state->state == STATE_SENSORS_CONFIGURED)
	    {
		hbridge_sendActuatorConfiguration(hbId + RECEIVER_ID_H_BRIDGE_1, hbridge_actuatorConfig + hbId);
		state->driverState = CONFIGURING_ACTUATORS_SENT;
	    }
	    break;
	case CONFIGURING_ACTUATORS_SENT:	    
	    if(state->gotState && state->state == STATE_ACTUATOR_CONFIGURED)
	    {
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

