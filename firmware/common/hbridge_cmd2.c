#include "hbridge_cmd.h"
#include "hbridge_cmd2.h"
#include "printf.h"
#include <stdint.h>
#include <stddef.h>
#include "protocol.h"
#include "common/time.h"

#define MAX_HBRIDGES 8
#define MAX_CONTROLLER_DATA_SIZE 20
struct hbridge_controllerData 
{
    uint8_t hasData;
    enum controllerModes controller;
    int packetId;
    uint16_t dataSize;
    uint8_t data[MAX_CONTROLLER_DATA_SIZE];
};

int hbridge_nrHbridges;
uint32_t hbridge_configureTimeout = 3000;
uint32_t hbridge_stateReplyTimeout = 3000;
uint32_t hbridge_stateAnnounceTimeout = 3000;

struct sensorConfig hbridge_sensorConfig[MAX_HBRIDGES];
struct actuatorConfig hbridge_actuatorConfig[MAX_HBRIDGES];
struct hbridge_controllerData hbridge_controllerData[MAX_HBRIDGES];

struct hbridgeState
{
    enum STATES state;
    uint32_t stateTime;
    uint8_t pendingStateUpdate;
    uint32_t pendingStateTime;
    
    uint8_t pendingAck;
    int pendingAckId;
    
    uint32_t sendTime;
};

struct hbridgeState hbridge_states[MAX_HBRIDGES];

void hbridge_setStatePending(int hbId)
{
    hbridge_states[hbId].pendingStateUpdate = 1;
    hbridge_states[hbId].pendingStateTime = time_getTimeInMs();
}

void hbridge_setAllStatePending()
{
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	hbridge_setStatePending(i);
    }
}

uint8_t hbridge_checkStateTimeout(char *state)
{
    int ret = 0;
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	uint32_t diff = time_getTimeInMs() - hbridge_states[i].pendingStateTime;
	if(diff > hbridge_stateAnnounceTimeout)
	{
	    printf("hbridge: Motor driver %i had a state timeout changing to state %s diff %i\n", i, state, diff);
	    ret = 1;
	}
    }
    return ret;
}

uint8_t hbridge_checkGotStates()
{
    int ret = 1;
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	if(hbridge_states[i].pendingStateUpdate)
	    ret = 0;
    }
    return ret;
}


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

uint8_t hbridge_checkAllAcked()
{
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	if(hbridge_states[i].pendingAck)
	    return 0;
    }
    return 1;
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
    int hbId = senderId - SENDER_ID_H_BRIDGE_1;
    struct ackData *aData = (struct ackData *) data;
    
    if(!hbridge_states[hbId].pendingAck)
    {
	printf("Warning, got unexpected ack from hb %i for packet %s\n", hbId, getPacketName(aData->packetId));
	return;
    }
    
    if(hbridge_states[hbId].pendingAckId != aData->packetId)
    {
	printf("Warning, got ack for unexpected packet id %s expected %s hbId %i\n", getPacketName(aData->packetId), getPacketName(hbridge_states[hbId].pendingAckId), hbId);
	return;
    }

    printf("Got ack from hb %i for packet %s\n", hbId, getPacketName(aData->packetId));
    hbridge_states[hbId].pendingAck = 0;
}

void hbridge_stateHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size)
{
    int hbId = senderId - SENDER_ID_H_BRIDGE_1;
    if(hbId < 0 || hbId > hbridge_nrHbridges)
    {
	printf("Error, got state announce for invalid hbridge id\n");
	return;
    }
    struct announceStateData *asd = (struct announceStateData*) data;
    hbridge_states[hbId].state = asd->curState;
    hbridge_states[hbId].stateTime = time_getTimeInMs();    
    hbridge_states[hbId].pendingStateUpdate = 0;    
    printf("StateHandler hb %i s %i\n", hbId, asd->curState);
}

void hbridge_init(uint16_t numHbridges)
{
    hbridge_nrHbridges = numHbridges;
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	hbridge_states[i].pendingAck = 0;
	hbridge_states[i].pendingStateUpdate = 0;
	hbridge_controllerData[i].dataSize = 0;
	hbridge_controllerData[i].hasData = 0;
    }
    
    uint8_t *ptr = (uint8_t *) hbridge_sensorConfig;
    for(i = 0; i < sizeof(struct sensorConfig) * MAX_HBRIDGES; i++)
    {
	*ptr = 0;
	ptr++;
    }
//     memset(hbridge_sensorConfig, 0, sizeof(struct sensorConfig) * MAX_HBRIDGES);
    
    protocol_registerHandler(PACKET_ID_ANNOUNCE_STATE, hbridge_stateHandler);
    protocol_registerHandler(PACKET_ID_ACK, hbridge_ackHandler);
}

void hbridge_setControllerWithData(const uint16_t hbridgeId, enum controllerModes controller, const int packetId, const char* data, const uint8_t dataSize)
{
    if(hbridgeId > hbridge_nrHbridges)
    {
	printf("Error, trying to set controller for invalid id %i\n", hbridgeId);
	return;
    }
    
    if(dataSize > MAX_CONTROLLER_DATA_SIZE)
    {
	printf("Error, controller data size to big increase MAX_CONTROLLER_DATA_SIZE\n");
	return;
    }
    
    hbridge_controllerData[hbridgeId].dataSize = dataSize;
    int i;
    for(i = 0; i < dataSize; i++)
	hbridge_controllerData[hbridgeId].data[i] = data[i];
    
    hbridge_controllerData[hbridgeId].packetId = packetId;
    hbridge_controllerData[hbridgeId].controller = controller;
    hbridge_controllerData[hbridgeId].hasData = 1;
}

uint8_t hbridge_getControlledHbridges()
{
    return hbridge_nrHbridges;
}



void hbridge_resetActuators()
{
    int i;
    for(i=0; i < hbridge_nrHbridges; i++)
    {
	hbridge_setActuatorUnconfigured(i + RECEIVER_ID_H_BRIDGE_1);
	hbridge_setPendingAck(i, PACKET_ID_SET_ACTUATOR_UNCONFIGURED);
    }
}

void hbridge_resetSensors()
{
    int i;
    for(i=0; i < hbridge_nrHbridges; i++)
    {
	hbridge_setActuatorUnconfigured(i + RECEIVER_ID_H_BRIDGE_1);
	hbridge_setPendingAck(i, PACKET_ID_SET_UNCONFIGURED);
    }
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
	hbridge_setPendingAck(i, PACKET_ID_REQUEST_STATE);
	hbridge_setStatePending(i);
    }   
    printf("Requesting motor driver state done\n");
    
}

uint8_t hbridge_deconfigureActuator()
{
    if(hbridge_checkAllInState(STATE_SENSORS_CONFIGURED))
	return 1;
    
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	hbridge_setActuatorUnconfigured(i + RECEIVER_ID_H_BRIDGE_1);
	hbridge_setPendingAck(i, PACKET_ID_SET_ACTUATOR_UNCONFIGURED);
	hbridge_setStatePending(i);
    }

    while(!hbridge_checkAllAcked())
    {
	hbridge_process();
	
	if(hbridge_checkTimeout(hbridge_configureTimeout, "unconfigure actuators"))
	{
	    return 0;
	}
    }

    //wait for state change
    while(!hbridge_checkGotStates())
    {
	//process incomming packages
	hbridge_process();

	if(hbridge_checkStateTimeout("SENSORS_CONFIGURED"))
	{
	    return 0;
	}
    }

    while(!hbridge_checkAllInState(STATE_SENSORS_CONFIGURED))
    {
	printf("Error motor drivers in unexprected state");
	return 0;
    }
    
    return 1;
}

uint8_t hbridge_configureSensors()
{
    hbridge_requestStates();
    
    printf("before First check done\n");
    int i;
    while(!hbridge_checkGotStates())
    {
	hbridge_process();
	
	if(hbridge_checkStateTimeout("initial"))
	    return 0;
    }
    
    printf("First check done\n");

    //check if allready configured
    enum STATES lowest = hbridge_getLowestHBState();
    if(lowest >= STATE_SENSORS_CONFIGURED)
    {
	printf("Deconfiguring actuators\n");
	return hbridge_deconfigureActuator();
    }
    
    printf("Configuration motor drivers\n");
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	if(hbridge_states[i].state != STATE_UNCONFIGURED)
	{
	    hbridge_setUnconfigured(i + RECEIVER_ID_H_BRIDGE_1);
	    hbridge_setPendingAck(i, PACKET_ID_SET_UNCONFIGURED);
	    hbridge_setStatePending(i);
	}	
    }
    
    //wait for state change
    while(!hbridge_checkGotStates())
    {
	//process incomming packages
	hbridge_process();
	
	if(hbridge_checkStateTimeout("UNCONFIGURED"))
	{
	    return 0;
	}
    }
    
    if(!hbridge_checkAllInState(STATE_UNCONFIGURED))
    {
	printf("Error, motor drives in unexpected state\n");
	return 0;
    }

    printf("configuring motor driver sensors\n");
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	hbridge_sendSensorConfiguration(i + RECEIVER_ID_H_BRIDGE_1, hbridge_sensorConfig + i);
	hbridge_setPendingAck(i, PACKET_ID_SET_SENSOR_CONFIG);
	hbridge_setStatePending(i);
    }

    //wait for state change
    while(!hbridge_checkGotStates())
    {
	//process incomming packages
	hbridge_process();

	if(hbridge_checkTimeout(hbridge_configureTimeout, "configuring sensors"))
	{
	    return 0;
	}

	if(hbridge_checkStateTimeout("SENSORS_CONFIGURED"))
	{
	    return 0;
	}
    }
    
    if(!hbridge_checkAllInState(STATE_SENSORS_CONFIGURED))
    {
	printf("Error, configured sensors but motor drivers are not in state SENSORS_CONFIGURED\n");
	return 0;
    }
    
    return 1;
}

uint8_t hbridge_configureActuators()
{
    if(!hbridge_configureSensors())
	return 0;

    printf("sending motor driver actuator configuration\n");
    int i;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	hbridge_sendActuatorConfiguration(i + RECEIVER_ID_H_BRIDGE_1, hbridge_actuatorConfig + i);
	hbridge_setPendingAck(i, PACKET_ID_SET_ACTUATOR_CONFIG);
	hbridge_setStatePending(i);
    }

    //wait for state change
    while(!hbridge_checkGotStates())
    {
	//process incomming packages
	hbridge_process();
	
	if(hbridge_checkTimeout(hbridge_configureTimeout, "configure actuators"))
	    return 0;

	if(hbridge_checkStateTimeout("SENSORS_CONFIGURED"))
	{
	    return 0;
	}
    }
    
    if(!hbridge_checkAllInState(STATE_ACTUATOR_CONFIGURED))
    {
	printf("Error, motor drivers in unexpected state\n");
	return 0;
    }
    
    return 1;
}

uint8_t hbridge_configureControllers()
{
    int i;
    for(i = 0; i < hbridge_nrHbridges;i++)
    {
	if(!hbridge_controllerData[i].hasData)
	{
	    printf("Error, controller data was not set\n");
	    return 0;
	}
    }	    
    
    if(!hbridge_configureActuators())
	return 0;

    struct hbridge_controllerData *cData;
    for(i = 0; i < hbridge_nrHbridges;i++)
    {
	cData = hbridge_controllerData + i;
	if(cData->dataSize)
	{
	    protocol_sendData(i + RECEIVER_ID_H_BRIDGE_1, cData->packetId, cData->data, cData->dataSize);
	    hbridge_setPendingAck(i, cData->packetId);
	}
    }
    
    while(!hbridge_checkAllAcked())
    {
	//process incomming packages
	hbridge_process();

	if(hbridge_checkTimeout(hbridge_configureTimeout, "controller data"))
	    return 0;
    }
    
    struct setActiveControllerData setCtrl;
    for(i = 0; i < hbridge_nrHbridges;i++)
    {
	setCtrl.controllerId = hbridge_controllerData[i].controller;
	hbridge_sendControllerConfiguration(i + RECEIVER_ID_H_BRIDGE_1, &setCtrl);
	hbridge_setPendingAck(i, PACKET_ID_SET_ACTIVE_CONTROLLER);
	hbridge_setStatePending(i);
    }
    
    while(!hbridge_checkGotStates())
    {
	//process incomming packages
	hbridge_process();

	if(hbridge_checkTimeout(hbridge_configureTimeout, "setting controller"))
	    return 0;
    }
    
    if(!hbridge_checkAllInState(STATE_CONTROLLER_CONFIGURED))
    {
	printf("Error, motor driver was in unexptected state\n");
	return 0;
    }
    return 1;
}

void hbridge_process()
{
    while(protocol_processPackage())
	;
}

uint8_t hbridge_hasSensorError()
{
    int i = 0;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	if(hbridge_states[i].state == STATE_SENSOR_ERROR)
	    return 1;
    }
    return 0;
}

uint8_t hbridge_hasActuatorError()
{
    int i = 0;
    for(i = 0; i < hbridge_nrHbridges; i++)
    {
	if(hbridge_states[i].state == STATE_SENSOR_ERROR ||
	    hbridge_states[i].state == STATE_ACTUATOR_ERROR)
	    return 1;
    }
    return 0;
}

