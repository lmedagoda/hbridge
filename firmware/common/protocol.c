#include "protocol.h"
#include "packets.h"
#include "printf.h"
#include "protocol_low_priority.h"
#include <stdio.h>

uint8_t maxPacketSize;
volatile enum hostIDs ownHostId;
protocol_callback_t protocolHandlers[PACKET_ID_TOTAL_COUNT];

uint8_t protocol_onlyMaster;

send_func_t sendPacket;
recv_func_t readPacket;

// void protocol_processPackage(uint16_t id, uint8_t *data, uint8_t size);

void protocol_setSendFunc(send_func_t func)
{
    sendPacket = func;
}

void protocol_setRecvFunc(recv_func_t func)
{
    readPacket = func;
}

enum hostIDs protocol_getOwnHostId()
{
    return ownHostId;
}

void protocol_defaultHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size)
{
    printf("Warning, packet with id %i %s was not handled \n", id, getPacketName(id));
}

void protocol_setAllowedSenderHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size)
{
    struct setAllowedSenderData *sasd = (struct setAllowedSenderData *) data;
    protocol_onlyMaster = sasd->onlyMainboard;
}

uint8_t protocol_getMaxPacketSize()
{
    return maxPacketSize;
}

void protocol_setMaxPacketSize(uint8_t size)
{
    maxPacketSize = size;
}

void protocol_setOwnHostId(enum hostIDs id)
{
    ownHostId = id;
}


void protocol_init(int isMaster)
{   
    //initially we only listen to the bus master
    protocol_onlyMaster = !isMaster;
    int i = 0;
    for(i = 0; i < PACKET_ID_TOTAL_COUNT; i++)
    {
	protocolHandlers[i] = protocol_defaultHandler;
    }   
    
    protocolHandlers[PACKET_ID_SET_ALLOWED_SENDER] = protocol_setAllowedSenderHandler;
}



void protocol_registerHandler(int id, protocol_callback_t handler)
{
    protocolHandlers[id] = handler;
}

void protocol_setAllowedSender(enum hostIDs allowed)
{
    if(allowed == SENDER_ID_MAINBOARD)
	protocol_onlyMaster = 1;
    
    protocol_onlyMaster = 0;
}

void protocol_checkCallHandler(int senderId, int receiverId, int id, unsigned char* data, short unsigned int size)
{
    if(receiverId == ownHostId || receiverId == RECEIVER_ID_ALL)
    {
	if(id > PACKET_ID_TOTAL_COUNT)
	{
	    printf("Error, got packet with to big packet id\n");
	    return;
	}
	
	//this means that only packets from the mainboard are allowed
	if(protocol_onlyMaster && senderId != SENDER_ID_MAINBOARD 
	    //we also allowe low priority trafic
	    //the low priority handler calls us back as soon as the 
	    //packets are assembled
	    && id != PACKET_LOW_PRIORITY_DATA
	    //request state is also allways allowed
	    && id != PACKET_ID_REQUEST_STATE
	    //request sensor config ist also allowed
	    //as the reader tasks needs it
	    && id != PACKET_ID_REQUEST_SENDOR_CONFIG
	)
	{
	    printf("DNM");
	    return;
	}

	protocolHandlers[id](senderId, receiverId, id, data, size);
    }
}


void protocol_processPackage()
{
    const uint8_t bufferSize = maxPacketSize;
    uint8_t buffer[bufferSize];
    uint16_t receiverId = 0;
    uint16_t senderId = 0;
    uint16_t packetId;
    int bytes = readPacket(&senderId, &receiverId, &packetId, buffer, bufferSize);
    if(bytes)
    {
	protocol_checkCallHandler(senderId, receiverId, packetId, buffer, bytes);
    }
}

void protocol_ackPacket(int id, int receiverId)
{
    struct ackData data;
    data.packetId = id;

    printf("acking %i id %i\n", id, id + ownHostId);
    sendPacket(ownHostId, receiverId, PACKET_ID_ACK, (uint8_t *) &data, sizeof(struct ackData));
}

uint8_t protocol_sendData(int receiverId, int id, const unsigned char* data, short unsigned int size)
{
    if(id > PACKET_LOW_PRIORITY_DATA)
    {
 	return protocol_sendLowPrio(ownHostId, receiverId, id, data, size);
    } else
    {
	if(size > maxPacketSize)
	{
	    printf("Error, packet to big for high priority transmission");
	}
	return sendPacket(ownHostId, receiverId, id, data, size);
    }
    
    return 0;
}
