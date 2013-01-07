#include "protocol.h"
#include "state.h"
#include "encoder.h"
#include "controllers.h"
#include "packets.h"
#include "printf.h"
#include <stdio.h>

uint8_t maxPacketSize;
volatile enum hostIDs ownHostId;
void (*protocolHandlers[PACKET_ID_TOTAL_COUNT])(int id, unsigned char *data, unsigned short size);

send_func_t sendPacket;
recv_func_t readPacket;

void protocol_ackPacket(int id);
// void protocol_processPackage(uint16_t id, uint8_t *data, uint8_t size);

void protocol_setSendFunc(send_func_t func)
{
    sendPacket = func;
}

void protocol_setRecvFunc(recv_func_t func)
{
    readPacket = func;
}

void protocol_ackHandler(int id, unsigned char *data, unsigned short size)
{
    //We do not support error handling or resend support on the
    //motor driver firmware. So we just ignore the ack.
}

enum hostIDs protocol_getOwnHostId()
{
    return ownHostId;
}

void protocol_defaultHandler(int id, unsigned char *data, unsigned short size)
{
    printf("Warning, packet with id %i %s was not handled \n", id, getPacketName(id));
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


void protocol_init()
{    
    int i = 0;
    for(i = 0; i < PACKET_ID_TOTAL_COUNT; i++)
    {
	protocolHandlers[i] = protocol_defaultHandler;
    }   
}



void protocol_registerHandler(int id, void (*handler)(int id, unsigned char *data, unsigned short size))
{
    protocolHandlers[id] = handler;
}

void protocol_processPackage()
{
    const uint8_t bufferSize = maxPacketSize;
    uint8_t buffer[bufferSize];
    uint16_t receiverId;
    uint16_t packetId;
    int bytes = readPacket(&receiverId, &packetId, buffer, bufferSize);
    if(bytes)
    {
	if(receiverId == ownHostId || receiverId == RECEIVER_ID_ALL)
	{
	    if(packetId > PACKET_ID_TOTAL_COUNT)
	    {
		printf("Error, got packet with to big packet id\n");
		return;
	    }

	    protocolHandlers[packetId](packetId, buffer, bytes);
	}
    }
}

void protocol_ackPacket(int id)
{
    struct ackData data;
    data.packetId = id;

    printf("acking %i id %i\n", id, id + ownHostId);

    sendPacket(ownHostId, PACKET_ID_ACK, (uint8_t *) &data, sizeof(struct ackData));
}

uint8_t protocol_sendData(int id, const unsigned char* data, short unsigned int size)
{
    if(id > PACKET_LOW_PRIORITY_DATA)
    {
// 	return protocol_sendLowPrio(id, data, size);
    } else
    {
	if(size > maxPacketSize)
	{
	    printf("Error, packet to big for high priority transmission");
	}
	return sendPacket(ownHostId, id, data, size);
    }
    
    return 0;
}
