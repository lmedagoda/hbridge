#include "protocol.h"
#include "packets.h"
#include "printf.h"
#include "protocol_low_priority.h"
#include <stdio.h>

uint8_t maxPacketSize;
volatile enum hostIDs ownHostId;
protocol_callback_t protocolHandlers[PACKET_ID_TOTAL_COUNT];

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

void protocol_ackHandler(int id, unsigned char *data, unsigned short size)
{
    //We do not support error handling or resend support on the
    //motor driver firmware. So we just ignore the ack.
}

enum hostIDs protocol_getOwnHostId()
{
    return ownHostId;
}

void protocol_defaultHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size)
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



void protocol_registerHandler(int id, protocol_callback_t handler)
{
    protocolHandlers[id] = handler;
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
        //printf("There is something: %i\n", senderId);
	if(receiverId == ownHostId || receiverId == RECEIVER_ID_ALL || ownHostId == -1)
	{
        //printf("Got Packet\n");
	    if(packetId > PACKET_ID_TOTAL_COUNT)
	    {
		printf("Error, got packet with to big packet id\n");
		return;
	    }

	    protocolHandlers[packetId](senderId, receiverId, packetId, buffer, bytes);
	}
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
        printf("sende LOW PRIO\n");
 	return protocol_sendLowPrio(ownHostId, receiverId, id, data, size);
    } else if(id == PACKET_LOW_PRIORITY_DATA) {
        //senderId == recieverId in Low-Prio-case
        printf("sende MIDDLE PRIO\n");
        return sendPacket(receiverId, receiverId, id, data, size);
    }
    {
	if(size > maxPacketSize)
	{
	    printf("Error, packet to big for high priority transmission");
	}
    printf("sende HIGH PRIO: %i an %i\n", id, receiverId);
	return sendPacket(ownHostId, receiverId, id, data, size);
    }
    
    return 0;
}
