#include "protocol.h"
#include "printf.h"
#include "packets.h"

#define MAX_PACKET_SIZE 30



uint8_t protocolBuffer[MAX_PACKET_SIZE];
struct LowPrioHeader protocolHeaderBuffer;
struct LowPrioHeader *curHeader;
uint8_t received = 0;

extern void (*protocolHandlers[PACKET_ID_TOTAL_COUNT])(int id, unsigned char *data, unsigned short size);



uint8_t protocol_sendLowPrio(uint16_t id, uint8_t *data, uint8_t size)
{
    //send header
    struct LowPrioPacket packet;
    packet.type = TYPE_HEADER;
    
    struct LowPrioHeader *header = (struct LowPrioHeader *) (data + sizeof(struct LowPrioPacket));
    header->id = id;
    header->size = size;
    //TODO 
    header->crc = 0;

    uint8_t ret = 0;
    ret |= protocol_sendData(PACKET_LOW_PRIORITY_DATA, (uint8_t *) &packet, sizeof(struct LowPrioHeader) + 1);
    
    if(ret)
	return ret;
    
    //send data
    const int maxPayloadSize = (protocol_getMaxPacketSize() - 1);
    int numPackets = size / maxPayloadSize + 1;
    int i;
    int sent = 0;
    for(i = 0; i < numPackets; i++)
    {
	packet.type = TYPE_DATA;
	int bytesLeft = size - sent;
	int curPayloadSize = bytesLeft > maxPayloadSize ? maxPayloadSize : bytesLeft;
	int j;
	for(j = 0;j < curPayloadSize; j++)
	{
	    data[sizeof(struct LowPrioPacket) + j] = data[sent + j];
	}
	
	ret |= protocol_sendData(PACKET_LOW_PRIORITY_DATA, (uint8_t *) &packet, curPayloadSize + 1);
	if(ret)
	    return ret;
	sent += curPayloadSize;
    }
    
    return 0;    
}

void protocol_processLowPrio(int id, unsigned char *data, unsigned short size)
{
    
    const uint8_t maxPacketSize = protocol_getMaxPacketSize();
    const uint8_t maxDataSize = maxPacketSize - sizeof(struct LowPrioPacket);
    
    struct LowPrioPacket *packet = (struct LowPrioPacket *) data;

    switch(packet->type)
    {
	case TYPE_HEADER:
	{
	    if(curHeader)
		print("Warning, got new header while old packet was not complete\n");
	    
	    struct LowPrioHeader *h = (struct LowPrioHeader *) (data + sizeof(struct LowPrioPacket));
	    protocolHeaderBuffer = *h;
	    
	    curHeader = &protocolHeaderBuffer;
	    received = 0;
	    break;
	}
	case TYPE_DATA:
	{
	    if(!curHeader)
	    {
		print("Warning got header without an active packet\n");
		return;
	    }
	    
	    int i;
	    uint8_t bytesLeft = curHeader->size - received;
	    uint8_t toCopy = bytesLeft > maxDataSize ? maxDataSize : bytesLeft;
	    
	    for(i = 0;i < toCopy;i++)
	    {
		protocolBuffer[i+ received] = data[sizeof(struct LowPrioPacket) + i];
	    }
	    
	    received+=toCopy;
	    printf("LOW PRIO received %i", received);
	    if(received >= curHeader->size)
	    {
		printf("LOW PRIO GOT PACKET %i %i %i\n", curHeader->id, curHeader->size, received);
		//TODO calculate CRC
		
		//packet complete, call handler
		protocolHandlers[curHeader->id](curHeader->id, protocolBuffer, curHeader->size);

		protocol_ackPacket(curHeader->id);

		curHeader = 0;
		received = 0;
		
	    }
	
	    break;
	}
	default:
	    break;
    }
};


void protocolLowPriority_init()
{
    curHeader = 0;
    received = 0;
    //register the handler for the encapsulated protocoll
    protocol_registerHandler(PACKET_LOW_PRIORITY_DATA, protocol_processLowPrio);
};