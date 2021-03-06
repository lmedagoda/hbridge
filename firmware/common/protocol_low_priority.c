#include "protocol.h"
#include "printf.h"
#include "packets.h"

#define MAX_PACKET_SIZE 35
#define NUM_SENDERS 5

struct LowPrioData {
    uint8_t protocolBuffer[MAX_PACKET_SIZE];
    struct LowPrioHeader protocolHeaderBuffer;
    struct LowPrioHeader *curHeader;
    uint8_t received;
};

struct LowPrioData lowPrio_data[NUM_SENDERS];

uint8_t protocol_sendLowPrio(uint16_t senderId, uint16_t receiverId, uint16_t id, uint8_t *data, uint8_t size)
{
    uint8_t sendBuffer[protocol_getMaxPacketSize()];
    
    //send header
    struct LowPrioPacket *packet = (struct LowPrioPacket *) sendBuffer;
    packet->type = TYPE_HEADER;
    packet->sequenceNumber = 0;
    
    struct LowPrioHeader *header = (struct LowPrioHeader *) (sendBuffer + sizeof(struct LowPrioPacket));
    header->id = id;
    header->size = size;
    //TODO 
    header->crc = 0;

    uint8_t ret = 0;
    ret |= protocol_sendData(receiverId, PACKET_LOW_PRIORITY_DATA, sendBuffer, sizeof(struct LowPrioHeader) + sizeof(struct LowPrioPacket));
    
    if(!ret)
	return ret;
    //send data
    const int maxPayloadSize = (protocol_getMaxPacketSize() - sizeof(struct LowPrioPacket));
    int numPackets = (size / maxPayloadSize) + 1;
    int i;
    int sent = 0;
    for(i = 0; i < numPackets; i++)
    {
	packet->type = TYPE_DATA;
	packet->sequenceNumber = i;
	int bytesLeft = size - sent;
	int curPayloadSize = bytesLeft > maxPayloadSize ? maxPayloadSize : bytesLeft;
	int j;
	for(j = 0;j < curPayloadSize; j++)
	{
	    sendBuffer[sizeof(struct LowPrioPacket) + j] = data[sent + j];
	}
	
	ret |= protocol_sendData(receiverId, PACKET_LOW_PRIORITY_DATA, sendBuffer, curPayloadSize + sizeof(struct LowPrioPacket));
	if(!ret)
	    return ret;
	sent += curPayloadSize;
    }
    
    return size;    
}

void protocol_processLowPrio(int senderId, int receiverId, int id, unsigned char *data, unsigned short size)
{
    
    const uint8_t maxPacketSize = protocol_getMaxPacketSize();
    const uint8_t maxDataSize = maxPacketSize - sizeof(struct LowPrioPacket);
    
    struct LowPrioPacket *packet = (struct LowPrioPacket *) data;

    struct LowPrioData *lpd = lowPrio_data + senderId;
    
    switch(packet->type)
    {
	case TYPE_HEADER:
	{
	    if(lpd->curHeader)
		printf("Warning, got new header while old packet was not complete\n");
	    
	    struct LowPrioHeader *h = (struct LowPrioHeader *) (data + sizeof(struct LowPrioPacket));
	    lpd->protocolHeaderBuffer = *h;
	    
	    lpd->curHeader = &(lpd->protocolHeaderBuffer);
	    lpd->received = 0;
	    
	    if(lpd->curHeader->size > MAX_PACKET_SIZE)
	    {
		printf("ERROR: Packet is too big, discarding packet");
		lpd->curHeader = 0;
	    }
	    break;
	}
	case TYPE_DATA:
	{
	    if(!lpd->curHeader)
	    {
		printf("Warning got header without an active packet\n");
		return;
	    }
	    
	    int i;
	    uint8_t bytesLeft = lpd->curHeader->size - lpd->received;
	    uint8_t toCopy = bytesLeft > maxDataSize ? maxDataSize : bytesLeft;
	    
	    for(i = 0;i < toCopy;i++)
	    {
		lpd->protocolBuffer[i+ lpd->received] = data[sizeof(struct LowPrioPacket) + i];
	    }
	    
	    lpd->received+=toCopy;
	    if(lpd->received >= lpd->curHeader->size)
	    {
		//TODO calculate CRC
		
		//packet complete, call handler
		protocol_checkCallHandler(senderId, receiverId, lpd->curHeader->id, lpd->protocolBuffer, lpd->curHeader->size);

		lpd->curHeader = 0;
		lpd->received = 0;
	    }
	
	    break;
	}
	default:
	    break;
    }
};


void protocolLowPriority_init()
{
    int i;
    for(i = 0; i < NUM_SENDERS; i++)
    {
	lowPrio_data[i].curHeader = 0;
	lowPrio_data[i].received = 0;
    }
    //register the handler for the encapsulated protocoll
    protocol_registerHandler(PACKET_LOW_PRIORITY_DATA, protocol_processLowPrio);
};