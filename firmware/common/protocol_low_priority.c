#include "protocol.h"
#include "printf.h"
#include "packets.h"

#define MAX_PACKET_SIZE 30

enum LOW_PRIO_TYPE
{
    ///Header, containing description of the upcomming data packets
    TYPE_HEADER = 0,
//     ///Everything is fine, message received
//     TYPE_ACK,
//     ///Message was broken, resend    
//     TYPE_REQUEST_RESEND,
//     ///Firmware does not know this message
//     TYPE_NACK,
    ///Data packet(s) followed by header
    TYPE_DATA,
};

struct LowPrioHeader
{
    enum LOW_PRIORITY_IDs id:8;
    ///Size in bytes of the data
    ///Note the data may be distributed over multiple packets
    uint8_t size;
    ///checksum of the data
    uint16_t crc;
    ///Sender ID ?
    ///DO ack ?
};

struct LowPrioPacket
{
    enum LOW_PRIO_TYPE type:2;
    unsigned sequenceNumber:6;
    uint8_t data[7];
};

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
    
    struct LowPrioHeader *header = (struct LowPrioHeader *) (packet.data);
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
	    packet.data[j] = data[sent + j];
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
    
    struct LowPrioPacket *packet = (struct LowPrioPacket *) data;

    switch(packet->type)
    {
	case TYPE_HEADER:
	{
	    if(curHeader)
		print("Warning, got new header while old packet was not complete\n");
	    
	    struct LowPrioHeader *h = (struct LowPrioHeader *) packet->data;
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
	    uint8_t bytesLeft = received - curHeader->size;
	    uint8_t toCopy = bytesLeft > maxPacketSize ? maxPacketSize : bytesLeft;
	    
	    for(i = 0;i < toCopy;i++)
	    {
		protocolBuffer[i+ received] = packet->data[i];
	    }
	    
	    received+=toCopy;
	    if(received >= curHeader->size)
	    {
		//TODO calculate CRC
		
		//packet complete, call handler
		protocolHandlers[curHeader->id](curHeader->id, protocolBuffer, curHeader->size);
		
		curHeader = 0;
		received = 0;
		
		protocol_ackPacket(curHeader->id);
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