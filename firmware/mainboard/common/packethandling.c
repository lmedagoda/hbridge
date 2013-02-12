#include "packethandling.h"
#include "mainboardstate.h"
#include "printf.h"
#include "../../hbridgeCommon/drivers/can.h"
#include "timeout.h"
#include <hbridgeCommon/lib/STM32F10x_StdPeriph_Driver/inc/stm32f10x_can.h>
#include "mb_types.h"

#define PACKET_MAX_HANDLERS 10

packet_callback_t packet_handlers[PACKET_MAX_HANDLERS];

void packet_pingHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size)
{
    timeout_reset();
}

void packet_statusHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size)
{
    printf("Mainboard has receive a Statuspacket, this should not happen...\n");
}

void packet_setStateHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size)
{
    timeout_reset();
//     assert(sizeof(enum MAINBOARDSTATE) == size);
    
    enum MAINBOARDSTATE wantedState = (enum MAINBOARDSTATE) *data;
    if(mbstate_changeState(wantedState))
    {
	printf("State changed: %i\n", wantedState);
    }
    else
    {
	printf("Error switching to state %i\n", wantedState);
    }
}

void packet_controlHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size)
{
    printf("Warning: Got control packet and handler was not overwritten by mainboard implementation\n");
    mbstate_changeState(MAINBOARD_OFF);
}

void packet_canSendData(struct canMsg *inMsg, unsigned short size)
{
    const uint8_t payloadSize = size-2;
    CanTxMsg msg;
    msg.StdId = inMsg->canId;
    msg.DLC = payloadSize;
    int i;
    //copy down data as first two are CAN id and index
    for(i=0; i < payloadSize; i++) {
	msg.Data[i] = inMsg->data[i];
    }
    
    CAN_SendMessage(&msg);
}

void packet_canHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size)
{
    timeout_reset();
    
    packet_canSendData((struct canMsg *) data, size);
}

void packet_canAckHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size)
{
    timeout_reset();
    
    static uint8_t lastIndex = 255;
    
    struct canMsg *inMsg = (struct canMsg *) data;

    /* prevent duplicates
     * This may happen if the mainboard just 
     * received the packet and sends the can
     * packet, at the moment the ocu times out.
     * In this case the OCU would resend the 
     * packet even it it was allready written
     * to the can bus.
     */
    if(lastIndex == inMsg->index)
	return;
    
    packet_canSendData(inMsg, size);

    //send ack
    struct canAckMsg ack;
    ack.canId = inMsg->canId;
    ack.index = inMsg->index;
    
    //TODO SEND
    
    lastIndex = inMsg->index;
}

void packet_defaultHandler(int senderId, int receiverId, int id, unsigned char *data, unsigned short size)
{
    printf("Warning, got unhandeled packet wit id %i\n", id);
}

void packet_init()
{
    int i;
    for(i=0;i<PACKET_MAX_HANDLERS; i++)
    {
	packet_handlers[i] = packet_defaultHandler;
    }
    
    packet_registerHandler(MB_PING, packet_pingHandler);
    packet_registerHandler(MB_STATUS, packet_statusHandler);
    packet_registerHandler(MB_ID_CAN, packet_canHandler);
    packet_registerHandler(MB_ID_CAN_ACK, packet_canAckHandler);
    packet_registerHandler(MB_SET_STATE, packet_setStateHandler);
    packet_registerHandler(MB_CONTROL, packet_controlHandler);
}


void packet_registerHandler(int id, packet_callback_t callback)
{
    if (id >= PACKET_MAX_HANDLERS){
	printf("ERROR, registered handler with to high id\n");
        return;
    }
    packet_handlers[id] = callback;
}

void packet_handlePacket(int senderId, int receiverId, int id, unsigned char* data, short unsigned int size)
{
    if (id >= PACKET_MAX_HANDLERS){
	printf("ERROR, tried to handle packet with to high id\n");
        return;
    }
    packet_handlers[id](senderId, receiverId, id, data, size);
    
}

