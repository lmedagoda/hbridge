#include "packethandling.h"
#include "mainboardstate.h"
#include "printf.h"

#ifndef STUBMODE
#include "../../hbridgeCommon/drivers/can.h"
#include <hbridgeCommon/lib/STM32F10x_StdPeriph_Driver/inc/stm32f10x_can.h>
#endif

#include "timeout.h"
#include "mb_types.h"
#include "../common/time.h"

#define PACKET_MAX_HANDLERS 16

packet_callback_t packet_handlers[PACKET_MAX_HANDLERS];

uint16_t packet_packetCnt;
uint16_t packet_packetsInLastSecond;
uint32_t packet_cntStartTime = 0;

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
    printf("Got a Set State\n");
    timeout_reset();
//     assert(sizeof(enum MAINBOARDSTATE) == size);
    
    enum MAINBOARDSTATE wantedState = (enum MAINBOARDSTATE) *data;
    if(wantedState == mbstate_getCurrentState())
	return;
    
    printf("Trying to switch to state %i from state %i \n", wantedState, mbstate_getCurrentState());
    if(!mbstate_changeState(wantedState))
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
    msg.RTR=CAN_RTR_DATA;
    msg.IDE=CAN_ID_STD;
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
    //printf("got a can packet\n");

    const uint8_t payloadSize = size-2;
    CanTxMsg msg;
    msg.StdId = data[0] << 3;
    msg.RTR=CAN_RTR_DATA;
    msg.IDE=CAN_ID_STD;
    msg.DLC = payloadSize;
    int i;

    //copy down data as first byte is the CAN id
    for(i=0; i < payloadSize; i++) {
	msg.Data[i] = data[i + 1];
    }
    
    CAN_SendMessage(&msg);
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
    //TODO WHERE ?
    
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
	printf("Packet: ERROR, registered handler with to high id %i max id is %i\n", id, PACKET_MAX_HANDLERS);
        return;
    }
    packet_handlers[id] = callback;
}

uint16_t packet_getPacketsInLastSecond()
{
    return packet_packetsInLastSecond;
}

void packet_handlePacket(int senderId, int receiverId, int id, unsigned char* data, short unsigned int size)
{
    uint32_t curTime = time_getTimeInMs();
    
    if(curTime - packet_cntStartTime > 1000)
    {
        packet_packetsInLastSecond = packet_packetCnt;
        packet_packetCnt = 0;
        packet_cntStartTime = curTime;
    }
    
    packet_packetCnt++;
    
    if (id >= PACKET_MAX_HANDLERS){
	printf("Packet: ERROR, tried to handle packet with to high id\n");
        return;
    }
    
    packet_handlers[id](senderId, receiverId, id, data, size);
}

