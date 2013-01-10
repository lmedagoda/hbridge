#include "drivers/can.h"
#include "../common/protocol.h"
#include "../interfaces/host_id.h"
#include "lib/STM32F10x_StdPeriph_Driver/inc/stm32f10x_can.h"
#include "drivers/assert.h"
#include "stm32f10x_conf.h"

struct packetLayout
{
    unsigned sender:3;
    unsigned receiver:4;
    unsigned packetId:4;
}  __attribute__ ((packed)) __attribute__((__may_alias__));

signed int can_recvPacket(uint16_t *senderId, uint16_t *receiverId, uint16_t *packetId, unsigned char *data, const unsigned int dataSize)
{
    CanRxMsg *msg = CAN_GetNextData();
    
    if(!msg)
	return 0;

    assert_param(dataSize >= msg->DLC);
    
    int i;
    for(i = 0; i < msg->DLC; i++)
    {
	data[i] = msg->Data[i];
    }
    
    int ret = msg->DLC;

    struct packetLayout *pl = (struct packetLayout *) &(msg->StdId);
    if((senderId == SENDER_ID_PC) || (senderId == SENDER_ID_MAINBOARD))
    {
	*senderId = pl->sender;
	*receiverId = pl->receiver;
    }
    else
    {
	//not in case of sender id hbridge the senderid is encoded in the receiver field.
	//the receiver is implicitly all.
	*senderId = pl->receiver;
	*receiverId = SENDER_ID_MAINBOARD;
    }
    *packetId = pl->packetId;

    
    *receiverId = (msg->StdId & ~0x0F) >> 4;
    *senderId = 0;
    
    *packetId = msg->StdId & 0x0F;
    
    CAN_MarkNextDataAsRead();

    
    return ret;
}

signed int can_sendPacket(uint16_t senderId, uint16_t receiverId, uint16_t packetId, const unsigned char *data, const unsigned int size)
{
    //   assert_param(size <= sizeof(struct CanTxMsg.Data));

    CanTxMsg msg;
    
    if(!senderId)
    {
	printf("Warning, senderid may not be zero\n");
	assert_failed((uint8_t *)__FILE__, __LINE__);
    }
    
    struct packetLayout *pl = (struct packetLayout *) &(msg.StdId);
    if((senderId == SENDER_ID_PC) || (senderId == SENDER_ID_MAINBOARD))
    {
	pl->sender = senderId;
	pl->receiver = receiverId;
    }
    else
    {
	//not in case of sender id hbridge the senderid is encoded in the receiver field.
	//the receiver is implicitly all.
	pl->sender = SENDER_ID_H_BRIDGE;
	pl->receiver = senderId;
    }
    pl->packetId = packetId;
    
    msg.RTR=CAN_RTR_DATA;
    msg.IDE=CAN_ID_STD;
    msg.DLC=size;

    int i;
    for(i = 0; i < size; i++)
    {
	msg.Data[i] = data[i];
    }
    
    while(CAN_SendMessage(&msg))
	;
    
    return size;
}

void can_protocolInit()
{
    protocol_setMaxPacketSize(8);
    protocol_setRecvFunc(can_recvPacket);
    protocol_setSendFunc(can_sendPacket);
}
