

void protocol_receiveData()
{
    CanRxMsg *msg;
    
    while(msg = CAN_GetNextData())
    {
	uint16_t packetId = msg->StdId & 0xF;
	uint16_t senderId = 0;
	uint16_t receiverId = 0;
	protocol_processPackage(packetId, msg->Data, msg->DLC);
	
	CAN_MarkNextDataAsRead();
    }
}

void protocol_sendPacket(uint8_t senderId, uint8_t packetId, uint8_t *data, uint8_t size)
{
    CanTxMsg msg;

    //send status message over CAN
    msg.StdId= packetId + ownHostId;
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
}
