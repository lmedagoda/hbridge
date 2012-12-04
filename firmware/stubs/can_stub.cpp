extern "C" {
#include "../hbridgeCommon/drivers/can.h"
#include "../hbridgeCommon/protocol_can.h"
}
#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
#include <canmessage.hh>
#include <iostream>
boost::circular_buffer<canbus::Message> canToHB(200);
boost::circular_buffer<canbus::Message> canFromHB(200);
boost::mutex canMutex;

void CAN_Configuration(enum CAN_REMAP remap)
{
}

void CAN_CancelAllTransmits()
{
    canMutex.lock();
    
    canMutex.unlock();

}

void CAN_ConfigureFilters(enum hostIDs boardNr)
{
}

CanRxMsg canMessage;

CanRxMsg *CAN_GetNextData()
{
    bool gotMessage = false;
    canMutex.lock();
    if(!canToHB.empty())
    {
	gotMessage = true;
	canbus::Message &msg(canToHB.front());
	canMessage.StdId = msg.can_id;
	canMessage.DLC = msg.size;
	for(int i = 0; i < msg.size; i++)
	    canMessage.Data[i] = msg.data[i];
    }
    canMutex.unlock();
    
    if(gotMessage)
    {
// 	std::cout << "Received message" << std::endl;
	return &canMessage;
    }
    return 0;
}

void CAN_MarkNextDataAsRead()
{
    canMutex.lock();
    canToHB.pop_front();
    canMutex.unlock();
}

int can_send_message_hard(CanTxMsg * message)
{
    return CAN_SendMessage(message);
}

unsigned char CAN_SendMessage(CanTxMsg* TxMessage)
{
    canbus::Message msg;
    msg.can_id = TxMessage->StdId;
    msg.size = TxMessage->DLC;
    for(int i = 0; i < TxMessage->DLC; i++)
	msg.data[i] = TxMessage->Data[i];

    canMutex.lock();
    if(canFromHB.full())
    {
	canMutex.unlock();
	return 1;
    }
    canFromHB.push_front(msg);
    canMutex.unlock();

    return 0;
}