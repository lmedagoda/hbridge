#include "CanBusInterface.hpp"
#include "Protocol.hpp"
#include "../firmware/common/protocol.h"
#include <iostream>

using namespace firmware;

namespace hbridge 
{

CanBusInterface::CanBusInterface(canbus::Interface *interface):
    interface(interface)
{

}

uint16_t CanBusInterface::getMaxPacketSize()
{
    return 8;
}

struct CanIdLayout
{
    unsigned packetId:4;
    unsigned receiver:4;
    unsigned sender:3;
    //unsigned sender:3;
    //unsigned receiver:4;
    //unsigned packetId:4;
}  __attribute__ ((packed)) __attribute__((__may_alias__));

bool CanBusInterface::readPacket(hbridge::Packet& packet)
{
    canbus::Message msg;
    if(!readCanMsg(msg))
	return false;
    
    
    packet.data.resize(msg.size);
    memcpy(packet.data.data(), msg.data, msg.size);

    CanIdLayout *cil = (CanIdLayout *) &(msg.can_id);
    
    packet.packetId = cil->packetId;
    if(cil->sender >= SENDER_ID_H_BRIDGE)
    {
	packet.receiverId = RECEIVER_ID_PC;
	packet.senderId = cil->receiver-RECEIVER_ID_H_BRIDGE_1;
    }
    else
    {
	packet.receiverId = cil->receiver-RECEIVER_ID_H_BRIDGE_1;
	packet.senderId = cil->sender;
    }
    
    packet.broadcastMsg = (cil->receiver == RECEIVER_ID_ALL);
    return true;
}

bool CanBusInterface::sendPacket(const hbridge::Packet& packet)
{
    canbus::Message msg;
    msg.can_id = 0;
    
    msg.size = packet.data.size();
    memcpy(msg.data, packet.data.data(), packet.data.size());
 
    CanIdLayout *cil = (CanIdLayout *) &(msg.can_id);
    
    cil->receiver = RECEIVER_ID_H_BRIDGE_1 + packet.receiverId;
    cil->packetId = packet.packetId;
    cil->sender = packet.senderId;
    
    if(packet.broadcastMsg)
	cil->receiver = RECEIVER_ID_ALL;
    
    return sendCanMsg(msg);
}

}

