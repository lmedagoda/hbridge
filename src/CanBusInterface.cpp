#include "CanBusInterface.hpp"
#include <iostream>

namespace hbridge 
{
CanBusInterface::CanBusInterface()
{

}

uint16_t CanBusInterface::getMaxPacketSize()
{
    return 8;
}

bool CanBusInterface::readPacket(hbridge::Packet& packet)
{
    canbus::Message msg;
    if(!readCanMsg(msg))
	return false;
    
    
    packet.data.resize(msg.size);
    memcpy(packet.data.data(), msg.data, msg.size);
    
    packet.packetId = msg.can_id & 0x0F;
    packet.senderId = (msg.can_id & 0xF0) >> 4;
    //not implemented yet
    packet.receiverId = 0;
    
    packet.broadcastMsg = (msg.can_id & 0xF0) == 0;
    
    return true;
}

bool CanBusInterface::sendPacket(const hbridge::Packet& packet)
{
    canbus::Message msg;
    
    msg.size = packet.data.size();
    std::cout << "MSG Size" << (int)msg.size <<std::endl;
    memcpy(msg.data, packet.data.data(), packet.data.size());
 
    std::cout << "Is Brodcast " << packet.broadcastMsg << std::endl;
    
    msg.can_id = 0;
    msg.can_id |= packet.packetId;
    if(!packet.broadcastMsg)
	msg.can_id |= (packet.receiverId << 4);
	
    std::cout << "Recv " << packet.receiverId  << " shifted " << (packet.receiverId << 4) << std::endl;
    
    return sendCanMsg(msg);
}

}

