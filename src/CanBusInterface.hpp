#ifndef CANBUSINTERFACE_HPP
#define CANBUSINTERFACE_HPP

#include "Protocol.hpp"
#include "canmessage.hh"

namespace hbridge {

class CanBusInterface: public BusInterface
{

public:
    CanBusInterface();
    virtual uint16_t getMaxPacketSize();
    virtual bool readPacket(Packet& packet);
    virtual bool sendPacket(const hbridge::Packet& packet);
    
    virtual bool readCanMsg(canbus::Message& msg) = 0;
    virtual bool sendCanMsg(const canbus::Message &msg) = 0;

};

}

#endif // CANBUSINTERFACE_HPP
