#ifndef CANBUSINTERFACE_HPP
#define CANBUSINTERFACE_HPP

#include "Protocol.hpp"
#include "canmessage.hh"
#include "canbus.hh"
namespace hbridge {

    class CanBusInterface: public BusInterface
    {

        public:
            CanBusInterface(canbus::Interface *interface);
	    virtual ~CanBusInterface() 
	    {
		delete interface;
	    };
            virtual uint16_t getMaxPacketSize();
            virtual bool readPacket(Packet& packet);
            virtual bool sendPacket(const hbridge::Packet& packet);


            virtual bool readCanMsg(canbus::Message& msg){
                assert(interface);
                return interface->readCanMsg(msg);
            };

            virtual bool sendCanMsg(const canbus::Message &msg){
                assert(interface);
                return interface->sendCanMsg(msg);
            }

        private:
            canbus::Interface *interface;

    };

}

#endif // CANBUSINTERFACE_HPP
