// #include "../../src/Protocol.hpp"
#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
#include <iostream>

class Packet
{
public:
    int senderId;
    int receiverId;
    int packetId;
    bool broadcastMsg;
    std::vector<uint8_t> data;
};

boost::circular_buffer<Packet> driverToFirmware(200);
boost::circular_buffer<Packet> firmwareToDriver(200);
boost::mutex comMutex;

signed int firmwareSendPacket(uint16_t senderId, uint16_t packetId, const unsigned char *data, const unsigned int size)
{
    Packet msg;
    msg.senderId = senderId;
    msg.packetId = packetId;
    msg.data.resize(size);
    
    for(int i = 0; i < size; i++)
	msg.data[i] = data[i];

    comMutex.lock();
    if(firmwareToDriver.full())
    {
	comMutex.unlock();
	return 0;
    }
    firmwareToDriver.push_front(msg);
    comMutex.unlock();

    return size;  
}

signed int firmwareReceivePacket(uint16_t *senderId, uint16_t *packetId, unsigned char *data, const unsigned int dataSize)
{    
    signed int ret = 0; 
    comMutex.lock();
    if(!driverToFirmware.empty())
    {
	Packet &msg(driverToFirmware.front());
	assert(dataSize >= msg.data.size());
	*senderId = msg.senderId;
	*packetId = msg.packetId;
	ret = msg.data.size();
	for(int i = 0; i < ret; i++)
	    data[i] = msg.data[i];
	
	driverToFirmware.pop_front();
    }
    comMutex.unlock();

    return ret;
}