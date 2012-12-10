#include "packet_stub.hpp"
extern "C" {
#include "../common/packets.h"
}

using namespace hbridge;

boost::circular_buffer<hbridge::Packet> driverToFirmware(200);
boost::circular_buffer<hbridge::Packet> firmwareToDriver(200);
boost::mutex comMutex;

signed int firmwareSendPacket(uint16_t senderId, uint16_t packetId, const unsigned char *data, const unsigned int size)
{
    Packet msg;
    msg.senderId = senderId;
    msg.receiverId = 0;
    msg.packetId = packetId;
    msg.broadcastMsg = false;
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
// 	std::cout << "Got new Packet : "; 
// 	std::cout << getPacketName(msg.packetId) << std::endl;
// 	std::cout << "Message size " << msg.data.size() << std::endl;
// 	std::cout << "Data size " << dataSize << std::endl;;
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