#include "../src/Protocol.hpp"
#include "../src/Controller.hpp"
#include "../protocol.hpp"
#include "../src/Reader.hpp"
#include "../src/Writer.hpp"


using namespace hbridge;

int boardId = 0;
int error = 0;

class CanbusDummy : public hbridge::BusInterface
{
    LowPriorityProtocol *lowPrioProt;
    std::queue<Packet> fakeMsgs;
public:
    CanbusDummy(): lowPrioProt(new LowPriorityProtocol(new Protocol(this)))
    {
    }
    
    virtual uint16_t getMaxPacketSize()
    {
	return 8;
    }
    
    virtual bool readPacket(Packet& packet)
    {
	if(!fakeMsgs.empty())
	{
	    packet = fakeMsgs.front();
	    fakeMsgs.pop();
	    return true;
	}
	return false;
    }
    
    bool handleLowPrio(const hbridge::Packet& packet)
    {
	const Packet *pkg = lowPrioProt->processPackage(packet);	
	if(!pkg)
	{
	   return false;
	}
	
	Packet fakeAck;	

	fakeAck.packetId = firmware::PACKET_ID_ACK;
	fakeAck.receiverId = pkg->senderId;
	assert(pkg->senderId == boardId);
	fakeAck.data.resize(sizeof(firmware::ackData));
	firmware::ackData *ackData =
	    reinterpret_cast<firmware::ackData *>(fakeAck.data.data());
    
	switch(pkg->packetId)
	{
	    case firmware::PACKET_ID_SET_BASE_CONFIG:
	    case firmware::PACKET_ID_SET_ACTIVE_CONTROLLER:
	    case firmware::PACKET_ID_SET_SPEED_CONTROLLER_DATA:
	    case firmware::PACKET_ID_SET_POS_CONTROLLER_DATA:
		//fake ack
		ackData->packetId = pkg->packetId;
		fakeMsgs.push(fakeAck);
		return true;
		break;
	    default:
		std::cout << "CanbusDummy : Got unexpected message with id " << pkg->packetId << std::endl;
		error = 1;
		break;
	}

	return false;
    }

    virtual bool sendPacket(const hbridge::Packet& packet)
    {
	switch(packet.packetId)
	{
	    case firmware::PACKET_ID_SET_VALUE14:
	    case firmware::PACKET_ID_SET_VALUE58:
// 		std::cout << "Sending packet with id " << packet.packetId << std::endl;
		return true;
		break;
	    case firmware::PACKET_ID_SET_BASE_CONFIG:
	    case firmware::PACKET_ID_SET_ACTIVE_CONTROLLER:
	    case firmware::PACKET_ID_SET_SPEED_CONTROLLER_DATA:
	    case firmware::PACKET_ID_SET_POS_CONTROLLER_DATA:
	    {
		Packet fakeAck;	

		fakeAck.packetId = firmware::PACKET_ID_ACK;
		fakeAck.receiverId = packet.senderId;
		fakeAck.senderId = packet.receiverId;
		fakeAck.broadcastMsg = false;
		assert(pkg->senderId == boardId);
		fakeAck.data.resize(sizeof(firmware::ackData));
		firmware::ackData *ackData =
		    reinterpret_cast<firmware::ackData *>(fakeAck.data.data());

		//fake ack
		ackData->packetId = packet.packetId;
		fakeMsgs.push(fakeAck);
		return true;
	    }
		break;

	    case firmware::PACKET_LOW_PRIORITY_DATA:
		return handleLowPrio(packet);
		break;
	    
	    default:
		std::cout << "CanbusDummy: Got unexpected brodcast message with id " << packet.packetId << std::endl;
		error = 1;
		break;
	}
	return false;
    }
};


class DummyCallback : public hbridge::Reader::CallbackInterface
{
    virtual void configurationError()
    {
	std::cout << "Error while configuring " << std::endl;
	error = 1;
    };
    
    virtual void configureDone()
    {
	std::cout << "Configuration done " << std::endl;
    }
};

int main(int argc, char **argv)
{

    
    hbridge::Protocol *proto = new Protocol(new CanbusDummy());

    hbridge::HbridgeHandle *handle = proto->getHbridgeHandle(boardId);
    
    PWMController pwmCtrl(handle);
    SpeedPIDController speedCtrl(handle);
    PosPIDController posCtrl(handle);
    
    MotorConfiguration conf;
    
    Reader *reader = handle->getReader();
    Writer *writer = handle->getWriter();
    reader->setCallbacks(new DummyCallback());
    reader->setConfiguration(conf);
    reader->startConfigure();
    
    int cnt = 0;
    
    while(!error)
    {    
	proto->processIncommingPackages();
// 	std::cout << "SQ" << std::endl;
	proto->processSendQueues();	

	if(reader->isWritable())
	{
	    writer->setActiveController(&speedCtrl);
	    writer->setTargetValue(10.0);
// 	    std::cout << "SM" << std::endl;
	}
	proto->sendSharedMessages();
	
	cnt++;
	
	if(cnt > 15)
	    break;
    }
}
