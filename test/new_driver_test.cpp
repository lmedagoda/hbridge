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
    std::queue<Packet> fakeMsgs;
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
    
    virtual bool sendPacket(const hbridge::Packet& packet)
    {
	std::cout << "Sending packet with id " << packet.packetId << std::endl;
	
	Packet fakeAck;
	
// 	switch(packet.can_id)
// 	{
// 	    case firmware::PACKET_ID_SET_MODE:
// 	    case firmware::PACKET_ID_SET_VALUE14:
// 	    case firmware::PACKET_ID_SET_VALUE58:
// 		return true;
// 		break;
// 	    default:
// 		std::cout << "Got unexpected brodcast message with id " << packet.can_id << std::endl;
// 		error = 1;
// 		break;
// 	}
// 	
// 	fakeAck.can_id = firmware::PACKET_ID_ACK | canBoardId;
// 	firmware::ackData *ackData =
// 		reinterpret_cast<firmware::ackData *>(fakeAck.data);
// 	fakeAck.size = sizeof(firmware::ackData);
//     
// 		    
// 	switch(packet.can_id - canBoardId)
// 	{
// 	    case firmware::PACKET_ID_SET_CONFIGURE:
// 	    case firmware::PACKET_ID_SET_CONFIGURE2:
// 	    case firmware::PACKET_ID_ENCODER_CONFIG_INTERN:
// 	    case firmware::PACKET_ID_ENCODER_CONFIG_EXTERN:
// 	    case firmware::PACKET_ID_SET_PID_SPEED:
// 	    case firmware::PACKET_ID_SET_PID_POS:
// 	    case firmware::PACKET_ID_POS_CONTROLLER_DATA:
// 		//fake ack
// 		ackData->packetId = packet.can_id;
// 		fakeMsgs.push(fakeAck);
// 		break;
// 	    default:
// 		std::cout << "Got unexpected message with id " << packet.can_id << std::endl;
// 		error = 1;
// 		break;
// 	}
	
	return true;
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

    
    hbridge::Protocol *proto = hbridge::Protocol::getInstance();

    proto->setBusInterface(new CanbusDummy());

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
    
    while(!error)
    {    
	proto->processIncommingPackages();
// 	std::cout << "SQ" << std::endl;
	proto->processSendQueues();	

	if(reader->isWritable())
	{
	    writer->setActiveController(&speedCtrl);
	    writer->setTargetValue(10.0);
	    std::cout << "SM" << std::endl;
	}
	proto->sendSharedMessages();
    }
}