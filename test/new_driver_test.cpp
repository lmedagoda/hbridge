#include "../src/Protocol.hpp"
#include "../src/Controller.hpp"
#include "../protocol.hpp"


using namespace hbridge;
#define HBRIDGE_BOARD_ID(x) ((x + 1) << 5)

int boardId = 0;
int error = 0;

class CanbusDummy : public hbridge::CanbusInterface
{
    std::queue<canbus::Message> fakeMsgs;
    virtual bool readCanPacket(canbus::Message& packet)
    {
	if(!fakeMsgs.empty())
	{
	    packet = fakeMsgs.front();
	    fakeMsgs.pop();
	    return true;
	}
	return false;
    };
    
    virtual bool sendCanPacket(const canbus::Message& packet)
    {
	std::cout << "Sending packet with id " << packet.can_id << std::endl;
	
	canbus::Message fakeAck;
	
	int canBoardId = HBRIDGE_BOARD_ID(boardId);

	if(packet.can_id < canBoardId)
	{
	    switch(packet.can_id)
	    {
		case firmware::PACKET_ID_SET_MODE14:
		case firmware::PACKET_ID_SET_MODE58:
		case firmware::PACKET_ID_SET_VALUE14:
		case firmware::PACKET_ID_SET_VALUE58:
		    return true;
		    break;
		default:
		    std::cout << "Got unexpected brodcast message with id " << packet.can_id << std::endl;
		    error = 1;
		    break;
	    }
	}
	
	fakeAck.can_id = firmware::PACKET_ID_ACK | canBoardId;
	firmware::ackData *ackData =
		reinterpret_cast<firmware::ackData *>(fakeAck.data);
	fakeAck.size = sizeof(firmware::ackData);
    
		    
	switch(packet.can_id - canBoardId)
	{
	    case firmware::PACKET_ID_SET_CONFIGURE:
	    case firmware::PACKET_ID_SET_CONFIGURE2:
	    case firmware::PACKET_ID_ENCODER_CONFIG_INTERN:
	    case firmware::PACKET_ID_ENCODER_CONFIG_EXTERN:
	    case firmware::PACKET_ID_SET_PID_SPEED:
	    case firmware::PACKET_ID_SET_PID_POS:
	    case firmware::PACKET_ID_POS_CONTROLLER_DATA:
		//fake ack
		ackData->packetId = packet.can_id;
		fakeMsgs.push(fakeAck);
		break;
	    default:
		std::cout << "Got unexpected message with id " << packet.can_id << std::endl;
		error = 1;
		break;
	}
	
	return true;
    };
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

    proto->setCanbusInterface(new CanbusDummy());

    hbridge::Protocol::HbridgeHandle *handle = proto->getHbridgeHandle(boardId);
    
    PWMController pwmCtrl;
    SpeedPIDController speedCtrl;
    PosPIDController posCtrl;
    
    proto->registerController(firmware::CONTROLLER_MODE_PWM, pwmCtrl);
    proto->registerController(firmware::CONTROLLER_MODE_SPEED, speedCtrl);
    proto->registerController(firmware::CONTROLLER_MODE_POSITION, posCtrl);
    
 
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
	    writer->setController(firmware::CONTROLLER_MODE_SPEED);
	    writer->setTargetValue(10.0);
	    std::cout << "SM" << std::endl;
	}
	proto->sendSharedMessages();
    }
}