#include "../src/Protocol.hpp"
#include "../src/Controller.hpp"
#include "../src/Reader.hpp"
#include "../src/Writer.hpp"


using namespace hbridge;

int boardId = 0;
int error = 0;

class HighLevelDummy: public hbridge::BusInterface
{
    std::queue<Packet> fakeMsgs;
    
public:
    
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
    
    virtual bool sendPacket(const hbridge::Packet& packet)
    {
	std::cout << "FakeFirmware received Packet " << firmware::getPacketName(packet.packetId) << std::endl;
	switch(packet.packetId)
	{
	    case firmware::PACKET_ID_SET_VALUE14:
	    case firmware::PACKET_ID_SET_VALUE58:
// 		std::cout << "Sending packet with id " << packet.packetId << std::endl;
		return true;
		break;
	    case firmware::PACKET_ID_SET_SENSOR_CONFIG:
	    case firmware::PACKET_ID_SET_ACTUATOR_CONFIG:
	    case firmware::PACKET_ID_SET_ACTIVE_CONTROLLER:
	    case firmware::PACKET_ID_SET_SPEED_CONTROLLER_DATA:
	    case firmware::PACKET_ID_SET_POS_CONTROLLER_DATA:
	    {
		Packet fakeAck;	

		fakeAck.packetId = firmware::PACKET_ID_ACK;
		fakeAck.receiverId = packet.senderId;
		fakeAck.senderId = packet.receiverId;
		fakeAck.broadcastMsg = false;
		std::cout << "Sender id " << packet.senderId << " receiver id " << packet.receiverId << " board id " << boardId << std::endl;
		assert(packet.receiverId == boardId);
		fakeAck.data.resize(sizeof(firmware::ackData));
		firmware::ackData *ackData =
		    reinterpret_cast<firmware::ackData *>(fakeAck.data.data());

		//fake ack
		ackData->packetId = packet.packetId;
		fakeMsgs.push(fakeAck);
		return true;
	    }
		break;

	    default:
		std::cout << "CanbusDummy: Got unexpected brodcast message with id " << packet.packetId << std::endl;
		error = 1;
		break;
	}
	return false;
    }
};

class CanbusDummy : public hbridge::BusInterface
{
    HighLevelDummy hlDummy;
    LowPriorityProtocol *lowPrioProt;
public:
    CanbusDummy(): lowPrioProt(new LowPriorityProtocol(new Protocol(&hlDummy)))
    {
    }
    
    virtual uint16_t getMaxPacketSize()
    {
	return 8;
    }
    
    virtual bool readPacket(Packet& packet)
    {
	return hlDummy.readPacket(packet);
    }
    
    void handleLowPrio(const hbridge::Packet& packet)
    {	
	const Packet *pkg = lowPrioProt->processPackage(packet);	
	if(pkg)
	{
	    hlDummy.sendPacket(*pkg);
	}

    }

    virtual bool sendPacket(const hbridge::Packet& packet)
    {
	std::cout << "Sending new Packet : "; 
	std::cout << firmware::getPacketName(packet.packetId) << std::endl;
	std::cout << "Message size " << packet.data.size() << std::endl;
	assert(packet.data.size() <= getMaxPacketSize());
	
	if(packet.packetId < firmware::PACKET_LOW_PRIORITY_DATA)
	{
	    hlDummy.sendPacket(packet);
	    return true;
	}
	
	handleLowPrio(packet);
	
	return true;
    }
};

bool configured;

class DummyCallback : public hbridge::Reader::CallbackInterface
{
    virtual void configurationError()
    {
	std::cout << "Error while configuring " << std::endl;
	error = 1;
    };
    
    virtual void configureDone()
    {
	configured = true;
	std::cout << "Configuration done " << std::endl;
    }
};

int main(int argc, char **argv)
{

    
    hbridge::Protocol *proto = new Protocol(new CanbusDummy());

    MotorConfiguration conf;
    
    Reader *reader = new Reader(boardId, proto);
    Writer *writer = new Writer(boardId, proto);
    PWMController pwmCtrl(writer);
    SpeedPIDController speedCtrl(writer);
    PosPIDController posCtrl(writer);
    
    configured = false;
    
    reader->setCallbacks(new DummyCallback());
    reader->setConfiguration(conf.sensorConfig);
    reader->startConfigure();
    
    int cnt = 0;
    while(!configured)
    {
	proto->processIncommingPackages();
	proto->processSendQueues();
	if(cnt == 500)
	{
	    std::cout << "Waiting for configuration to be done" << std::endl;
	    cnt = 0;
	}
	cnt++;
	usleep(1000);
    }
    
    while(!error)
    {    
	proto->processIncommingPackages();
// 	std::cout << "SQ" << std::endl;
	proto->processSendQueues();	

	if(writer->isActuatorConfigured())
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
