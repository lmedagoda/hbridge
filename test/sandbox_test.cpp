#include "../src/Protocol.hpp"
#include "../src/Controller.hpp"
#include "../protocol.hpp"
#include "../src/Reader.hpp"
#include "../src/Writer.hpp"
#include "../firmware/stubs/packet_stub.hpp"


using namespace hbridge;

int boardId = 0;
int error = 0;

class PacketStub : public hbridge::BusInterface
{
    virtual uint16_t getMaxPacketSize()
    {
	return 8;
    }
    
    virtual bool readPacket(Packet& packet)
    {
	bool ret = false;
	comMutex.lock();
	if(!firmwareToDriver.empty())
	{
	    packet = firmwareToDriver.front();
	    firmwareToDriver.pop_front();
	    ret = true;
	}
	comMutex.unlock();    
	return ret;
    };
    
    virtual bool sendPacket(const hbridge::Packet& packet)
    {
	bool ret = false;
	
// 	std::cout << "Sending new Packet : "; 
// 	std::cout << firmware::getPacketName(packet.packetId) << std::endl;
// 	std::cout << "Message size " << packet.data.size() << std::endl;
	
// 	std::cout << "Sending packet with id " << packet.can_id << std::endl;
	comMutex.lock();
	if(!driverToFirmware.full())
	{
	    driverToFirmware.push_back(packet);
	    ret = true;
	}
	comMutex.unlock();
	
	return ret;
    };
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

int fw_main(void);

int main(int argc, char **argv)
{
    PacketStub *busInterface = new PacketStub();
    boost::thread fwThread(fw_main);
    hbridge::Protocol *proto = new hbridge::Protocol(busInterface);

    hbridge::HbridgeHandle *handle = proto->getHbridgeHandle(boardId);
    
    PWMController pwmCtrl(handle);
    SpeedPIDController speedCtrl(handle);
    PosPIDController posCtrl(handle);
    
    MotorConfiguration conf;
    
    configured = false;
    
    Reader *reader = handle->getReader();
    Writer *writer = handle->getWriter();
    reader->setCallbacks(new DummyCallback());
    reader->setConfiguration(conf);
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

	if(reader->isWritable())
	{
	    writer->setActiveController(&speedCtrl);
	    writer->setTargetValue(0.20);
// 	    std::cout << "SM" << std::endl;
	}
	proto->sendSharedMessages();
	usleep(10000);
    }
}