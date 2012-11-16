#include "../src/Protocol.hpp"
#include "../src/Controller.hpp"
#include "../protocol.hpp"
#include "firmware-stubs/can_stub.hpp"
#include "../src/Reader.hpp"
#include "../src/Writer.hpp"


using namespace hbridge;
#define HBRIDGE_BOARD_ID(x) ((x + 1) << 5)

int boardId = 0;
int error = 0;

class CanbusStub : public hbridge::BusInterface
{
    virtual bool readPacket(Packet& packet)
    {
    }
    virtual bool sendPacket(const hbridge::Packet& packet)
    {
    }
    virtual bool readCanPacket(canbus::Message& packet)
    {
	bool ret = false;
	canMutex.lock();
	if(!canFromHB.empty())
	{
	    packet = canFromHB.front();
	    canFromHB.pop_front();
	    ret = true;
	}
	canMutex.unlock();    
	return ret;
    };
    
    virtual bool sendCanPacket(const canbus::Message& packet)
    {
	bool ret = false;
// 	std::cout << "Sending packet with id " << packet.can_id << std::endl;
	canMutex.lock();
	if(!canToHB.full())
	{
	    canToHB.push_back(packet);
	    ret = true;
	}
	canMutex.unlock();
	
	return ret;
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

extern "C"
{
int fw_main(void);
}

int main(int argc, char **argv)
{
    boost::thread fwThread(fw_main);
    hbridge::Protocol *proto = hbridge::Protocol::getInstance();

    proto->setBusInterface(new CanbusStub());

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
	    writer->setTargetValue(0.20);
// 	    std::cout << "SM" << std::endl;
	}
	proto->sendSharedMessages();
	usleep(10000);
    }
}