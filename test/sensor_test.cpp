#include <iostream>
#include <strings.h>
#include <stdlib.h>
#include <time.h>
#include <arpa/inet.h>
#include <canbus.hh>
#include "../src/CanBusInterface.hpp"
#include "../src/Reader.hpp"
#include "../src/Writer.hpp"
#include "../src/Controller.hpp"

using namespace hbridge;

class CanDriver: public CanBusInterface
{
    canbus::Driver *driver;
public:
    CanDriver(const std::string &dev):
        driver(canbus::openCanDevice(dev)),
        CanBusInterface(driver)
    {
    };
};

int hbridge_id;
int error;
bool configured;
bool reset;

class DummyCallback : public Reader::CallbackInterface
{
    virtual void configurationError()
    {
	std::cout << "Error while configuring " << std::endl;
	error = 1;
	exit(EXIT_FAILURE);
    };
    
    virtual void configureDone()
    {
	configured = true;
	std::cout << "Configuration done " << std::endl;
    }
    
    virtual void deviceReseted()
    {
	reset = true;
    }
};

int main(int argc, char *argv[]) {
    if (argc == 2) {
	hbridge_id = strtol(argv[1],NULL,10);
    } else {
	std::cerr << "usage: ./pwm_test <id>" << std::endl;
	exit(0);
    }

    hbridge::Protocol *proto = new Protocol(new CanDriver("can0"));

    proto->setSendTimeout(base::Time::fromMilliseconds(150));
    
    hbridge::HbridgeHandle *handle = proto->getHbridgeHandle(hbridge_id);
    
    MotorConfiguration conf;
    
    Reader *reader = handle->getReader();
    reader->setCallbacks(new DummyCallback());
    reader->setConfiguration(conf);
    reset = false;
    reader->resetDevice();
    while(!reset)
    {
	proto->processIncommingPackages();
	proto->processSendQueues();	
    }
    
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
    
    cnt = 0;
    
    std::cout << "Sensors Configured "<< std::endl;
    
    while(!error)
    {    
	proto->processIncommingPackages();
	proto->processSendQueues();
	proto->sendSharedMessages();
    }    
}

