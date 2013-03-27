#include <iostream>
#include <strings.h>
#include <stdlib.h>
#include <time.h>
#include <arpa/inet.h>
#include <canbus.hh>
#include <canmessage.hh>
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
        CanBusInterface(driver),
        driver(canbus::openCanDevice(dev))
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
    canbus::Driver *driver = canbus::openCanDevice("can0"); 
    CanBusInterface *interface = new CanBusInterface(driver);
    hbridge::Protocol *proto = new Protocol(interface);

    proto->setSendTimeout(base::Time::fromMilliseconds(350));
    
    proto->setDriverAsBusMaster();
    
    Reader *reader = new Reader(hbridge_id, proto);
    
    MotorConfiguration conf;
    
    reader->setConfiguration(conf.sensorConfig);
    reader->startConfigure();
    
    int cnt = 0;
    while(!reader->isConfigured())
    {
	if(reader->hasError())
	{
	    std::cout << "Reader reported error " << std::endl;
	    exit(EXIT_FAILURE);
	}
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

