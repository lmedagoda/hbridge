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
    CanDriver(const std::string &dev)
    {
	driver = canbus::openCanDevice(dev);	
    };
    
    virtual bool readCanMsg(canbus::Message& msg)
    {
        if(driver->getPendingMessagesCount() > 0)
	{
	    msg = driver->read();
	    return true;
	}
	
	return false;
    }
    
    virtual bool sendCanMsg(const canbus::Message& msg)
    {
	driver->write(msg);
	return true;
    }
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


    timeval start, tick;
    gettimeofday(&start, 0);

    double pwm = 0;

    if (argc == 3) {
	double pwm_in = strtod(argv[2],NULL);
	hbridge_id = strtol(argv[1],NULL,10);

	if (pwm_in >= -1.0 && pwm_in <= 1.0) {    
	    pwm = pwm_in;
	    std::cerr << "Set static PWM value to " << pwm << std::endl;
	}
	else {
	    std::cerr << "Value must be between -1.0 and 1.0" << pwm << std::endl;
	    exit(0);
	}
    } else {
	std::cerr << "usage: ./pwm_test <id> <pwm>" << std::endl;
	exit(0);
    }

    hbridge::Protocol *proto = new Protocol(new CanDriver("can0"));

    proto->setSendTimeout(base::Time::fromMilliseconds(150));
    
    hbridge::HbridgeHandle *handle = proto->getHbridgeHandle(hbridge_id);
    
    PWMController pwmCtrl(handle);
    SpeedPIDController speedCtrl(handle);
    PosPIDController posCtrl(handle);
    
    MotorConfiguration conf;
    
    Reader *reader = handle->getReader();
    Writer *writer = handle->getWriter();

    ActuatorConfiguration &accConf(writer->getActuatorConfig());
    accConf.maxPWM = 200;
    accConf.pwmStepPerMs = 200;
    accConf.openCircuit = 1;
    accConf.maxCurrent = 2000;
    accConf.maxCurrentCount = 100;
    accConf.maxBoardTemp = 80;
    accConf.maxBoardTempCount = 200;
    accConf.maxMotorTemp = 60;
    accConf.maxMotorTempCount = 200;
    accConf.controllerInputEncoder = INTERNAL;
    accConf.timeout = 200;
    
    configured = false;
    error = 0;
    
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
    
    writer->setActiveController(&pwmCtrl);

    writer->startConfigure();
    
    while(!writer->isActuatorConfigured())
    {
	proto->processIncommingPackages();
	proto->processSendQueues();
	usleep(10000);
    }
    std::cout << "Actuator Configured "<< std::endl;
    
    while(!error)
    {    
	proto->processIncommingPackages();
// 	std::cout << "SQ" << std::endl;
	proto->processSendQueues();	

	if(writer->isActuatorConfigured())
	{
	    writer->setTargetValue(pwm);
// 	    std::cout << "SM" << std::endl;
	}
	proto->sendSharedMessages();
	
	cnt++;
	
	if(cnt > 15)
	    break;
    }    
}

