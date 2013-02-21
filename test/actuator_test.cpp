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

class CanDriver: public CanBusInterface{
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
    canbus::Driver *driver = canbus::openCanDevice("can0"); 
    CanBusInterface *interface = new CanBusInterface(driver);
    hbridge::Protocol *proto = new Protocol(interface);


    //hbridge::Protocol *proto = new Protocol(new CanDriver("can0"));

    proto->setSendTimeout(base::Time::fromMilliseconds(150));
    
    Writer *writer = new Writer(hbridge_id, proto);
    
    PWMController pwmCtrl(writer);
    SpeedPIDController speedCtrl(writer);
    PosPIDController posCtrl(writer);

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

    writer->setActiveController(&pwmCtrl);

    writer->startConfigure();
    
    while(!writer->isActuatorConfigured())
    {
	proto->processIncommingPackages();
	proto->processSendQueues();
	usleep(10000);
    }
    
    if(writer->hasError())
	exit(EXIT_FAILURE);
    
    std::cout << "Actuator Configured "<< std::endl;
    
    while(!writer->hasError())
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
	
    }    
}

