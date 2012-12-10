#ifndef WRITER_HPP
#define WRITER_HPP
#include <canmessage.hh>
#include <vector>
#include "../HBridge.hpp"
#include "Protocol.hpp"

namespace hbridge
{

class Protocol;
class Controller;
class HbridgeHandle;

class WriterState;

class Writer:public PacketReveiver
{
    friend class Controller;
public:
    class CallbackInterface
    {
    public:
	/**
	* Callback, it is called as soon as the configuration packages
	* have been ACKed by the hardware
	* 
	* */
	virtual void configureDone() {};
	
	/**
	* This callback will be called if the configuration 
	* fails. The reason of the failure as well as the id 
	* of the device will passed as argument.
	* */
	virtual void configurationError() {};
	
	/**
	* This callback will be called if the hardware enters an error state.
	* */
	virtual void actuatorError() {};
    };
    
private:
    WriterState *state;
    friend class HbridgeHandle;
    Controller *curController;
    
    friend class Protocol;
    Writer(HbridgeHandle *handle);
    HbridgeHandle *handle;

    ActuatorConfiguration actuatorConfig;
    CallbackInterface *callbacks;
    void sendActuatorConfig();
    void sendController();
    void sendControllerConfig();
    
    void configurationError(const Packet &msg);
public:
    void startConfigure();
    bool isActuatorConfigured();
    
    bool hasError();
    
    virtual void processMsg(const Packet& msg);
    
    void setActiveController(Controller *ctrl);
    void setTargetValue(double value);
};

}
#endif // WRITER_HPP
