#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <canmessage.hh>
#include <vector>
#include <base/actuators/commands.h>
#include "../protocol.hpp"
#include "../HBridge.hpp"
#include "Protocol.hpp"

namespace hbridge 
{

class Reader;
    
class Controller: public PacketReveiver
{
public:
    Controller(HbridgeHandle *handle, firmware::controllerModes controllerId);
    virtual void sendControllerConfig() {};
    virtual void printSendError(int packetId) {};
    
    virtual unsigned short getTargetValue(double value) = 0;
    
    /**
     * Registeres the controller for the given packet Id.
     * Whenever a packet packet with this id is received,
     * processMsg will be called;
     * */
    void registerForCanId(int packetId);
    
    /**
     * Callback, gets called if an error while sending
     * a packet message occured
     * */
    void packetSendError(const Packet &msg);
    
    /**
     * Switched the hbrige to this controller by using 
     * the set mode message;
     * */
    void activateController();
    
    /**
     * Transmittes the given value to the hbridge
     * */
    void setTargetValue(double value);
    
    firmware::controllerModes getControllerId() const
    {
	return mode;
    };
    
    void sendPacket(const Packet &msg, bool isAcked);
private:
    firmware::controllerModes mode;
    HbridgeHandle *handle;
};


/**
 * This is not a real controller, it just 
 * passed the given value directly through as
 * pwm.
 * */
class PWMController : public Controller
{
public:
    PWMController(HbridgeHandle* handle);
    virtual void processMsg(const hbridge::Packet& msg) {};
private:
    virtual short unsigned int getTargetValue(double value);
};



class SpeedPIDController : public Controller
{
public:
    class Config
    {
    public:
	base::actuators::PIDValues pidValues;
    };
    SpeedPIDController(hbridge::HbridgeHandle* handle);
    void setConfig(const Config &config);
    virtual short unsigned int getTargetValue(double value);
    virtual void processMsg(const hbridge::Packet& msg);
    virtual void printSendError(const hbridge::Packet& msg);
    virtual void sendControllerConfig();
private:
    Config config;
    SpeedControllerDebug speedControllerDebug;
};

class PosPIDController: public Controller
{
public:
    class Config
    {
    public:
	base::actuators::PIDValues pidValues;
	PositionControllerConfiguration posCtrlConfig;
    };
    PosPIDController(hbridge::HbridgeHandle* handle);
    void setConfig(const Config &config);
    virtual short unsigned int getTargetValue(double value);
    virtual void processMsg(const hbridge::Packet& msg);
    virtual void printSendError(const hbridge::Packet& msg);
    virtual void sendControllerConfig();
private:
    Config config;
    PositionControllerDebug positionControllerDebug;
};

}
#endif // CONTROLLER_HPP
