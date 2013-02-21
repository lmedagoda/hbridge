#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <canmessage.hh>
#include <vector>
#include <base/actuators/commands.h>
#include "MotorDriverTypes.hpp"
#include "Protocol.hpp"

namespace hbridge 
{

class Writer;
    
class Controller: public PacketReceiver
{
private:
    firmware::controllerModes mode;
    Writer *writer;
    const static unsigned int maxCommandSize = 8;
    std::vector<uint8_t> commandData;
    bool hasCommand;
protected:
    virtual void printSendError(int packetId) {};
    
    void clearCommand()
    {
	hasCommand = false;
    };
    
    template <class A>
    void sendCommand(const A &command)
    {
	if(sizeof(command) > maxCommandSize)
	    throw std::runtime_error("Error: Command is too big");
	
	commandData.resize(sizeof(command));
	A *ptr = reinterpret_cast<A *>(commandData.data()); 
	*ptr = command;
	hasCommand = true;
    }
    
    /**
     * This function checks if there is valid command data
     * if this is the case, it will send it out.
     * */
    void sendCommandData();
    
    /**
     * Registeres the controller for the given packet Id.
     * Whenever a packet packet with this id is received,
     * processMsg will be called;
     * */
    void registerForPacketId(int packetId);
    
    /**
     * Callback, gets called if an error while sending
     * a packet message occured
     * */
    void packetSendError(const Packet &msg);
    
    
    /**
     * Convenience send method. Sends out the given message.
     * The receiver, sender will be filled in automatically
     * */
    void sendPacket(const Packet &msg, bool isAcked);
    
public:
    Controller(Writer *writer, firmware::controllerModes controllerId);
    virtual void sendControllerConfig() {};

    const std::vector<uint8_t> *getCommandData()
    {
	if(!hasCommand)
	    return NULL;
	
	return &commandData;
    }
    
    /**
     * Switched the hbrige to this controller by using 
     * the set mode message;
     * */
    void activateController();
    
    firmware::controllerModes getControllerId() const
    {
	return mode;
    };

};


/**
 * This is not a real controller, it just 
 * passed the given value directly through as
 * pwm.
 * */
class PWMController : public Controller
{
public:
    PWMController(Writer *writer);
    void setTargetValue(double value);
    virtual void processMsg(const hbridge::Packet& msg) {};
private:
};



class SpeedPIDController : public Controller
{
public:
    class Config
    {
    public:
	base::actuators::PIDValues pidValues;
    };
    SpeedPIDController(hbridge::Writer *writer);
    void setConfig(const Config &config);
    void setTargetValue(double);

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
    PosPIDController(hbridge::Writer *writer);
    void setConfig(const Config &config);
    
    void setTargetValue(double value);
    
    virtual void processMsg(const hbridge::Packet& msg);
    virtual void printSendError(const hbridge::Packet& msg);
    virtual void sendControllerConfig();
private:
    Config config;
    PositionControllerDebug positionControllerDebug;
};

}
#endif // CONTROLLER_HPP
