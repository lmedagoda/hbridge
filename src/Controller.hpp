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
	
	sendCommandData();
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

    Controller(Writer *writer, firmware::controllerModes controllerId);
public:
    virtual void sendControllerConfig() {};
    virtual ~Controller() {};

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

    struct PID_Debug
    {
        PID_Debug() : pPart(0), iPart(0), dPart(0), minMaxPidOutput(0) {};
        signed short pPart;
        signed short iPart;
        signed short dPart;
        unsigned short minMaxPidOutput;
    };

};


/**
 * This is not a real controller, it just 
 * passed the given value directly through as
 * pwm.
 * */
class PWMController : public Controller
{
    friend class hbridge::Writer;
public:
    void setTargetValue(double value);
    virtual void processMsg(const hbridge::Packet& msg) {};
private:
    PWMController(Writer *writer);
};



class SpeedPIDController : public Controller
{
    friend class hbridge::Writer;
public:
    class Config
    {
    public:
	base::actuators::PIDValues pidValues;
    };
    
    struct Debug
    {
        Debug() : targetValue(0), pwmValue(0), encoderValue(0), speedValue(0) {};
        unsigned short targetValue;
        signed short pwmValue;
        unsigned int encoderValue;
        unsigned int speedValue;
        PID_Debug pidDebug;
    };
    
    void setConfig(const Config &config);
    void setTargetValue(double);

    virtual void processMsg(const hbridge::Packet& msg);
    virtual void printSendError(const hbridge::Packet& msg);
    virtual void sendControllerConfig();
private:
    SpeedPIDController(hbridge::Writer *writer);
    Config config;
    Debug speedControllerDebug;
};

class PosPIDController: public Controller
{
    friend class hbridge::Writer;
public:
    class Config
    {
    public:
        Config() :minHystDist(0), maxHystDist(0), 
        hysteresisActive(false), allowWrapAround(false), overDistCount(0) {};

        base::actuators::PIDValues pidValues;
        double minHystDist;
        double maxHystDist;
        bool hysteresisActive;
        bool allowWrapAround;
        short overDistCount;
    };
    
    struct Debug
    {
        Debug() : targetValue(0), pwmValue(0), encoderValue(0), positionValue(0) {};
        unsigned short targetValue;
        signed short pwmValue;
        unsigned int encoderValue;
        unsigned int positionValue;
        PID_Debug pidDebug;
    };
    
    void setConfig(const Config &config);
    
    void setTargetValue(double value);
    
    virtual void processMsg(const hbridge::Packet& msg);
    virtual void printSendError(const hbridge::Packet& msg);
    virtual void sendControllerConfig();
private:
    PosPIDController(hbridge::Writer *writer);
    Config config;
    Debug positionControllerDebug;
};

}
#endif // CONTROLLER_HPP
