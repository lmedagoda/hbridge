#ifndef WRITER_HPP
#define WRITER_HPP
#include <canmessage.hh>
#include <vector>
#include "MotorDriverTypes.hpp"
#include "Protocol.hpp"
#include <base/JointState.hpp>

namespace hbridge
{

class Protocol;
class Controller;

class WriterState;

class Writer:public PacketReceiver
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
    Controller *curController;
    std::vector<Controller *> controllers;
    std::vector<PacketReceiver *> msgHandlers;
    
    bool driverError;
    
    friend class Protocol;
    uint32_t boardId;
    Protocol *protocol;

    ActuatorConfiguration actuatorConfig;
    CallbackInterface *callbacks;
    void sendActuatorConfig();
    void sendController();
    void sendControllerConfig();
    void controllersConfiguredCallback();
    void controllerSetCallback();
    
    void configurationError(const Packet &msg);
    void registerController(hbridge::Controller *ctrl);
    
    void registerForMsg(PacketReceiver *reveiver, int packetId);

    void processStateAnnounce(const Packet& msg);
public:
    Writer(uint32_t boardId, Protocol *protocol);
    
    virtual ~Writer();
    
    void startConfigure();
    
    ActuatorConfiguration &getActuatorConfig()
    {
	return actuatorConfig;
    }
    
    /**
     * Returns weather the device is in a state where
     * target values might be written to the controllers.
     * */
    bool isActuatorConfigured();
    
    void requestDeviceState();

    bool hasError();
    
    /**
     * Reset the actiuator config of the motor driver.
     * This clears any error that are actuator specific.
     * */
    void resetActuator();
    
    void setConfiguration(const ActuatorConfiguration &actuatorConfig);
    
    virtual void processMsg(const Packet& msg);
    
    /**
     * Set the active controller inside the firmware.
     * Note this function MUST be called after the  
     * device was configured
     * */
    void setActiveController(Controller *ctrl);

    /**
     * Set the active controller using one
     * of the previous registered Controllers
     * with the correct id.
     * */
    void setActiveController(base::JointState::MODE id);
    
    /**
     * Return the active controller
     * */
    Controller *getActiveController();

    /**
     * Returns wheather the firmware received the
     * command to the a controller. 
     * */
    bool isControllerSet();    
    
    /**
     * Returns wheather the driver is trying to
     * set the controller in the motor driver
     * */
    bool isControllerConfiguring();    
    uint32_t getId() const
    {
        return boardId;
    }
};

}
#endif // WRITER_HPP
