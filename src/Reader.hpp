#ifndef READER_HPP
#define READER_HPP

#include "../HBridge.hpp"
#include <canmessage.hh>
#include <boost/function.hpp>
#include "Encoder.hpp"
#include "Protocol.hpp"

namespace hbridge {
class Protocol;
class Controller;
class HbridgeHandle;

class Reader
{
friend class Protocol;
friend class HbridgeHandle;
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
	* This callback will be called whenever the hardware
	* sends out a new status package.
	* */
	virtual void gotStatus(const BoardState &status) {};
	
	/**
	* This callback will be called if the hardware enters an error state.
	* */
	virtual void gotErrorStatus(const ErrorState &error) {};
    };


private:
    Reader(HbridgeHandle *handle);
    virtual ~Reader() {};
    
    HbridgeHandle *handle;
    CallbackInterface *callbacks;
    std::vector<Controller *> packetHandlers;    
    bool configured;
    
    MotorConfiguration configuration;
    
    BoardState state;
    
    Encoder encoderIntern;
    Encoder encoderExtern;
    
    int getCurrentTickDivider() const;
    
    void sendConfigureMsg();
    
    void registerControllerForPacketId(Controller *ctrl, int packetId);
    void unregisterController(Controller *ctrl);
    
    void configureDone();
    void configurationError(const Packet &msg);
    
    void gotError(const ErrorState &error);
public:
    const Encoder &getInternalEncoder();
    const Encoder &getExternalEncoder();
    
    void setCallbacks(CallbackInterface *cbs);

    /**
     * Returns weather the device is in a state where
     * target values might be written to the controllers.
     * */
    bool isWritable();
    
    void processMsg(const Packet &msg);
    
    /**
    * Sets the configuration for the hardware. 
    * Must be called prior to startConfigure().
    * */
    void setConfiguration(const MotorConfiguration &config);
    
    /**
    * Send out the configuration messages.
    * */
    void startConfigure();
    
    /**
    * Shuts down the device and moves it into an 
    * unconfigured state.
    * */
    void shutdown();

    
};

}
#endif // READER_HPP
