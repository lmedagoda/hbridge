#ifndef READER_HPP
#define READER_HPP

#include "../HBridge.hpp"
#include <canmessage.hh>
#include <boost/function.hpp>
#include "Encoder.hpp"

namespace hbridge {
class Protocol;
class Controller;

class Reader
{
friend class Protocol;
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
    Reader(int id, Protocol *protocol);
    virtual ~Reader() {};
    
    Protocol *protocol;
    CallbackInterface *callbacks;
    std::vector<boost::function<void (const canbus::Message &)> > canMsgHandlers;    
    int boardId;
    bool configured;
    
    MotorConfiguration configuration;
    
    BoardState state;
    hbridge::DRIVE_MODE current_mode;
    
    Encoder encoderIntern;
    Encoder encoderExtern;
    
    int getCurrentTickDivider() const;
    
    void fillEncoderMessage(canbus::Message &msg, const EncoderConfiguration &cfg);
    
    void sendEncConf1();
    void sendEncConf2();
    
    void sendConf1Msg();
    void sendConf2Msg();
    
    void registerCanMsgHandler(boost::function<void (const canbus::Message &)> cb, int canId);
    void unregisterCanMsgHandler(int canId);
    
    void configureDone();
    void configurationError();
    
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
    
    void processMsg(const canbus::Message &msg);
    
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
