#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <canmessage.hh>
#include <vector>
#include <base/actuators/commands.h>
#include "../HBridge.hpp"

namespace hbridge 
{

class Reader;
    
class Controller
{
public:
    Controller();
    virtual void sendControllerConfig() {};
    virtual void processMsg(const canbus::Message &msg) {};
    virtual void printSendError(const canbus::Message &msg) {};
    
    virtual unsigned short getTargetValue(double value) = 0;
    virtual Controller *getCopy() const = 0;
    
    void setReader(Reader *reader);

    /**
     * Returns an vector of can id that are processed
     * by this controller.
     * */
    virtual std::vector<int> getAcceptedCanIds() {return std::vector<int>();};

    /**
     * Returns an vector of can id that this controller will use to send data
     * */
    virtual std::vector<int> getSendCanIds() {return std::vector<int>();};

    void sendCanMsg(const canbus::Message &msg, bool isAcked);
private:
    Reader *reader;

};


/**
 * This is not a real controller, it just 
 * passed the given value directly through as
 * pwm.
 * */
class PWMController : public Controller
{
    virtual Controller* getCopy() const;
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
    SpeedPIDController();
    void setConfig(const Config &config);
    virtual short unsigned int getTargetValue(double value);
    virtual void processMsg(const canbus::Message& msg);
    virtual void printSendError(const canbus::Message& msg);
    virtual void sendControllerConfig();
    virtual Controller* getCopy() const;
    virtual std::vector< int > getSendCanIds();
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
    PosPIDController();
    void setConfig(const Config &config);
    virtual short unsigned int getTargetValue(double value);
    virtual void processMsg(const canbus::Message& msg);
    virtual void printSendError(const canbus::Message& msg);
    virtual void sendControllerConfig();
    virtual Controller* getCopy() const;
    virtual std::vector< int > getAcceptedCanIds();
    virtual std::vector< int > getSendCanIds();
private:
    Config config;
    PositionControllerDebug positionControllerDebug;
};

}
#endif // CONTROLLER_HPP
