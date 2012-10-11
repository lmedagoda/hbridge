#include "Controller.hpp"
#include "Protocol.hpp"
#include <boost/bind.hpp>
#include "../protocol.hpp"
#include "Reader.hpp"
#include <limits>

namespace hbridge 
{

Controller::Controller()
{

}

void Controller::setReader(Reader* reader)
{
    this->reader = reader;
    
    std::vector<int> ids = getAcceptedCanIds();
    
    for(std::vector<int>::const_iterator it = ids.begin(); it != ids.end(); it++)
    {
	reader->registerControllerForCanMsg(this, *it);
    }
    
    ids = getSendCanIds();
    
    for(std::vector<int>::const_iterator it = ids.begin(); it != ids.end(); it++)
    {
	reader->registerControllerForSendError(this, *it);
    }
    
}

void Controller::sendCanMsg(const canbus::Message& msg, bool isAcked)
{
    reader->protocol->sendCanPacket(reader->boardId, msg, isAcked, boost::bind(&Reader::configurationError,reader, _1));
}

unsigned short Controller::getTargetValue(double value)
{
    return 0;
}

SpeedPIDController::SpeedPIDController()
{

}

unsigned short SpeedPIDController::getTargetValue(double value)
{
    return value * std::numeric_limits<uint16_t>::max();
}

void SpeedPIDController::processMsg(const canbus::Message& msg)
{
    switch(msg.can_id)
    {
	case firmware::PACKET_ID_PID_DEBUG_SPEED:
	{
	    const firmware::pidDebugData * data =
		reinterpret_cast<const firmware::pidDebugData *> (msg.data);

	    speedControllerDebug.pidDebug.dPart = data->dPart;
	    speedControllerDebug.pidDebug.iPart = data->iPart;
	    speedControllerDebug.pidDebug.pPart = data->pPart;
	    speedControllerDebug.pidDebug.minMaxPidOutput = data->minMaxPidOutput;
	    break;
	}	
	case firmware::PACKET_ID_SPEED_DEBUG: {
	    const firmware::speedDebugData *data =
		reinterpret_cast<const firmware::speedDebugData *>(msg.data);
		
		//BUG fix encoder value
	    speedControllerDebug.encoderValue = data->encoderVal;
	    speedControllerDebug.pwmValue = data->pwmVal;
	    speedControllerDebug.speedValue = data->speedVal;
	    speedControllerDebug.targetValue = data->targetVal;
	    break;
	}
    }
}

void SpeedPIDController::printSendError(const canbus::Message& msg)
{
    switch(msg.can_id)
    {
	case firmware::PACKET_ID_SET_PID_SPEED:
	    std::cout << "SpeedPIDController:: SetPid message was not acked" << std::endl;
	    break;
    }
}

std::vector< int > SpeedPIDController::getSendCanIds()
{
    std::vector<int> ids;
    ids.push_back(firmware::PACKET_ID_SET_PID_SPEED);
    return ids;
}


void SpeedPIDController::sendControllerConfig()
{
    canbus::Message msg;

    msg.can_id = firmware::PACKET_ID_SET_PID_SPEED;
    
    //check if values exceed signed short
    if(config.pidValues.kp * 100 > (1<<16) || config.pidValues.kp * 100 < -(1<<16) ||
	config.pidValues.ki * 100 > (1<<16) || config.pidValues.ki * 100 < -(1<<16) ||
	config.pidValues.kd * 100 > (1<<16) || config.pidValues.kd * 100 < -(1<<16))
	throw std::runtime_error("Given PID Parameters are out of bound");
    
    firmware::setPidData *data = reinterpret_cast<firmware::setPidData *>(msg.data);
    //convert given parameters to fixed point values for transmission
    data->kp = config.pidValues.kp * 100;
    data->ki = config.pidValues.ki * 100;
    data->kd = config.pidValues.kd * 100;
    data->minMaxPidOutput = config.pidValues.maxPWM;
    
    msg.size = sizeof(firmware::setPidData);
    
    sendCanMsg(msg, true);
}

Controller* SpeedPIDController::getCopy() const
{
    return new SpeedPIDController();
}



PosPIDController::PosPIDController()
{
}

std::vector< int > PosPIDController::getAcceptedCanIds()
{
    std::vector<int> ids;
    ids.push_back(firmware::PACKET_ID_POS_DEBUG);
    ids.push_back(firmware::PACKET_ID_PID_DEBUG_POS);
    return ids;
}


void PosPIDController::processMsg(const canbus::Message& msg)
{
    switch(msg.can_id)
    {
	case firmware::PACKET_ID_POS_DEBUG:
	{
	    const firmware::posDebugData * data =
		reinterpret_cast<const firmware::posDebugData *> (msg.data);

		//BUG fix encoder value
	    positionControllerDebug.encoderValue = data->encoderVal;
	    positionControllerDebug.pwmValue = data->pwmVal;
	    positionControllerDebug.positionValue = data->posVal;
	    positionControllerDebug.targetValue = data->targetVal;
	    break;
	}

	case firmware::PACKET_ID_PID_DEBUG_POS:
	{
	    const firmware::pidDebugData * data =
		reinterpret_cast<const firmware::pidDebugData *> (msg.data);

	    positionControllerDebug.pidDebug.dPart = data->dPart;
	    positionControllerDebug.pidDebug.iPart = data->iPart;
	    positionControllerDebug.pidDebug.pPart = data->pPart;
	    positionControllerDebug.pidDebug.minMaxPidOutput = data->minMaxPidOutput;
	    break;
	}
	default:
	    break;
    }
}

std::vector< int > PosPIDController::getSendCanIds()
{
    std::vector<int> ids;
    ids.push_back(firmware::PACKET_ID_SET_PID_POS);
    ids.push_back(firmware::PACKET_ID_POS_CONTROLLER_DATA);
    return ids;
}

void PosPIDController::printSendError(const canbus::Message& msg)
{
    switch(msg.can_id)
    {
	case firmware::PACKET_ID_SET_PID_POS:
	    std::cout << "PosPIDController:: SetPid message was not acked" << std::endl;
	    break;
	case firmware::PACKET_ID_POS_CONTROLLER_DATA:
	    std::cout << "PosPIDController:: Set controller data message was not acked" << std::endl;
	    break;
    }
}

void PosPIDController::sendControllerConfig()
{
    canbus::Message ret;
    ret.can_id = firmware::PACKET_ID_POS_CONTROLLER_DATA;
    ret.size = sizeof(firmware::posControllerData);
    firmware::posControllerData *data = (firmware::posControllerData *) ret.data;
    data->hysteresisActive = config.posCtrlConfig.hysteresisActive;
    data->allowWrapAround = config.posCtrlConfig.allowWrapAround;
    data->minHystDist = config.posCtrlConfig.maxHystDist;
    data->maxHystDist = config.posCtrlConfig.maxHystDist;
    data->overDistCount = config.posCtrlConfig.overDistCount;
    data->unused = 0;
    
    sendCanMsg(ret, true);
    
    canbus::Message msg;
    msg.can_id = firmware::PACKET_ID_SET_PID_POS;

    //check if values exceed signed short
    if(config.pidValues.kp * 100 > (1<<16) || config.pidValues.kp * 100 < -(1<<16) ||
	config.pidValues.ki * 100 > (1<<16) || config.pidValues.ki * 100 < -(1<<16) ||
	config.pidValues.kd * 100 > (1<<16) || config.pidValues.kd * 100 < -(1<<16))
	throw std::runtime_error("Given PID Parameters are out of bound");
    
    firmware::setPidData *data2 = reinterpret_cast<firmware::setPidData *>(msg.data);
    //convert given parameters to fixed point values for transmission
    data2->kp = config.pidValues.kp * 100;
    data2->ki = config.pidValues.ki * 100;
    data2->kd = config.pidValues.kd * 100;
    data2->minMaxPidOutput = config.pidValues.maxPWM;
    
    msg.size = sizeof(firmware::setPidData);
    
    sendCanMsg(msg, true);
}


short unsigned int PosPIDController::getTargetValue(double value)
{
    //BUG does not work
    //fails in case of uneven number of ticks per turn
    
    //value is in radian
    //we scale it to 2 to the power of 16
    return (value / M_PI) * (1<<16) - 1;
}

Controller* PosPIDController::getCopy() const
{
    return new PosPIDController();
}

short unsigned int PWMController::getTargetValue(double value)
{
    return value * 1800;
}

Controller* PWMController::getCopy() const
{
    return new PWMController;
}


}