#include "Controller.hpp"
#include "Protocol.hpp"
#include <boost/bind.hpp>
#include "../protocol.hpp"
#include <limits>
#include "Writer.hpp"

namespace hbridge 
{

Controller::Controller(HbridgeHandle* handle, firmware::controllerModes controllerId):mode(controllerId), handle(handle)
{
    handle->registerController(this);
}

void Controller::sendPacket(const hbridge::Packet& msg, bool isAcked)
{
    handle->getProtocol()->sendPacket(handle->getBoardId(), msg, isAcked, boost::bind(&Controller::packetSendError, this, _1));
}

void Controller::packetSendError(const hbridge::Packet& msg)
{
    printSendError(msg.packetId);
    handle->getWriter()->callbacks->configurationError();
}


void Controller::registerForCanId(int canId)
{
    handle->registerForMsg(this, canId);
}


unsigned short Controller::getTargetValue(double value)
{
    return 0;
}

void Controller::setTargetValue(double value)
{

}

SpeedPIDController::SpeedPIDController(HbridgeHandle* handle):Controller(handle, firmware::CONTROLLER_MODE_SPEED)
{
    registerForCanId(firmware::PACKET_ID_SPEED_CONTROLLER_DEBUG);
}

unsigned short SpeedPIDController::getTargetValue(double value)
{
    return value * std::numeric_limits<uint16_t>::max();
}

void SpeedPIDController::processMsg(const hbridge::Packet& msg)
{
    switch(msg.packetId)
    {
	case firmware::PACKET_ID_SPEED_CONTROLLER_DEBUG:
	{
	    const firmware::speedDebugData *data =
		reinterpret_cast<const firmware::speedDebugData *>(msg.data.data());
		
		//BUG fix encoder value
	    speedControllerDebug.encoderValue = data->encoderVal;
	    speedControllerDebug.pwmValue = data->pwmVal;
	    speedControllerDebug.speedValue = data->speedVal;
	    speedControllerDebug.targetValue = data->targetVal;
	    speedControllerDebug.pidDebug.dPart = data->pidData.dPart;
	    speedControllerDebug.pidDebug.iPart = data->pidData.iPart;
	    speedControllerDebug.pidDebug.pPart = data->pidData.pPart;
	    speedControllerDebug.pidDebug.minMaxPidOutput = data->pidData.minMaxPidOutput;
	    break;
	}
    }
}

void SpeedPIDController::printSendError(const hbridge::Packet& msg)
{
    switch(msg.packetId)
    {
	case firmware::PACKET_ID_SET_SPEED_CONTROLLER_DATA:
	    std::cout << "SpeedPIDController:: SetPid message was not acked" << std::endl;
	    break;
    }
}

void SpeedPIDController::sendControllerConfig()
{
    Packet msg;

    msg.packetId = firmware::PACKET_ID_SET_SPEED_CONTROLLER_DATA;
    
    //check if values exceed signed short
    if(config.pidValues.kp * 100 > (1<<16) || config.pidValues.kp * 100 < -(1<<16) ||
	config.pidValues.ki * 100 > (1<<16) || config.pidValues.ki * 100 < -(1<<16) ||
	config.pidValues.kd * 100 > (1<<16) || config.pidValues.kd * 100 < -(1<<16))
	throw std::runtime_error("Given PID Parameters are out of bound");

    msg.data.resize(sizeof(firmware::setPidData));
    
    firmware::setPidData *data = reinterpret_cast<firmware::setPidData *>(msg.data.data());
    //convert given parameters to fixed point values for transmission
    data->kp = config.pidValues.kp * 100;
    data->ki = config.pidValues.ki * 100;
    data->kd = config.pidValues.kd * 100;
    data->minMaxPidOutput = config.pidValues.maxPWM;
    
    sendPacket(msg, true);
}

PosPIDController::PosPIDController(HbridgeHandle* handle):Controller(handle, firmware::CONTROLLER_MODE_POSITION)
{
    registerForCanId(firmware::PACKET_ID_POS_CONTROLLER_DEBUG);
}

void PosPIDController::processMsg(const hbridge::Packet& msg)
{
    switch(msg.packetId)
    {
	case firmware::PACKET_ID_POS_CONTROLLER_DEBUG:
	{
	    const firmware::posDebugData * data =
		reinterpret_cast<const firmware::posDebugData *> (msg.data.data());

		//BUG fix encoder value
	    positionControllerDebug.encoderValue = data->encoderVal;
	    positionControllerDebug.pwmValue = data->pwmVal;
	    positionControllerDebug.positionValue = data->posVal;
	    positionControllerDebug.targetValue = data->targetVal;
	    positionControllerDebug.pidDebug.dPart = data->pidData.dPart;
	    positionControllerDebug.pidDebug.iPart = data->pidData.iPart;
	    positionControllerDebug.pidDebug.pPart = data->pidData.pPart;
	    positionControllerDebug.pidDebug.minMaxPidOutput = data->pidData.minMaxPidOutput;
	    break;
	}
	default:
	    break;
    }
}

void PosPIDController::printSendError(const hbridge::Packet& msg)
{
    switch(msg.packetId)
    {
	case firmware::PACKET_ID_SET_POS_CONTROLLER_DATA:
	    std::cout << "PosPIDController:: Set controller data message was not acked" << std::endl;
	    break;
    }
}

void PosPIDController::sendControllerConfig()
{
    Packet msg;
    msg.packetId = firmware::PACKET_ID_SET_POS_CONTROLLER_DATA;
    msg.data.resize(sizeof(firmware::posControllerData));
    firmware::posControllerData *data = (firmware::posControllerData *) msg.data.data();
    data->hysteresisActive = config.posCtrlConfig.hysteresisActive;
    data->allowWrapAround = config.posCtrlConfig.allowWrapAround;
    data->minHystDist = config.posCtrlConfig.maxHystDist;
    data->maxHystDist = config.posCtrlConfig.maxHystDist;
    data->overDistCount = config.posCtrlConfig.overDistCount;
    data->unused = 0;
    
    //check if values exceed signed short
    if(config.pidValues.kp * 100 > (1<<16) || config.pidValues.kp * 100 < -(1<<16) ||
	config.pidValues.ki * 100 > (1<<16) || config.pidValues.ki * 100 < -(1<<16) ||
	config.pidValues.kd * 100 > (1<<16) || config.pidValues.kd * 100 < -(1<<16))
	throw std::runtime_error("Given PID Parameters are out of bound");
    //convert given parameters to fixed point values for transmission
    data->pidData.kp = config.pidValues.kp * 100;
    data->pidData.ki = config.pidValues.ki * 100;
    data->pidData.kd = config.pidValues.kd * 100;
    data->pidData.minMaxPidOutput = config.pidValues.maxPWM;
    
    sendPacket(msg, true);
}


short unsigned int PosPIDController::getTargetValue(double value)
{
    //BUG does not work
    //fails in case of uneven number of ticks per turn
    
    //value is in radian
    //we scale it to 2 to the power of 16
    return (value / M_PI) * (1<<16) - 1;
}

PWMController::PWMController(HbridgeHandle* handle): Controller(handle, firmware::CONTROLLER_MODE_PWM)
{

}


short unsigned int PWMController::getTargetValue(double value)
{
    return value * 1800;
}



}