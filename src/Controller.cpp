#include "Controller.hpp"
#include "Protocol.hpp"
#include <boost/bind.hpp>
#include "Protocol.hpp"
#include <limits>
#include "Writer.hpp"

namespace hbridge 
{

Controller::Controller(Writer *writer, firmware::controllerModes controllerId):mode(controllerId), writer(writer)
{
    writer->registerController(this);
}

void Controller::sendPacket(const hbridge::Packet& msg, bool isAcked)
{
    writer->protocol->sendPacket(writer->boardId, msg, isAcked, boost::bind(&Controller::packetSendError, this, _1));
}

void Controller::packetSendError(const hbridge::Packet& msg)
{
    printSendError(msg.packetId);
    writer->callbacks->configurationError();
}

void Controller::sendCommandData()
{
    if(!hasCommand)
	return;
    
    if(commandData.size() == 2)
    {
	int packetId = firmware::PACKET_ID_SET_VALUE14;
	if(writer->boardId > 3)
	    packetId = firmware::PACKET_ID_SET_VALUE58;

	Packet &sharedMsg(writer->protocol->getSharedMsg(packetId));
	sharedMsg.data.resize(sizeof(firmware::setValueData));
	
	firmware::setValueData *data =
	    reinterpret_cast<firmware::setValueData *>(sharedMsg.data.data());
	
	
	switch(writer->boardId)
	{
	    case 0:
	    case 4:
		data->board1Value = *((uint16_t *) commandData.data());
		break;
	    case 1:
	    case 5:
		data->board2Value = *((uint16_t *) commandData.data());
		break;
	    case 2:
	    case 6:
		data->board3Value = *((uint16_t *) commandData.data());
		break;
	    case 3:
	    case 7:
		data->board4Value = *((uint16_t *) commandData.data());
		break;
	};
	
    }
    else
    {
	Packet msg;
	msg.packetId = firmware::PACKET_ID_SET_VALUE;
	msg.broadcastMsg = false;
	msg.data = commandData;
	sendPacket(msg, false);
    }
}

void Controller::registerForPacketId(int canId)
{
    writer->registerForMsg(this, canId);
}

SpeedPIDController::SpeedPIDController(Writer *writer):Controller(writer, firmware::CONTROLLER_MODE_SPEED)
{
    registerForPacketId(firmware::PACKET_ID_SPEED_CONTROLLER_DEBUG);
}

void SpeedPIDController::setTargetValue(double value)
{
    if((value < -1.0) || (value > 1.0))
	throw std::runtime_error("SpeedPIDController:Error, got target value which is out of range -1 +1");
    
    uint16_t transmitValue = value * std::numeric_limits<uint16_t>::max();
    sendCommand(transmitValue);
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

PosPIDController::PosPIDController(Writer *writer):Controller(writer, firmware::CONTROLLER_MODE_POSITION)
{
    registerForPacketId(firmware::PACKET_ID_POS_CONTROLLER_DEBUG);
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

void PosPIDController::setTargetValue(double value)
{
    if((value < -M_PI) || (value > M_PI))
	throw std::runtime_error("PosPIDController::Error, got target value which is out of range -M_PI +M_PI");
    
    //value is in radian
    //we scale it to int16_t
    uint16_t transmitValue = (value / M_PI) * std::numeric_limits< int16_t >::max();
    sendCommand(transmitValue);
}

PWMController::PWMController(Writer *writer): Controller(writer, firmware::CONTROLLER_MODE_PWM)
{

}

void PWMController::setTargetValue(double value)
{
    if((value < -1.0) || (value > 1.0))
	throw std::runtime_error("PWMController::Error, got target value which is out of range -1 +1");

    uint16_t transmitValue = value * std::numeric_limits< int16_t >::max();
    sendCommand(transmitValue);
}

}
