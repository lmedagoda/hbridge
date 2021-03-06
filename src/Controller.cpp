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

bool Controller::isInverted()
{
    return writer->actuatorConfig.inverted;
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
		data->board1Value = *((int16_t *) commandData.data());
		break;
	    case 1:
	    case 5:
		data->board2Value = *((int16_t *) commandData.data());
		break;
	    case 2:
	    case 6:
		data->board3Value = *((int16_t *) commandData.data());
		break;
	    case 3:
	    case 7:
		data->board4Value = *((int16_t *) commandData.data());
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

void SpeedPIDController::setTargetValue(double radPerSecond)
{
    double turnPerSecond = radPerSecond / (2*M_PI);

    if(isInverted())
        turnPerSecond *= -1;
    
    //output is in turns/second * 100
    int32_t tpsTimes100 = turnPerSecond * 100;

    if((tpsTimes100 < std::numeric_limits<int16_t>::min()) || (tpsTimes100 > std::numeric_limits<int16_t>::max()))
        throw std::runtime_error("SpeedPIDController:Error, got target value which is out of range (-1000 +1000 radian/second)");

    int16_t transmitValue = tpsTimes100; // std::min<int32_t>(std::max<int32_t>(tpsTimes100, std::numeric_limits<int16_t>::min()), std::numeric_limits<int16_t>::max());

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
		
            speedControllerDebug.boardId = msg.receiverId;
	    speedControllerDebug.encoderValue = data->encoderVal;
	    speedControllerDebug.pwmValue = data->pwmVal;
	    speedControllerDebug.speedValue = data->speedVal;
	    speedControllerDebug.targetValue = data->targetVal;
	    speedControllerDebug.pidDebug.dPart = data->pidData.dPart;
	    speedControllerDebug.pidDebug.iPart = data->pidData.iPart;
	    speedControllerDebug.pidDebug.pPart = data->pidData.pPart;
	    speedControllerDebug.pidDebug.minMaxPidOutput = data->pidData.minMaxPidOutput;
            
            if(debugCallback)
                debugCallback(speedControllerDebug);
	    break;
	}
    }
}

void SpeedPIDController::registerDebugCallback(boost::function<void (const Debug &debugData)> callback)
{
    debugCallback = callback;
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

void SpeedPIDController::setConfig(const SpeedPIDController::Config& config)
{
    this->config = config;
}

void SpeedPIDController::sendControllerConfig()
{
    Packet msg;

    msg.packetId = firmware::PACKET_ID_SET_SPEED_CONTROLLER_DATA;

    const int16_t max = std::numeric_limits<int16_t>::max();
    const int16_t min = std::numeric_limits<int16_t>::min();

    //check if values exceed signed short
    if(config.pidValues.kp * 10 > max || config.pidValues.kp * 10 < min ||
       config.pidValues.ki * 10 > max || config.pidValues.ki * 10 < min ||
       config.pidValues.kd * 10 > max || config.pidValues.kd * 10 < min)
	throw std::runtime_error("Given PID Parameters are out of bound");

    if(config.pidValues.maxPWM < 0 || config.pidValues.maxPWM > 1.0)
	throw std::runtime_error("Given PID MAXPwm Parameter is out of bound [0 1]");

    msg.data.resize(sizeof(firmware::speedControllerData));
    
    firmware::speedControllerData *data = reinterpret_cast<firmware::speedControllerData *>(msg.data.data());
    //convert given parameters to fixed point values for transmission
    data->pidData.kp = config.pidValues.kp * 10;
    data->pidData.ki = config.pidValues.ki * 10;
    data->pidData.kd = config.pidValues.kd * 10;
    data->pidData.minMaxPidOutput = config.pidValues.maxPWM * std::numeric_limits<int16_t>::max();
    data->debugActive = config.debugActive;

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

void PosPIDController::setConfig(const PosPIDController::Config& config)
{
    this->config = config;
}

void PosPIDController::sendControllerConfig()
{
    Packet msg;
    msg.packetId = firmware::PACKET_ID_SET_POS_CONTROLLER_DATA;
    msg.data.resize(sizeof(firmware::posControllerData));
    firmware::posControllerData *data = (firmware::posControllerData *) msg.data.data();
    data->hysteresisActive = config.hysteresisActive;
    data->allowWrapAround = config.allowWrapAround;
    data->minHystDist = config.maxHystDist;
    data->maxHystDist = config.maxHystDist;
    data->overDistCount = config.overDistCount;
    data->unused = 0;
    
    const int16_t max = std::numeric_limits<int16_t>::max();
    const int16_t min = std::numeric_limits<int16_t>::min();

    //check if values exceed signed short
    if(config.pidValues.kp * 10 > max || config.pidValues.kp * 10 < min ||
       config.pidValues.ki * 10 > max || config.pidValues.ki * 10 < min ||
       config.pidValues.kd * 10 > max || config.pidValues.kd * 10 < min)
	throw std::runtime_error("Given PID Parameters are out of bound");
    
    if(config.pidValues.maxPWM < 0 || config.pidValues.maxPWM > 1.0)
	throw std::runtime_error("Given PID MAXPwm Parameter is out of bound [0 1]");
	
    //convert given parameters to fixed point values for transmission
    data->pidData.kp = config.pidValues.kp * 10;
    data->pidData.ki = config.pidValues.ki * 10;
    data->pidData.kd = config.pidValues.kd * 10;
    data->pidData.minMaxPidOutput = config.pidValues.maxPWM * std::numeric_limits<int16_t>::max();
    
    sendPacket(msg, true);
}

void PosPIDController::setTargetValue(double value)
{
    if((value < -M_PI) || (value > M_PI))
	throw std::runtime_error("PosPIDController::Error, got target value which is out of range -M_PI +M_PI");
    
    if(isInverted())
        value *= -1;

    //value is in radian
    //we scale it to int16_t
    uint16_t transmitValue = ((value +M_PI) / (2*M_PI)) * std::numeric_limits< uint16_t >::max();
    sendCommand(transmitValue);
}

PWMController::PWMController(Writer *writer): Controller(writer, firmware::CONTROLLER_MODE_PWM)
{

}

void PWMController::setTargetValue(double value)
{
    if((value < -1.0) || (value > 1.0))
	throw std::runtime_error("PWMController::Error, got target value which is out of range -1 +1");

    if(isInverted())
        value *= -1;

    uint16_t transmitValue = value * std::numeric_limits< int16_t >::max();
    sendCommand(transmitValue);
}

}
