#include "Writer.hpp"
#include "Reader.hpp"
#include "../protocol.hpp"
#include "Protocol.hpp"
#include "Controller.hpp"

using namespace firmware;

namespace hbridge
{

class WriterState 
{
public:
    enum STATES firmwareState;
};
    
Writer::Writer(hbridge::HbridgeHandle* handle): state(new WriterState), curController(0), handle(handle), callbacks(0)
{

}

void Writer::sendActuatorConfig()
{
    Packet msg;
    msg.packetId = firmware::PACKET_ID_SET_ACTUATOR_CONFIG;
    msg.data.resize(sizeof(firmware::actuatorConfig));
    
    firmware::actuatorConfig *cfg = reinterpret_cast<firmware::actuatorConfig *>(msg.data.data());
    
    cfg->openCircuit = actuatorConfig.openCircuit;
    switch(actuatorConfig.controllerInputEncoder)
    {
	case hbridge::INTERNAL:
	    cfg->controllerInputEncoder = firmware::INTERNAL;
	    break;
	case hbridge::EXTERNAL:
	    cfg->controllerInputEncoder = firmware::EXTERNAL;
	    break;
    }
    cfg->maxMotorTemp = actuatorConfig.maxMotorTemp;
    cfg->maxMotorTempCount= actuatorConfig.maxMotorTempCount;
    cfg->maxBoardTemp = actuatorConfig.maxBoardTemp;
    cfg->maxBoardTempCount = actuatorConfig.maxBoardTempCount;
    cfg->timeout = actuatorConfig.timeout;
    cfg->maxCurrent = actuatorConfig.maxCurrent;
    cfg->maxCurrentCount = actuatorConfig.maxCurrentCount;
    cfg->pwmStepPerMs = actuatorConfig.pwmStepPerMs;
    
    handle->getProtocol()->sendPacket(handle->getBoardId(), msg, true, boost::bind(&Writer::configurationError, this, _1), boost::bind(&Writer::sendController, this));
}

void Writer::sendController()
{
    std::cout << "Sending controller " << std::endl; 
    Packet msg;
    msg.packetId = PACKET_ID_SET_ACTIVE_CONTROLLER;
    msg.data.resize(sizeof(setActiveControllerData));
    
    setActiveControllerData *cfg = reinterpret_cast<setActiveControllerData *>(msg.data.data());

    handle->getProtocol()->sendPacket(handle->getBoardId(), msg, true, boost::bind(&Writer::configurationError, this, _1));

    cfg->controllerId = curController->getControllerId();
}

void Writer::startConfigure()
{
    sendActuatorConfig();
}



bool Writer::isActuatorConfigured()
{
    return state->firmwareState >= STATE_CONTROLLER_CONFIGURED;
}

bool Writer::hasError()
{
    return state->firmwareState == STATE_ACTUATOR_ERROR || state->firmwareState == STATE_SENSOR_ERROR;
}

void Writer::configurationError(const hbridge::Packet& msg)
{
    std::cout << "Writer: Error, actuator config was not acked" << std::endl;
    if(callbacks)
	callbacks->configurationError();
}

void Writer::processMsg(const Packet& msg)
{
    switch(msg.packetId)
    {
	case PACKET_ID_ANNOUNCE_STATE:
	{
	    const announceStateData *stateData = 
		reinterpret_cast<const announceStateData *>(msg.data.data());
	    
	    state->firmwareState = stateData->curState;
		
	    switch(stateData->curState)
	    {
		case STATE_UNCONFIGURED:
		    break;
		case STATE_SENSORS_CONFIGURED:
		    break;    
		case STATE_SENSOR_ERROR:
		    break;    
		case STATE_ACTUATOR_CONFIGURED:
		    break;
		case STATE_CONTROLLER_CONFIGURED:
		    if(callbacks)
			callbacks->configureDone();
		    break;
		case STATE_ACTUATOR_ERROR:
		    if(callbacks)
			callbacks->actuatorError();
		    break;
		default:
		    break;
	    }
	    break;
	}
    }
}


void Writer::setActiveController(Controller* ctrl)
{
    if(state->firmwareState >= STATE_ACTUATOR_CONFIGURED)
	throw std::runtime_error("Error: Tried to set Controller while actuator is active");

    if(handle->getControllers()[ctrl->getControllerId()] != ctrl)
	throw std::runtime_error("Error: given controller is not registered at handle");
    
    curController = ctrl;    
}

void Writer::setTargetValue(double value)
{
    if(state->firmwareState < STATE_CONTROLLER_CONFIGURED)
	throw std::runtime_error("Writer : Error: Tried to write on unconfigured hbridge");

    if(!curController)
	throw std::runtime_error("Writer : Error: No Controller selected");
    
    int packetId = firmware::PACKET_ID_SET_VALUE14;
    if(handle->getBoardId() > 3)
	packetId = firmware::PACKET_ID_SET_VALUE58;

    Packet &sharedMsg(handle->getProtocol()->getSharedMsg(packetId));
    sharedMsg.data.resize(sizeof(firmware::setValueData));
    
    firmware::setValueData *data =
	reinterpret_cast<firmware::setValueData *>(sharedMsg.data.data());
    
    unsigned short transportValue = curController->getTargetValue(value);
    
    switch(handle->getBoardId())
    {
	case 0:
	case 4:
	    data->board1Value = transportValue;
	    break;
	case 1:
	case 5:
	    data->board2Value = transportValue;
	    break;
	case 2:
	case 6:
	    data->board3Value = transportValue;
	    break;
	case 3:
	case 7:
	    data->board4Value = transportValue;
	    break;
    };
}

    
    
    
    
}