#include "Writer.hpp"
#include "Reader.hpp"
#include "Protocol.hpp"
#include "Controller.hpp"
#include <boost/lexical_cast.hpp>

using namespace firmware;

namespace hbridge
{

enum DriverState
{
    WRITER_NOT_ACTIVE,
    WRITER_STATE_REQUEST,
    WRITER_CONFIGURING,
    WRITER_CONTORLLERS_CONFIGURING,
    WRITER_CONTORLLERS_CONFIGURED,
};
    
class WriterState 
{
public:
    WriterState() : firmwareState(STATE_UNCONFIGURED) , driverState(WRITER_NOT_ACTIVE)
    {}
    enum STATES firmwareState;
    enum DriverState driverState;
};
    
Writer::Writer(uint32_t boardId, hbridge::Protocol* protocol): state(new WriterState), curController(0), driverError(false), boardId(boardId), protocol(protocol), callbacks(0)
{
    protocol->registerReceiver(this, boardId);
    controllers.resize(firmware::NUM_CONTROLLERS, NULL);
    msgHandlers.resize(PACKET_ID_TOTAL_COUNT, NULL);
}

Writer::~Writer()
{
    for(std::vector<Controller *>::iterator it = controllers.begin(); it != controllers.end(); it++)
    {
	if(*it)
	    delete *it;
    }
    controllers.clear();
}


void Writer::setConfiguration(const hbridge::ActuatorConfiguration& actuatorConfig)
{
    this->actuatorConfig = actuatorConfig;
}


void Writer::requestDeviceState()
{
    Packet msg;
    msg.packetId = PACKET_ID_REQUEST_STATE;
    
    protocol->sendPacket(boardId, msg, true, boost::bind(&Writer::configurationError, this, _1));
}

void Writer::resetActuator()
{
    Packet msg;
    msg.packetId = PACKET_ID_CLEAR_ACTUATOR_ERROR;
    
    protocol->sendPacket(boardId, msg, true, boost::bind(&Writer::configurationError, this, _1));
}

void Writer::sendActuatorConfig()
{
    Packet msg;
    msg.packetId = firmware::PACKET_ID_SET_ACTUATOR_CONFIG;
    msg.data.resize(sizeof(firmware::actuatorConfig));
    
    std::cout << "HB " << boardId << " sendinf ACC configuration " << std::endl;

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
    cfg->maxMotorTempCount= 200;
    cfg->maxBoardTemp = actuatorConfig.maxBoardTemp;
    cfg->maxBoardTempCount = 200;
    cfg->timeout = actuatorConfig.timeout;
    cfg->maxCurrent = actuatorConfig.maxCurrent;
    cfg->maxCurrentCount = actuatorConfig.maxCurrentCount;
    cfg->pwmStepPerMs = actuatorConfig.pwmStepPerMs;
    
    protocol->sendPacket(boardId, msg, true, boost::bind(&Writer::configurationError, this, _1));
}

void Writer::sendControllerConfig()
{
    for(std::vector<Controller *>::iterator it = controllers.begin(); it != controllers.end(); it++)
    {
	if(*it)
	    (*it)->sendControllerConfig();
    }
    
    protocol->setQueueEmptyCallback(boost::bind(&Writer::controllersConfiguredCallback, this), boardId);
}

void Writer::controllersConfiguredCallback()
{
    state->driverState = WRITER_CONTORLLERS_CONFIGURED;
    //remove callback
    protocol->setQueueEmptyCallback(boost::function<void (void)>(), boardId);
}

void Writer::startConfigure()
{
    driverError = false;
    state->driverState = WRITER_STATE_REQUEST;
    std::cout << "HB " << boardId << " Starting configuration " << std::endl;
    requestDeviceState();    
}

bool Writer::isActuatorConfigured()
{
    return (state->firmwareState >= STATE_ACTUATOR_CONFIGURED) && (state->driverState == WRITER_CONTORLLERS_CONFIGURED);
}

bool Writer::hasError()
{
    return driverError || 
    (state->driverState == WRITER_CONTORLLERS_CONFIGURED && 
     (state->firmwareState == STATE_ACTUATOR_ERROR || state->firmwareState == STATE_SENSOR_ERROR));
}

void Writer::configurationError(const hbridge::Packet& msg)
{
    std::cout << "HB " << boardId << " Writer: Error, actuator config was not acked" << std::endl;
    if(callbacks)
	callbacks->configurationError();
}

void Writer::processStateAnnounce(const hbridge::Packet& msg)
{
    assert(msg.packetId == PACKET_ID_ANNOUNCE_STATE);
    
    const announceStateData *stateData = 
	reinterpret_cast<const announceStateData *>(msg.data.data());
    
    state->firmwareState = stateData->curState;

    std::cout << "GOT STATE ACCOUNCE hb " << boardId << " new state " <<  getStateName(stateData->curState) <<  std::endl;

    switch(state->driverState)
    {
	case WRITER_NOT_ACTIVE:
	    return;
	case WRITER_STATE_REQUEST:
	    switch(stateData->curState)
	    {
		case STATE_UNCONFIGURED:
		case STATE_SENSOR_ERROR:
		    {
			std::cout << "Error, Sensors not configured hb " << boardId << std::endl;
			driverError = true;
			if(callbacks)
			    callbacks->configurationError();
		    }
		    break;    
		case STATE_SENSORS_CONFIGURED:
		    std::cout << "WRITING ACC CONFIG hb " << boardId << std::endl;
		    sendActuatorConfig();
		    state->driverState = WRITER_CONFIGURING;
		    break;
		case STATE_ACTUATOR_CONFIGURED:
		case STATE_CONTROLLER_CONFIGURED:
		case STATE_RUNNING:
		case STATE_ACTUATOR_ERROR:
		    std::cout << "HB " << boardId << " is still configured, removing old configuration" << std::endl;
		    resetActuator();
		    break;
	    }
	    break;
	    
	case WRITER_CONFIGURING:
	    switch(stateData->curState)
	    {
		case STATE_ACTUATOR_CONFIGURED:
		    std::cout << "HB " << boardId << " is reconfigured, sendinf controller config" << std::endl;
		    sendControllerConfig();
		    break;
		case STATE_SENSORS_CONFIGURED:
		    std::cout << "HB " <<  boardId << " Warning, got reanounce of state SENSORS_CONFIGURED, ignoring it" << std::endl;
		    break;
		case STATE_UNCONFIGURED:
		case STATE_SENSOR_ERROR:
		default:
		  std::cout << "HB " << boardId << " ERROR got wrong response" << std::endl;
		    driverError = true;
		    if(callbacks)
			callbacks->actuatorError();
		    break;
	    }
	    break;
	case WRITER_CONTORLLERS_CONFIGURING:
	    if(stateData->curState == STATE_CONTROLLER_CONFIGURED)
		state->driverState = WRITER_CONTORLLERS_CONFIGURED;
	    //NO break use default handling
	default:
	    switch(stateData->curState)
	    {
		case STATE_UNCONFIGURED:
		case STATE_SENSORS_CONFIGURED:
		case STATE_SENSOR_ERROR:
		    driverError = true;
		    if(callbacks)
			callbacks->actuatorError();
		case STATE_ACTUATOR_ERROR:
		    driverError = true;
		    if(callbacks)
			callbacks->actuatorError();
		    break;    
		default:
		    break;
	    }	    
	    break;
    }
}


void Writer::processMsg(const Packet& msg)
{
    if(msgHandlers[msg.packetId])
	msgHandlers[msg.packetId]->processMsg(msg);
    
    switch(msg.packetId)
    {
	case PACKET_ID_ANNOUNCE_STATE:
	{
	    processStateAnnounce(msg);
	    break;
	}
        default:
	    break;
    }
}

void Writer::registerController(Controller* ctrl)
{
    if(controllers[ctrl->getControllerId()])
      throw std::out_of_range("HbridgeHandle: Error: There is already a controller with id " + boost::lexical_cast<std::string>((int) ctrl->getControllerId()) + std::string(" registered"));
    
    controllers[ctrl->getControllerId()] = ctrl;
}

void Writer::registerForMsg(PacketReceiver* receiver, int packetId)
{
    if(packetId < 0 || packetId > PACKET_ID_TOTAL_COUNT)
	throw std::out_of_range("HbridgeHandle: Error tried to register receiver for invalid id " + packetId );
    
    msgHandlers[packetId] = receiver;
}

void Writer::setActiveController(base::JointState::MODE id)
{	
    firmware::controllerModes modeIntern;

    switch(id)
    {
        case base::JointState::RAW:
	    modeIntern = firmware::CONTROLLER_MODE_PWM;
	    break;
        case base::JointState::SPEED:
	    modeIntern = firmware::CONTROLLER_MODE_SPEED;
	    break;
        case base::JointState::POSITION:
	    modeIntern = firmware::CONTROLLER_MODE_POSITION;
	    break;
	default:
	    throw std::runtime_error("Given controller mode is not mapped to firmware controller");
	    break;
    }

    if(controllers.size() < modeIntern || !controllers[modeIntern])
	throw std::runtime_error("Error: no controller for drive mode registered");

    setActiveController(controllers[modeIntern]);
}

Controller* Writer::getActiveController()
{
    return curController;
}

void Writer::setActiveController(Controller* ctrl)
{
    if(state->firmwareState != STATE_ACTUATOR_CONFIGURED)
	throw std::runtime_error("Error: Tried to set Controller while actuator was not configured");

    if(controllers[ctrl->getControllerId()] != ctrl)
	throw std::runtime_error("Error: given controller is not registered at handle");
    
    curController = ctrl;
    
    std::cout << "Sending controller " << std::endl; 
    Packet msg;
    msg.packetId = PACKET_ID_SET_ACTIVE_CONTROLLER;
    msg.data.resize(sizeof(setActiveControllerData));
    
    setActiveControllerData *cfg = reinterpret_cast<setActiveControllerData *>(msg.data.data());

    cfg->controllerId = curController->getControllerId();

    protocol->sendPacket(boardId, msg, true, boost::bind(&Writer::configurationError, this, _1));
    
    state->driverState = WRITER_CONTORLLERS_CONFIGURING;
}

bool Writer::isControllerConfiguring()
{
    return state->driverState == WRITER_CONTORLLERS_CONFIGURING;
}

bool Writer::isControllerSet()
{
    return !driverError && (
	((state->firmwareState == STATE_CONTROLLER_CONFIGURED) && (state->driverState == WRITER_CONTORLLERS_CONFIGURED))
	|| (state->firmwareState == STATE_RUNNING));
}
    
    
}
