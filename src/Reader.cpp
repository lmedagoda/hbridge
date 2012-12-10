#include "Reader.hpp"
#include "Controller.hpp"
#include "../protocol.hpp"
#include "Protocol.hpp"

using namespace firmware;

namespace hbridge {

Reader::Reader(HbridgeHandle *handle): handle(handle), callbacks(NULL), configured(false)
{
    packetHandlers.resize(firmware::PACKET_ID_TOTAL_COUNT);
}

void Reader::setCallbacks(Reader::CallbackInterface* cbs)
{
    callbacks = cbs;
}

void Reader::startConfigure()
{
//     //configure all controllers
//     for(std::vector<Controller *>::iterator it = handle->getControllers().begin(); 
// 	it != handle->getControllers().end(); it++)
//     {
// 	if(*it)
// 	    (*it)->sendControllerConfig();
//     }
    
    sendConfigureMsg();
}

void Reader::resetDevice()
{
    Packet msg;
    msg.packetId = PACKET_ID_CLEAR_SENSOR_ERROR;
    
    handle->getProtocol()->sendPacket(handle->getBoardId(), msg, true, boost::bind(&Reader::configurationError, this, _1));
}

void Reader::requestDeviceState()
{
    Packet msg;
    msg.packetId = PACKET_ID_REQUEST_STATE;
    
    handle->getProtocol()->sendPacket(handle->getBoardId(), msg, true, boost::bind(&Reader::configurationError, this, _1));
}


void Reader::setConfiguration(const hbridge::MotorConfiguration& config)
{
    configuration = config;

    //calcualte correct tick divider for 16 bit width
    configuration.encoder_config_intern.tickDivider = configuration.encoder_config_intern.ticksPerTurn / (1<<16) +1;
    configuration.encoder_config_intern.validate();
    
    encoderIntern.setConfiguration(configuration.encoder_config_intern);
    encoderIntern.setZeroPosition(configuration.encoder_config_intern.zeroPosition);

    //calcualte correct tick divider for 12 bit width
    configuration.encoder_config_extern.tickDivider = configuration.encoder_config_extern.ticksPerTurn / (1<<12) +1;
    configuration.encoder_config_extern.validate();

    encoderExtern.setConfiguration(configuration.encoder_config_extern);
    encoderExtern.setZeroPosition(configuration.encoder_config_extern.zeroPosition);
}

void Reader::sendConfigureMsg()
{
    SensorConfiguration &cfg(configuration.sensorConfig);
    Packet msg;
    msg.packetId = firmware::PACKET_ID_SET_SENSOR_CONFIG;
    msg.data.resize(sizeof(firmware::sensorConfig));
    
    firmware::sensorConfig *cfg1 = reinterpret_cast<firmware::sensorConfig *>(msg.data.data());

    cfg1->externalTempSensor = cfg.externalTempSensor;
    cfg1->statusEveryMs = 1;
    cfg1->encoder1Config.encoderType = static_cast<firmware::encoderTypes>(configuration.encoder_config_intern.type);
    cfg1->encoder1Config.ticksPerTurn = configuration.encoder_config_intern.ticksPerTurnDivided;
    cfg1->encoder1Config.tickDivider = configuration.encoder_config_intern.tickDivider;
    cfg1->encoder2Config.encoderType = static_cast<firmware::encoderTypes>(configuration.encoder_config_extern.type);
    cfg1->encoder2Config.ticksPerTurn = configuration.encoder_config_extern.ticksPerTurnDivided;
    cfg1->encoder2Config.tickDivider = configuration.encoder_config_extern.tickDivider;

    handle->getProtocol()->sendPacket(handle->getBoardId(), msg, true, boost::bind(&Reader::configurationError, this, _1));
}

int Reader::getCurrentTickDivider() const
{
    int tickDivider = 1;
    switch(configuration.actuatorConfig.controllerInputEncoder)
    {
	case INTERNAL:
	    tickDivider = encoderIntern.getEncoderConfig().tickDivider;
	    break;
	case EXTERNAL:
	    tickDivider = encoderExtern.getEncoderConfig().tickDivider;
	    break;
    }
    return tickDivider;
}

void Reader::registerControllerForPacketId(Controller* ctrl, int packetId)
{
    int size = packetHandlers.size();
    if(packetId < 0 || packetId > size)
	throw std::runtime_error("Reader::registerControllerForPacketId: Error tried to register controller for base protocol id");
	
    packetHandlers[packetId] = ctrl;

}

void Reader::unregisterController(Controller* ctrl)
{
    for(std::vector<Controller *>::iterator it = packetHandlers.begin(); it != packetHandlers.end(); it++)
    {
	if(*it == ctrl)
	    *it = NULL;
    }
}

void Reader::processMsg(const Packet &msg)
{    
    if(msg.packetId < firmware::PACKET_ID_LOWIDS_START)
    {
	switch (msg.packetId)
	{
	    case firmware::PACKET_ID_ERROR: {
		const firmware::errorData *edata =
		    reinterpret_cast<const firmware::errorData *>(msg.data.data());
		
		state.index   = edata->index;
		//hbridge is off, no current is flowing
		state.current = 0;
		state.pwm = 0;
		state.error.badConfig = edata->badConfig;
		state.error.boardOverheated = edata->boardOverheated;
		state.error.encodersNotInitialized = edata->encodersNotInitalized;
		state.error.motorOverheated = edata->motorOverheated;
		state.error.overCurrent = edata->overCurrent;
		state.error.timeout = edata->timeout;
		state.temperature = edata->temperature;

		encoderIntern.setRawEncoderValue(edata->position);
		encoderExtern.setRawEncoderValue(edata->externalPosition);
		
		this->state.position = encoderIntern.getAbsoluteTurns();
		this->state.positionExtern = encoderExtern.getAbsoluteTurns();
		
		if(callbacks)
		    callbacks->gotErrorStatus(state.error);

	    }
	    break;
	    case firmware::PACKET_ID_STATUS:
	    {
		const firmware::statusData *data =
		    reinterpret_cast<const firmware::statusData *>(msg.data.data());

		this->state.index   = data->index;
		this->state.current = data->currentValue; // Current in [mA]
		this->state.pwm     = static_cast<float>(data->pwm) / 1800; // PWM in [-1; 1]
		encoderIntern.setRawEncoderValue(data->position);
		encoderExtern.setRawEncoderValue(data->externalPosition);

		this->state.position = encoderIntern.getAbsoluteTurns();
		this->state.positionExtern = encoderExtern.getAbsoluteTurns();

		//getting an status package is an implicit cleaner for all error states
		bzero(&(this->state.error), sizeof(struct ErrorState));
		
		if(callbacks)
		    callbacks->gotStatus(state);
		break;
	    }
	    case firmware::PACKET_ID_EXTENDED_STATUS:
	    {
		const firmware::extendedStatusData *esdata = 
		    reinterpret_cast<const firmware::extendedStatusData *>(msg.data.data());
		    
		this->state.temperature = esdata->temperature;
		this->state.motorTemperature = esdata->motorTemperature;
		if(callbacks)
		    callbacks->gotStatus(state);
		break;
	    }
	    case firmware::PACKET_ID_ANNOUNCE_STATE:
	    {
		const firmware::announceStateData *stateData = 
		    reinterpret_cast<const firmware::announceStateData *>(msg.data.data());
		
		    switch(stateData->curState)
		    {
			case firmware::STATE_UNCONFIGURED:
			    if(callbacks)
				callbacks->deviceReseted();
			    break;
			case firmware::STATE_SENSORS_CONFIGURED:
			    if(callbacks)
				callbacks->configureDone();
			    break;    
			case firmware::STATE_SENSOR_ERROR:
			    if(callbacks)
				callbacks->configurationError();
			    break;    
			default:
			    break;
		    }
	    }
		break;

	    default:
		std::cout << "Got unknow message with id " << msg.packetId <<  " " << firmware::getPacketName(msg.packetId) << std::endl;
		break;
	}
    }
    else
    {	
	unsigned int lowHandlerId = msg.packetId - firmware::PACKET_ID_LOWIDS_START;
	if(packetHandlers[lowHandlerId])
	{
	    packetHandlers[lowHandlerId]->processMsg(msg);
	}
	else
	{
	    std::cout << "Got unknow extension message with id " << msg.packetId << std::endl;
	}
    }
}

void Reader::configurationError(const Packet &msg)
{
    std::cout << "Error: Configure failed for hbridge " << handle->getBoardId() << std::endl;
    std::cout << "Reason:" << std::endl;
    switch(msg.packetId)
    {
	case PACKET_ID_CLEAR_SENSOR_ERROR:
	    std::cout << "Clear sensor config message was not acked" << std::endl;
	    break;	    
	case firmware::PACKET_ID_SET_SENSOR_CONFIG:
	    std::cout << "Sensor config message was not acked" << std::endl;
	    break;
	default:
	    std::cout << "Driver error: This means there is a bug in the driver" << std::endl;
	    std::cout << "Driver error: Got send error for unknown package with id " << msg.packetId << std::endl;
	    break;
    };
    
    if(callbacks)
	callbacks->configurationError();

}

void Reader::configureDone()
{
    configured = true;

    if(callbacks)
	callbacks->configureDone();
}

void Reader::gotError(const hbridge::ErrorState& error)
{
    configured = false;
    if(callbacks)
	callbacks->gotErrorStatus(error);
}


const hbridge::Encoder& Reader::getExternalEncoder()
{
    return encoderExtern;
}

const hbridge::Encoder& Reader::getInternalEncoder()
{
    return encoderIntern;
}

void Reader::shutdown()
{

}

}
