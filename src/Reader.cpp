#include "Reader.hpp"
#include "Protocol.hpp"

using namespace firmware;

namespace hbridge {

Reader::Reader(uint32_t boardId, Protocol *protocol): boardId(boardId), protocol(protocol), callbacks(NULL), configured(false), error(false), gotBoardState(false), dstate(READER_OFF)
{
    protocol->registerReceiver(this, boardId);
}

void Reader::setCallbacks(Reader::CallbackInterface* cbs)
{
    callbacks = cbs;
}

void Reader::startConfigure()
{
    configured = false;
    error = false;
    gotBoardState = false;
    requestDeviceState();
    dstate = READER_REQUEST_STATE;
}

void Reader::resetDevice()
{
    Packet msg;
    msg.packetId = PACKET_ID_CLEAR_SENSOR_ERROR;
    
    protocol->sendPacket(boardId, msg, true, boost::bind(&Reader::configurationError, this, _1));
}

void Reader::requestDeviceState()
{
    Packet msg;
    msg.packetId = PACKET_ID_REQUEST_STATE;
    
    protocol->sendPacket(boardId, msg, true, boost::bind(&Reader::configurationError, this, _1));
}


void Reader::setConfiguration(const hbridge::SensorConfiguration& config)
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
    SensorConfiguration &cfg(configuration);
    Packet msg;
    msg.packetId = firmware::PACKET_ID_SET_SENSOR_CONFIG;
    msg.data.resize(sizeof(firmware::sensorConfig));
    
    firmware::sensorConfig *cfg1 = reinterpret_cast<firmware::sensorConfig *>(msg.data.data());

    cfg1->externalTempSensor = cfg.externalTempSensor;
    cfg1->statusEveryMs = 1000 / cfg.statusFrequency;
    cfg1->encoder1Config.encoderType = static_cast<firmware::encoderTypes>(configuration.encoder_config_intern.type);
    cfg1->encoder1Config.ticksPerTurn = configuration.encoder_config_intern.ticksPerTurnDivided;
    cfg1->encoder1Config.tickDivider = configuration.encoder_config_intern.tickDivider;
    cfg1->encoder1Config.leapTickCounter = configuration.encoder_config_intern.leapTickValue;

    cfg1->encoder2Config.encoderType = static_cast<firmware::encoderTypes>(configuration.encoder_config_extern.type);
    cfg1->encoder2Config.ticksPerTurn = configuration.encoder_config_extern.ticksPerTurnDivided;
    cfg1->encoder2Config.tickDivider = configuration.encoder_config_extern.tickDivider;
    cfg1->encoder2Config.leapTickCounter = configuration.encoder_config_extern.leapTickValue;

    protocol->sendPacket(boardId, msg, true, boost::bind(&Reader::configurationError, this, _1));
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
		
                state.can_time = msg.receiveTime;
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
		    callbacks->gotErrorStatus(boardId, state.error);

	    }
	    break;
	    case firmware::PACKET_ID_STATUS:
	    {
		const firmware::statusData *data =
		    reinterpret_cast<const firmware::statusData *>(msg.data.data());

                state.can_time = msg.receiveTime;
		state.index   = data->index;
		state.current = data->currentValue; // Current in [mA]
		state.pwm     = static_cast<float>(data->pwm) / std::numeric_limits<int16_t>::max() * 16;
		encoderIntern.setRawEncoderValue(data->position);
		encoderExtern.setRawEncoderValue(data->externalPosition);

		state.position = encoderIntern.getAbsoluteTurns();
		state.positionExtern = encoderExtern.getAbsoluteTurns();

		//getting an status package is an implicit cleaner for all error states
		bzero(&(state.error), sizeof(struct ErrorState));

		gotBoardState = true;
		
		if(callbacks)
		    callbacks->gotStatus(boardId, state);
		break;
	    }
	    case firmware::PACKET_ID_EXTENDED_STATUS:
	    {
		const firmware::extendedStatusData *esdata = 
		    reinterpret_cast<const firmware::extendedStatusData *>(msg.data.data());
		    
                state.can_time = msg.receiveTime;
                state.temperature = esdata->temperature;
		state.motorTemperature = esdata->motorTemperature;
		if(callbacks)
		    callbacks->gotStatus(boardId, state);
		break;
	    }
	    case firmware::PACKET_ID_ANNOUNCE_STATE:
	    {
		processStateMsg(msg);
		break;
	    }

	    default:
		std::cout << "Got unknow message with id " << msg.packetId <<  " " << firmware::getPacketName(msg.packetId) << std::endl;
		break;
	}
    }
}


void Reader::processStateMsg(const hbridge::Packet& msg)
{
    const firmware::announceStateData *stateData = 
	reinterpret_cast<const firmware::announceStateData *>(msg.data.data());

    std::cout << "GOT STATE ACCOUNCE hb " << boardId << " new state " <<  getStateName(stateData->curState) <<  std::endl;

    switch(dstate)
    {
	case READER_OFF:
	    break;
	case READER_REQUEST_STATE:
	{
	    switch(stateData->curState)
	    {
		case firmware::STATE_UNCONFIGURED:
		    sendConfigureMsg();
		    dstate = READER_CONFIG_SENT;
		    break;
		case firmware::STATE_SENSORS_CONFIGURED:
		    configureDone();
		    break;    
		case firmware::STATE_SENSOR_ERROR:
		    resetDevice();
		    break;    
		default:
		    configureDone();
		    break;
	    }
	    break;
	}
	case READER_CONFIG_SENT:
	    switch(stateData->curState)
	    {
		case firmware::STATE_SENSORS_CONFIGURED:
		    configureDone();
		    break;    
		case firmware::STATE_UNCONFIGURED:
		case firmware::STATE_SENSOR_ERROR:
		    std::cout << "Board id " << boardId << " went into error state " << std::endl;
		    error = true;
		    if(callbacks)
			callbacks->configurationError();
		    break;    
		default:
		    std::cout << "Board id " << boardId << " got unexpected packet " << std::endl;
		    break;    
	    }
	    break;
	case READER_RUNNING:
	    switch(stateData->curState)
	    {
		case firmware::STATE_UNCONFIGURED:
		case firmware::STATE_SENSOR_ERROR:
		    error = true;
		    if(callbacks)
			callbacks->configurationError();
		    break;    
		default:
		    break;
	    }
	    break;
    };
	
    this->internal_state = stateData->curState;
    if (callbacks)
	callbacks->gotInternalState(boardId, stateData->curState);    

}

void Reader::configurationError(const Packet &msg)
{
    std::cout << "Error: Configure failed for hbridge " << boardId << std::endl;
    std::cout << "Reason:" << std::endl;
    switch(msg.packetId)
    {
	case PACKET_ID_REQUEST_STATE:
	    std::cout << "Motor driver did not ack state request packet." << std::endl;
	    break;	    	    
	case PACKET_ID_CLEAR_SENSOR_ERROR:
	    std::cout << "Clear sensor config message was not acked" << std::endl;
	    break;	    
	case firmware::PACKET_ID_SET_SENSOR_CONFIG:
	    std::cout << "Sensor config message was not acked" << std::endl;
	    break;
	default:
	    std::cout << "Driver error: This means there is a bug in the driver" << std::endl;
	    std::cout << "Driver error: Got send error for unknown package with id " << msg.packetId << " packet name " << getPacketName(msg.packetId) << std::endl;
	    break;
    };
    
    error = true;
    
    if(callbacks)
	callbacks->configurationError();

}

bool Reader::hasError()
{
    return error;
}


void Reader::configureDone()
{
    dstate = READER_RUNNING;
    configured = true;

    if(callbacks)
	callbacks->configureDone();
}

bool Reader::isConfigured()
{
    return (dstate == READER_RUNNING) && (internal_state >= firmware::STATE_SENSORS_CONFIGURED);
}

void Reader::gotError(const hbridge::ErrorState& errorState)
{
    error = true;
    configured = false;
    if(callbacks)
	callbacks->gotErrorStatus(boardId, errorState);
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
