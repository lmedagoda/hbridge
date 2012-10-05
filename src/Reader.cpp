#include "Reader.hpp"
#include "Controller.hpp"
#include "../protocol.hpp"
#include "Protocol.hpp"

#define HBRIDGE_BOARD_ID(x) ((x + 1) << 5)

namespace hbridge {

Reader::Reader(int id, Protocol* protocol): protocol(protocol), callbacks(NULL), boardId(id), configured(false)
{
    canMsgHandlers.resize(firmware::NUM_PACKET_IDS - firmware::END_BASE_PACKETS);
}

void Reader::setCallbacks(Reader::CallbackInterface* cbs)
{
    callbacks = cbs;
}

bool Reader::isWritable()
{
    return configured;
}

void Reader::startConfigure()
{
    sendEncConf1();
    sendEncConf2();
    
    //configure all controllers
    for(std::vector<Controller *>::iterator it = protocol->getControllers(boardId).begin(); 
	it != protocol->getControllers(boardId).end(); it++)
    {
	(*it)->sendControllerConfig();
    }
    
    sendConf1Msg();
    sendConf2Msg();
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

void Reader::sendConf1Msg()
{
    Configuration &cfg(configuration.base_config);
    canbus::Message cfgMsg1;
    
    firmware::configure1Data *cfg1 =
	reinterpret_cast<firmware::configure1Data *>(cfgMsg1.data);
    cfg1->openCircuit                = cfg.openCircuit;
    cfg1->externalTempSensor         = cfg.externalTempSensor;
    cfg1->unused                     = 0;
    cfg1->maxMotorTemp               = cfg.maxMotorTemp;
    cfg1->maxMotorTempCount          = cfg.maxMotorTempCount;
    cfg1->maxBoardTemp               = cfg.maxBoardTemp;
    cfg1->maxBoardTempCount          = cfg.maxBoardTempCount;
    cfg1->timeout                    = cfg.timeout;
    switch(cfg.controllerInputEncoder)
    {
	case hbridge::INTERNAL:
	    cfg1->controllerInputEncoder = firmware::INTERNAL;
	    break;
	case hbridge::EXTERNAL:
	    cfg1->controllerInputEncoder = firmware::EXTERNAL;
	    break;
    }
    

    cfgMsg1.can_id = HBRIDGE_BOARD_ID(boardId) | firmware::PACKET_ID_SET_CONFIGURE;
    cfgMsg1.size = sizeof(firmware::configure1Data);
    
    protocol->sendCanPacket(boardId, cfgMsg1, true, boost::bind(&Reader::configurationError, this));
}

void Reader::sendConf2Msg()
{
    Configuration &cfg(configuration.base_config);
    canbus::Message cfgMsg2;

    firmware::configure2Data *cfg2 =
	reinterpret_cast<firmware::configure2Data *>(cfgMsg2.data);

    cfg2->maxCurrent                 = cfg.maxCurrent;
    cfg2->maxCurrentCount            = cfg.maxCurrentCount;
    cfg2->pwmStepPerMs               = cfg.pwmStepPerMs;

    cfgMsg2.can_id = HBRIDGE_BOARD_ID(boardId) | firmware::PACKET_ID_SET_CONFIGURE2;
    cfgMsg2.size = sizeof(firmware::configure2Data);

    protocol->sendCanPacket(boardId, cfgMsg2, true, boost::bind(&Reader::configurationError, this), boost::bind(&Reader::configureDone, this));

}

void Reader::fillEncoderMessage(canbus::Message& msg, const hbridge::EncoderConfiguration& cfg)
{
    firmware::encoderConfiguration *data =
	reinterpret_cast<firmware::encoderConfiguration *>(msg.data);

    data->encoderType = static_cast<firmware::encoderTypes>(cfg.type);    
    data->ticksPerTurn = cfg.ticksPerTurnDivided;
    data->tickDivider = cfg.tickDivider;

    msg.size = sizeof(firmware::encoderConfiguration);
}

void Reader::sendEncConf1()
{
    canbus::Message msg;
    fillEncoderMessage(msg, configuration.encoder_config_intern);
    
    msg.can_id = HBRIDGE_BOARD_ID(boardId) | firmware::PACKET_ID_ENCODER_CONFIG_INTERN;
    msg.size = sizeof(firmware::encoderConfiguration);
    
    protocol->sendCanPacket(boardId, msg, true, boost::bind(&Reader::configurationError, this));
}


void Reader::sendEncConf2()
{
    canbus::Message msg;
    fillEncoderMessage(msg, configuration.encoder_config_extern);

    msg.can_id = HBRIDGE_BOARD_ID(boardId) | firmware::PACKET_ID_ENCODER_CONFIG_EXTERN;
    msg.size = sizeof(firmware::encoderConfiguration);

    protocol->sendCanPacket(boardId, msg, true, boost::bind(&Reader::configurationError, this));
}


int Reader::getCurrentTickDivider() const
{
    int tickDivider = 1;
    switch(configuration.base_config.controllerInputEncoder)
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

void Reader::registerCanMsgHandler(boost::function<void (const canbus::Message &)> cb, int canId)
{
    int newId = canId - firmware::END_BASE_PACKETS;
    int size = canMsgHandlers.size();
    if(newId < 0 || newId > size)
	throw std::runtime_error("Reader::registerCanMsgHandler: Error tried to register callback for invalid id");
	
    canMsgHandlers[canId - firmware::END_BASE_PACKETS] = cb;
}

void Reader::unregisterCanMsgHandler(int canId)
{
    canMsgHandlers[canId - firmware::END_BASE_PACKETS] = NULL;
}


void Reader::processMsg(const canbus::Message& msg)
{
    unsigned int msgId = msg.can_id & 0x1f;
    if(msgId < firmware::END_BASE_PACKETS)
    {
	switch (msgId)
	{
	    case firmware::PACKET_ID_ERROR: {
		const firmware::errorData *edata =
		    reinterpret_cast<const firmware::errorData *>(msg.data);
		
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
		this->state.can_time = msg.can_time;
		
		if(callbacks)
		    callbacks->gotErrorStatus(state.error);

	    }
	    break;
	    case firmware::PACKET_ID_STATUS:
	    {
		const firmware::statusData *data =
		    reinterpret_cast<const firmware::statusData *>(msg.data);

		this->state.index   = data->index;
		this->state.current = data->currentValue; // Current in [mA]
		this->state.pwm     = static_cast<float>(data->pwm) / 1800; // PWM in [-1; 1]
		encoderIntern.setRawEncoderValue(data->position);
		encoderExtern.setRawEncoderValue(data->externalPosition);

		this->state.position = encoderIntern.getAbsoluteTurns();
		this->state.positionExtern = encoderExtern.getAbsoluteTurns();

		//getting an status package is an implicit cleaner for all error states
		bzero(&(this->state.error), sizeof(struct ErrorState));
		this->state.can_time = msg.can_time;
		
		if(callbacks)
		    callbacks->gotStatus(state);
		break;
	    }
	    case firmware::PACKET_ID_EXTENDED_STATUS:
	    {
		const firmware::extendedStatusData *esdata = 
		    reinterpret_cast<const firmware::extendedStatusData *>(msg.data);
		    
		this->state.temperature = esdata->temperature;
		this->state.motorTemperature = esdata->motorTemperature;
		if(callbacks)
		    callbacks->gotStatus(state);
		break;
	    }

	    default:
		std::cout << "Got unknow message with id " << msg.can_id << std::endl;
		break;
	}
    }
    else
    {	
	if(canMsgHandlers[msgId])
	{
	}
	else
	{
	    std::cout << "Got unknow extension message with id " << msg.can_id << std::endl;
	}
    }
}

void Reader::configurationError()
{
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