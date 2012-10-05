#include "HBridgeDriver.hpp"
#include <strings.h>
#include <stdlib.h>
#include <iostream>

#include "protocol.hpp"
#include <stdexcept>

#define HBRIDGE_BOARD_ID(x) ((x + 1) << 5)
#include <stdio.h>
#include <base/float.h>

using namespace std;
namespace hbridge
{
 
Encoder::Encoder()
{
    lastPositionInTurn = 0;
    turns = 0;
    zeroPosition = 0;
    gotValidReading = false;    

}

void Encoder::setConfiguration(EncoderConfiguration& cfg)
{
    if(encoderConfig.ticksPerTurn == cfg.ticksPerTurn)
        return;
    
    encoderConfig = cfg;
    
    //we go sane and reset the encoder if wrap value changes
    lastPositionInTurn = 0;
    turns = 0;
    gotValidReading = false;    
}

void Encoder::setZeroPosition(Ticks zeroPos) 
{
    zeroPosition = zeroPos;
};

void Encoder::setRawEncoderValue(uint value)
{
    //reverse scale down for transmission
    value *= encoderConfig.tickDivider;
    
    if(!gotValidReading)
    {
	if(value > 0)
	{
	    gotValidReading = true;
	    lastPositionInTurn = value;
	}
    }
    else 
    {
	int diff = value - lastPositionInTurn;
	lastPositionInTurn = value;
	
	// We assume that a motor rotates less than half a turn per [ms]
	// (a status packet is sent every [ms])
	if ((uint) abs(diff) > encoderConfig.ticksPerTurnMotorDriver / 2)
	{
	    turns += (diff < 0 ? 1 : -1);
	}
    }
}

double Encoder::getAbsoluteTurns() const
{
    if(!gotValidReading)
	return base::unknown<double>();
    
    Ticks allMotorTicks = turns* ((int64_t)encoderConfig.ticksPerTurnMotorDriver) + ((int)lastPositionInTurn) - zeroPosition;
    double turns = allMotorTicks / encoderConfig.ticksPerTurn;
    return turns;
}


Ticks Encoder::getMotorTicksFromAbsoluteTurn(double targetValue) const
{
    double curPos = getAbsoluteTurns();
    if(fabs(curPos - targetValue) > 1)
	throw std::runtime_error("Target value is more than one turn apart");
    
    if(base::isUnknown<double>(curPos))
	throw std::runtime_error("Tried to access encoder while it is unknown (did not pass zero mark yet)");
    
    int64_t target_ticks = targetValue * encoderConfig.ticksPerTurn + zeroPosition;
    int64_t targetInTurn_ticks = target_ticks % encoderConfig.ticksPerTurnMotorDriver;
    
    return targetInTurn_ticks;
}


const EncoderConfiguration& Encoder::getEncoderConfig() const
{
    return encoderConfig;
}

    Driver::Driver() :
        states()
    {
	reset();
        for (int i = 0; i < BOARD_COUNT; ++i)
	{
            current_modes[i] = base::actuators::DM_UNINITIALIZED;
	    newPosPIDDebug[i] = false;
	    newSpeedPIDDebug[i] = false;
	}
    }

    Driver::~Driver()
    {
    }

int Driver::getCurrentTickDivider(int index) const
{
    int tickDivider = 1;
    switch(configuration[index].controllerInputEncoder)
    {
	case INTERNAL:
	    tickDivider = encoderIntern[index].getEncoderConfig().tickDivider;
	    break;
	case EXTERNAL:
	    tickDivider = encoderExtern[index].getEncoderConfig().tickDivider;
	    break;
    }
    return tickDivider;
}

    void Driver::reset()
    {
        bzero(&this->states, BOARD_COUNT * sizeof(BoardState));
    }

    int Driver::getBoardIdFromMessage(const canbus::Message& msg) const
    {
	int index = ((msg.can_id & 0x1E0) >> 5) - 1;
	return index;
    }

    bool Driver::updateFromCAN(const canbus::Message& msg)
    {
	int index = getBoardIdFromMessage(msg);
	
	//broadcast message is allways a controll message
	//and thuis does not update the internal state
	if(index == -1) {
	    if(msg.can_id == firmware::PACKET_ID_EMERGENCY_STOP)
	    {
	        for(int i = 0; i < BOARD_COUNT; i++)
		{
		    states[i].error.emergencyOff = true;
		}
	    }
	    std::cout << "Got broadcast message" << std::endl;
	    return false;
	}
	
	if ((index < -1) || (index > (BOARD_COUNT - 1)))
	{
	    // Invalid id specified in packet
	    throw std::invalid_argument("Got message with non existing board ID");
	}

        switch (msg.can_id & 0x1f)
        {
	    case firmware::PACKET_ID_ERROR: {
		const firmware::errorData *edata =
		    reinterpret_cast<const firmware::errorData *>(msg.data);
                
                states[index].index   = edata->index;
		//hbridge is off, no current is flowing
                states[index].current = 0;
		states[index].pwm = 0;
		states[index].error.badConfig = edata->badConfig;
		states[index].error.boardOverheated = edata->boardOverheated;
		states[index].error.encodersNotInitialized = edata->encodersNotInitalized;
		states[index].error.motorOverheated = edata->motorOverheated;
		states[index].error.overCurrent = edata->overCurrent;
		states[index].error.timeout = edata->timeout;
		states[index].temperature = edata->temperature;

		encoderIntern[index].setRawEncoderValue(edata->position);
		encoderExtern[index].setRawEncoderValue(edata->externalPosition);
		
		this->states[index].position = encoderIntern[index].getAbsoluteTurns();
		this->states[index].positionExtern = encoderExtern[index].getAbsoluteTurns();
                this->states[index].can_time = msg.can_time;
	    }
	    break;
            case firmware::PACKET_ID_STATUS:
            {
		const firmware::statusData *data =
		    reinterpret_cast<const firmware::statusData *>(msg.data);

                this->states[index].index   = data->index;
                this->states[index].current = data->currentValue; // Current in [mA]
                this->states[index].pwm     = static_cast<float>(data->pwm) / 1800; // PWM in [-1; 1]
		//printf("Current PWM Level for %i is: %f (%i) Direction: %i\n",index,this->states[index].pwm,data->pwm,directions[index]);
		encoderIntern[index].setRawEncoderValue(data->position);
		encoderExtern[index].setRawEncoderValue(data->externalPosition);

		this->states[index].position = encoderIntern[index].getAbsoluteTurns();
		this->states[index].positionExtern = encoderExtern[index].getAbsoluteTurns();

		//getting an status package is an implicit cleaner for all error states
	        bzero(&(this->states[index].error), sizeof(struct ErrorState));
                this->states[index].can_time = msg.can_time;
                break;
            }
	    case firmware::PACKET_ID_EXTENDED_STATUS:
	    {
		const firmware::extendedStatusData *esdata = 
		    reinterpret_cast<const firmware::extendedStatusData *>(msg.data);
		    
		this->states[index].temperature = esdata->temperature;
		this->states[index].motorTemperature = esdata->motorTemperature;
		break;
	    }

            case firmware::PACKET_ID_SPEED_DEBUG: {
		int tickDivider = getCurrentTickDivider(index);

		const firmware::speedDebugData *data =
		    reinterpret_cast<const firmware::speedDebugData *>(msg.data);
		    
		speedControllerDebug[index].encoderValue = data->encoderVal * tickDivider;
		speedControllerDebug[index].pwmValue = data->pwmVal;
		speedControllerDebug[index].speedValue = data->speedVal;
		speedControllerDebug[index].targetValue = data->targetVal;
                break;
	    }
            case firmware::PACKET_ID_POS_DEBUG:
            {
                int tickDivider = getCurrentTickDivider(index);
                const firmware::posDebugData * data =
                    reinterpret_cast<const firmware::posDebugData *> (msg.data);

                positionControllerDebug[index].encoderValue = data->encoderVal * tickDivider;
                positionControllerDebug[index].pwmValue = data->pwmVal;
                positionControllerDebug[index].positionValue = data->posVal * tickDivider;
                positionControllerDebug[index].targetValue = data->targetVal * tickDivider;
		newPosPIDDebug[index] = true;
                break;
            }
            case firmware::PACKET_ID_PID_DEBUG_SPEED:
            {
                const firmware::pidDebugData * data =
                    reinterpret_cast<const firmware::pidDebugData *> (msg.data);

                speedControllerDebug[index].pidDebug.dPart = data->dPart;
                speedControllerDebug[index].pidDebug.iPart = data->iPart;
                speedControllerDebug[index].pidDebug.pPart = data->pPart;
                speedControllerDebug[index].pidDebug.minMaxPidOutput = data->minMaxPidOutput;
		newSpeedPIDDebug[index] = true;
                break;
            }
            case firmware::PACKET_ID_PID_DEBUG_POS:
            {
                const firmware::pidDebugData * data =
                    reinterpret_cast<const firmware::pidDebugData *> (msg.data);

                positionControllerDebug[index].pidDebug.dPart = data->dPart;
                positionControllerDebug[index].pidDebug.iPart = data->iPart;
                positionControllerDebug[index].pidDebug.pPart = data->pPart;
                positionControllerDebug[index].pidDebug.minMaxPidOutput = data->minMaxPidOutput;
                break;
            }
	    default:
	      std::cout << "Got unknow message with id " << msg.can_id << std::endl;
		//whatever it is, it did not update our internal state
		return false;
	      break;
        }

        return true;
    }

    const BoardState &Driver::getState(int board) const
    {
	if(board < 0 || board > BOARD_COUNT)
	    throw std::out_of_range("Wrong board id");
        return this->states[board];
    }

    canbus::Message Driver::emergencyShutdown() const
    {
        canbus::Message msg;

        bzero(&msg, sizeof(canbus::Message));

        msg.can_id = firmware::PACKET_ID_EMERGENCY_STOP;
        msg.size = 0;

        return msg;
    }

    canbus::Message Driver::setDriveMode(hbridge::BOARD_SET set, hbridge::DRIVE_MODE mode)
    {
        return setDriveMode(set, mode, mode, mode, mode);
    }

    canbus::Message Driver::setDriveMode(BOARD_SET set, DRIVE_MODE board1, DRIVE_MODE board2,
                                      DRIVE_MODE board3, DRIVE_MODE board4)
    {
        canbus::Message msg;

        bzero(&msg, sizeof(canbus::Message));

        firmware::setModeData *data =
            reinterpret_cast<firmware::setModeData *>(msg.data);

        data->board1Mode = (firmware::controllerModes)board1;
        data->board2Mode = (firmware::controllerModes)board2;
        data->board3Mode = (firmware::controllerModes)board3;
        data->board4Mode = (firmware::controllerModes)board4;

	switch(set) {
	    case BOARDS_14:
        	current_modes[0] = board1;
	        current_modes[1] = board2;
	        current_modes[2] = board3;
	        current_modes[3] = board4;
		msg.can_id = firmware::PACKET_ID_SET_MODE14;
		break;
	    case BOARDS_58:
        	current_modes[4] = board1;
	        current_modes[5] = board2;
	        current_modes[6] = board3;
	        current_modes[7] = board4;
		msg.can_id = firmware::PACKET_ID_SET_MODE58;
		break;
	}
        msg.size = sizeof(firmware::setModeData);

        return msg;
    }

    canbus::Message Driver::setTargetValues(BOARD_SET set, double value1, double value2,
                                         double value3, double value4) const
    {
        double value_array[4] = { value1, value2, value3, value4 };
        return setTargetValues(set, value_array);
    }
    canbus::Message Driver::setTargetValues(BOARD_SET set, double* targets) const
    {
        canbus::Message msg;

        bzero(&msg, sizeof(canbus::Message));

        firmware::setValueData *data =
            reinterpret_cast<firmware::setValueData *>(msg.data);

	const hbridge::DRIVE_MODE *current_modes_local;
	switch(set) {
	    case BOARDS_14:
	        current_modes_local = current_modes;
		break;
	    case BOARDS_58:
	        current_modes_local = current_modes + 4;
		break;
	}

	
        short int* values = &(data->board1Value);

        for (int i = 0; i < 4; ++i)
        {
	    if(targets[i] > 1.0 || targets[i] < -1.0)
                    throw std::out_of_range("Given target value is out of bound. Value Range: [-1.0,1.0]");

	    switch(current_modes_local[i])
            {
            case base::actuators::DM_SPEED:
                values[i] = targets[i] * configuration[i].maxSpeed;
                break;

            case base::actuators::DM_PWM:
                values[i] = targets[i] * configuration[i].maxPWM;
                break;

            case base::actuators::DM_POSITION:
            {
		//TODO add min max check
                int tickDivider = getCurrentTickDivider(i);
		switch(configuration[i].controllerInputEncoder)
		{
		    case EXTERNAL:
			values[i] = encoderExtern[i].getMotorTicksFromAbsoluteTurn(targets[i]) / tickDivider;
			break;
		    case INTERNAL:
			values[i] = encoderIntern[i].getMotorTicksFromAbsoluteTurn(targets[i]) / tickDivider;
			break;
		}
                break;
	    }
            default:
                throw std::runtime_error("setTargetValues called before setDriveMode");
            }
        }

	switch(set) {
	    case BOARDS_14:
		msg.can_id = firmware::PACKET_ID_SET_VALUE14;
		break;
	    case BOARDS_58:
		msg.can_id = firmware::PACKET_ID_SET_VALUE58;
		break;
	}

        msg.size = sizeof(firmware::setValueData);

        return msg;
    }

    canbus::Message Driver::setSpeedPID(int board,
                                     double kp, double ki, double kd,
                                     double minMaxValue) const
    {
        canbus::Message msg;
        bzero(&msg, sizeof(canbus::Message));
        setPID(msg, kp, ki, kd, minMaxValue);
        msg.can_id = HBRIDGE_BOARD_ID(board) | firmware::PACKET_ID_SET_PID_SPEED;
        return msg;
    }

    canbus::Message Driver::setPositionPID(int board,
                                        double kp, double ki, double kd,
                                        double minMaxValue) const
    {
        canbus::Message msg;
        bzero(&msg, sizeof(canbus::Message));
        setPID(msg, kp, ki, kd, minMaxValue);
        msg.can_id = HBRIDGE_BOARD_ID(board) | firmware::PACKET_ID_SET_PID_POS;
        return msg;
    }

    MessagePair Driver::setConfiguration(int board,
                                         const Configuration &cfg)
    {
	//save configuration
	configuration[board] = cfg;
	
        MessagePair msgs;
    
        firmware::configure1Data *cfg1 =
            reinterpret_cast<firmware::configure1Data *>(msgs.first.data);
        firmware::configure2Data *cfg2 =
            reinterpret_cast<firmware::configure2Data *>(msgs.second.data);
    
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
        
        cfg2->maxCurrent                 = cfg.maxCurrent;
        cfg2->maxCurrentCount            = cfg.maxCurrentCount;
        cfg2->pwmStepPerMs               = cfg.pwmStepPerMs;

        msgs.first.can_id = HBRIDGE_BOARD_ID(board) | firmware::PACKET_ID_SET_CONFIGURE;
        msgs.first.size = sizeof(firmware::configure1Data);

        msgs.second.can_id = HBRIDGE_BOARD_ID(board) | firmware::PACKET_ID_SET_CONFIGURE2;
        msgs.second.size = sizeof(firmware::configure2Data);

	//reset internal emergency off status
	states[board].error.emergencyOff = false;

        return msgs;
    }

    canbus::Message Driver::setInternalEncoderConfiguration(int board, EncoderConfiguration cfg)
    {      
	//calcualte correct tick divider for 16 bit width
	cfg.tickDivider = cfg.ticksPerTurn / (1<<16) +1;
	cfg.validate();
	
	encoderIntern[board].setConfiguration(cfg);
	encoderIntern[board].setZeroPosition(cfg.zeroPosition);

	canbus::Message msg = setEncoderConfiguration(board, cfg);

	msg.can_id = HBRIDGE_BOARD_ID(board) | firmware::PACKET_ID_ENCODER_CONFIG_INTERN;
        msg.size = sizeof(firmware::encoderConfiguration);

		
	return msg;
    }
    
    canbus::Message Driver::setExternalEncoderConfiguration(int board, hbridge::EncoderConfiguration cfg)
    {
	//calcualte correct tick divider for 12 bit width
	cfg.tickDivider = cfg.ticksPerTurn / (1<<12) +1;
	cfg.validate();

	encoderExtern[board].setConfiguration(cfg);
	encoderExtern[board].setZeroPosition(cfg.zeroPosition);
		
	canbus::Message msg = setEncoderConfiguration(board, cfg);

	msg.can_id = HBRIDGE_BOARD_ID(board) | firmware::PACKET_ID_ENCODER_CONFIG_EXTERN;
        msg.size = sizeof(firmware::encoderConfiguration);

	return msg;
    }

    
    canbus::Message Driver::setEncoderConfiguration(int board, const EncoderConfiguration &cfg)
    {
	canbus::Message msg;
        bzero(&msg, sizeof(canbus::Message));
	
	EncoderConfiguration icfg = cfg;
	
	icfg.validate();

        firmware::encoderConfiguration *data =
            reinterpret_cast<firmware::encoderConfiguration *>(msg.data);

	data->encoderType = static_cast<firmware::encoderTypes>(icfg.type);    
	data->ticksPerTurn = icfg.ticksPerTurnDivided;
	data->tickDivider = icfg.tickDivider;

        msg.size = sizeof(firmware::encoderConfiguration);

	return msg;
    }
    
    void Driver::setPID(canbus::Message &msg,
                       double kp, double ki, double kd,
                       double minMaxValue) const
    {
		//check if values exceed signed short
	if(kp * 100 > (1<<16) || kp * 100 < -(1<<16) ||
	   ki * 100 > (1<<16) || ki * 100 < -(1<<16) ||
	   kd * 100 > (1<<16) || kd * 100 < -(1<<16))
	    throw std::runtime_error("Given PID Parameters are out of bound");
	
        firmware::setPidData *data = reinterpret_cast<firmware::setPidData *>(msg.data);
	//convert given parameters to fixed point values for transmission
        data->kp = kp * 100;
        data->ki = ki * 100;
        data->kd = kd * 100;
        data->minMaxPidOutput = minMaxValue;
        
        msg.size = sizeof(firmware::setPidData);
    }
    
    const hbridge::PositionControllerDebug& Driver::getPositionControllerDebugData(const int board) const
    {
	return positionControllerDebug[board];
    }

    const hbridge::SpeedControllerDebug& Driver::getSpeedControllerDebugData(const int board) const
    {
	return speedControllerDebug[board];
    }
    
bool Driver::getPositionControllerDebugData(const int board, PositionControllerDebug& data)
{
    bool ret = newPosPIDDebug[board];
    newPosPIDDebug[board] = false;
    data = positionControllerDebug[board];
    return ret;
}

bool Driver::getSpeedControllerDebugData(const int board, SpeedControllerDebug& data)
{
    bool ret = newSpeedPIDDebug[board];
    newSpeedPIDDebug[board] = false;
    data = speedControllerDebug[board];
    return ret;
}

canbus::Message Driver::setPositionControllerConfiguration(int board, const PositionControllerConfiguration& posCtrlCfg) const
{
    return setPositionControllerConfiguration(board, posCtrlCfg.hysteresisActive, posCtrlCfg.allowWrapAround, posCtrlCfg.minHystDist, posCtrlCfg.maxHystDist, posCtrlCfg.overDistCount);
}


canbus::Message Driver::setPositionControllerConfiguration(int board, bool hysteresisActive, bool allowWrapAround, double minHystDist, double maxHystDist, int overDistCount) const
{
    canbus::Message ret;
    ret.can_id = firmware::PACKET_ID_POS_CONTROLLER_DATA | HBRIDGE_BOARD_ID(board);
    ret.size = sizeof(firmware::posControllerData);
    firmware::posControllerData *data = (firmware::posControllerData *) ret.data;
    data->hysteresisActive = hysteresisActive;
    data->allowWrapAround = allowWrapAround;
    data->minHystDist = maxHystDist;
    data->maxHystDist = maxHystDist;
    data->overDistCount = overDistCount;
    data->unused = 0;
    
    return ret;
}

    
}

