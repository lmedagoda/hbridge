#include "HBridgeDriver.hpp"
#include <strings.h>
#include <stdlib.h>
#include <iostream>

#include "protocol.hpp"
#include <stdexcept>

#define HBRIDGE_BOARD_ID(x) ((x + 1) << 5)
#include <stdio.h>

using namespace std;
namespace hbridge
{
 
Encoder::Encoder()
{
    wrapValue = 0;
    lastPositionInTurn = 0;
    turns = 0;
    zeroPosition = 0;
    gotValidReading = false;
}

void Encoder::setWrapValue(uint value)
{
    if(wrapValue == value)
        return;

    //we go sane and reset the encoder if wrap value changes
    wrapValue = value;
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

    if(!gotValidReading)
    {
	if(value > 0)
	{
	    gotValidReading = true;
	    lastPositionInTurn = value;
	}
	else
	{
	    //report 0 to the outside world
	    lastPositionInTurn = 0;
	}
    }
    else 
    {
	int diff = value - lastPositionInTurn;
	lastPositionInTurn = value;
	
	// We assume that a motor rotates less than half a turn per [ms]
	// (a status packet is sent every [ms])
	if (abs(diff) > wrapValue / 2)
	{
	    turns += (diff < 0 ? 1 : -1);
	}
    }
}

Ticks Encoder::getAbsolutPosition()
{
    Ticks ret = turns* ((int)wrapValue) + ((int)lastPositionInTurn) - zeroPosition;
    return ret;
}

    Driver::Driver() :
        states()
    {
	reset();
	for(int i=0;i< BOARD_COUNT; i++){
		directions[i]   = 1;
	}

        directions[MOTOR_REAR_LEFT]   = -1;
        directions[MOTOR_FRONT_LEFT]  = -1;
        directions[MOTOR_REAR_RIGHT]  = 1;
        directions[MOTOR_FRONT_RIGHT] = 1;
        for (int i = 0; i < BOARD_COUNT; ++i)
            current_modes[i] = DM_UNINITIALIZED;
    }

    Driver::~Driver()
    {
    }

    void Driver::reset()
    {
        bzero(&this->states, BOARD_COUNT * sizeof(BoardState));
    }

    int Driver::getBoardIdFromMessage(const can::Message& msg) const
    {
	int index = ((msg.can_id & 0xE0) >> 5) - 1;
	return index;
    }

    bool Driver::updateFromCAN(const can::Message& msg)
    {
	int index = getBoardIdFromMessage(msg);
	
	//broadcast message is allways a controll message
	//and thuis does not update the internal state
	if(index == -1)
	    return false;
	
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
		states[index].error.encodersNotInitalized = edata->encodersNotInitalized;
		states[index].error.motorOverheated = edata->motorOverheated;
		states[index].error.overCurrent = edata->overCurrent;
		states[index].error.timeout = edata->timeout;
		states[index].temperature = edata->temperature;

		encoderIntern[index].setRawEncoderValue(edata->position);
		encoderExtern[index].setRawEncoderValue(edata->externalPosition);
		
		this->states[index].position = encoderIntern[index].getAbsolutPosition() * directions[index];
		this->states[index].positionExtern = encoderExtern[index].getAbsolutPosition() * directions[index];
	    }
	    break;
            case firmware::PACKET_ID_STATUS:
            {
		const firmware::statusData *data =
		    reinterpret_cast<const firmware::statusData *>(msg.data);

                this->states[index].index   = data->index;
                this->states[index].current = data->currentValue; // Current in [mA]
                this->states[index].pwm     = directions[index] * static_cast<float>(data->pwm) / 1800; // PWM in [-1; 1]
		//printf("Current PWM Level for %i is: %f (%i) Direction: %i\n",index,this->states[index].pwm,data->pwm,directions[index]);
		encoderIntern[index].setRawEncoderValue(data->position);
		encoderExtern[index].setRawEncoderValue(data->externalPosition);

		this->states[index].position = encoderIntern[index].getAbsolutPosition() * directions[index];
		this->states[index].positionExtern = encoderExtern[index].getAbsolutPosition() * directions[index];

		//FIXME, What is this ?
// 		this->states[index].delta = diff;
		
		//getting an status package is an implicit cleaner for all error states
	        bzero(&(this->states[index].error), sizeof(struct ErrorState));
                this->states[index].can_time = msg.can_time;
                break;
            }

            case firmware::PACKET_ID_SPEED_DEBUG:
                // To be implemented
                break;

            case firmware::PACKET_ID_POS_DEBUG:
                // To be implemented
                break;

            case firmware::PACKET_ID_PID_DEBUG_SPEED:
                // To be implemented
                break;

            case firmware::PACKET_ID_PID_DEBUG_POS:
                // To be implemented
                break;
	    default:
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

    can::Message Driver::emergencyShutdown() const
    {
        can::Message msg;

        bzero(&msg, sizeof(can::Message));

        msg.can_id = firmware::PACKET_ID_EMERGENCY_STOP;
        msg.size = 0;

        return msg;
    }

    can::Message Driver::setDriveMode(hbridge::BOARD_SET set, hbridge::DRIVE_MODE mode)
    {
        return setDriveMode(set, mode, mode, mode, mode);
    }

    can::Message Driver::setDriveMode(BOARD_SET set, DRIVE_MODE board1, DRIVE_MODE board2,
                                      DRIVE_MODE board3, DRIVE_MODE board4)
    {
        can::Message msg;

        bzero(&msg, sizeof(can::Message));

        firmware::setModeData *data =
            reinterpret_cast<firmware::setModeData *>(msg.data);

        data->board1Mode = (firmware::controllerModes)board1;
        data->board2Mode = (firmware::controllerModes)board2;
        data->board3Mode = (firmware::controllerModes)board3;
        data->board4Mode = (firmware::controllerModes)board4;
        current_modes[0] = board1;
        current_modes[1] = board2;
        current_modes[2] = board3;
        current_modes[3] = board4;

	switch(set) {
	    case BOARDS_14:
		msg.can_id = firmware::PACKET_ID_SET_MODE14;
		break;
	    case BOARDS_58:
		msg.can_id = firmware::PACKET_ID_SET_MODE58;
		break;
	}
        msg.size = sizeof(firmware::setModeData);

        return msg;
    }

    can::Message Driver::setTargetValues(BOARD_SET set, short int value1, short int value2,
                                         short int value3, short int value4) const
    {
        short int value_array[4] = { value1, value2, value3, value4 };
        return setTargetValues(set, value_array);
    }
    can::Message Driver::setTargetValues(BOARD_SET set, short int* targets) const
    {
        can::Message msg;

        bzero(&msg, sizeof(can::Message));

        firmware::setValueData *data =
            reinterpret_cast<firmware::setValueData *>(msg.data);

        short int* values = &(data->board1Value);
        for (int i = 0; i < BOARD_COUNT; ++i)
        {
            switch(current_modes[i])
            {
            case DM_SPEED:
            case DM_PWM:
                values[i] = directions[i] * targets[i];
                break;
            case DM_POSITION:
                values[i] = targets[i];
                if (directions[i] < 0)
                    values[i] = TICKS_PER_TURN - values[i];
                break;
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

    can::Message Driver::setSpeedPID(int board,
                                     double kp, double ki, double kd,
                                     double minMaxValue) const
    {
        can::Message msg;
        bzero(&msg, sizeof(can::Message));
        setPID(msg, kp, ki, kd, minMaxValue);
        msg.can_id = HBRIDGE_BOARD_ID(board) | firmware::PACKET_ID_SET_PID_SPEED;
        return msg;
    }

    can::Message Driver::setPositionPID(int board,
                                        double kp, double ki, double kd,
                                        double minMaxValue) const
    {
        can::Message msg;
        bzero(&msg, sizeof(can::Message));
        setPID(msg, kp, ki, kd, minMaxValue);
        msg.can_id = HBRIDGE_BOARD_ID(board) | firmware::PACKET_ID_SET_PID_POS;
        return msg;
    }

    MessagePair Driver::setConfiguration(int board,
                                         const Configuration &cfg) const
    {
        MessagePair msgs;
    
        firmware::configure1Data *cfg1 =
            reinterpret_cast<firmware::configure1Data *>(msgs.first.data);
        firmware::configure2Data *cfg2 =
            reinterpret_cast<firmware::configure2Data *>(msgs.second.data);
    
        cfg1->openCircuit                = cfg.openCircuit;
        cfg1->activeFieldCollapse        = cfg.activeFieldCollapse;
        cfg1->externalTempSensor         = cfg.externalTempSensor;
        cfg1->cascadedPositionController = cfg.cascadedPositionController;
        cfg1->enablePIDDebug             = cfg.pidDebugActive;
        cfg1->unused                     = 0;
        cfg1->maxMotorTemp               = cfg.maxMotorTemp;
        cfg1->maxMotorTempCount          = cfg.maxMotorTempCount;
        cfg1->maxBoardTemp               = cfg.maxBoardTemp;
        cfg1->maxBoardTempCount          = cfg.maxBoardTempCount;
        cfg1->timeout                    = cfg.timeout;
        
        cfg2->maxCurrent                 = cfg.maxCurrent;
        cfg2->maxCurrentCount            = cfg.maxCurrentCount;
        cfg2->pwmStepPerMs               = cfg.pwmStepPerMs;

        msgs.first.can_id = HBRIDGE_BOARD_ID(board) | firmware::PACKET_ID_SET_CONFIGURE;
        msgs.first.size = sizeof(firmware::configure1Data);

        msgs.second.can_id = HBRIDGE_BOARD_ID(board) | firmware::PACKET_ID_SET_CONFIGURE2;
        msgs.second.size = sizeof(firmware::configure2Data);

        return msgs;
    }

    can::Message Driver::setInternalEncoderConfiguration(int board, const EncoderConfiguration &cfg)
    {      
	can::Message msg = setEncoderConfiguration(board, cfg);

	msg.can_id = HBRIDGE_BOARD_ID(board) | firmware::PACKET_ID_ENCODER_CONFIG_INTERN;
        msg.size = sizeof(firmware::encoderConfiguration);

	encoderIntern[board].setWrapValue(cfg.ticksPerTurnDivided);
	encoderIntern[board].setZeroPosition(cfg.zeroPosition * directions[board]);
		
	return msg;
    }
    
    can::Message Driver::setExternalEncoderConfiguration(int board, const hbridge::EncoderConfiguration& cfg)
    {
	can::Message msg = setEncoderConfiguration(board, cfg);

	msg.can_id = HBRIDGE_BOARD_ID(board) | firmware::PACKET_ID_ENCODER_CONFIG_EXTERN;
        msg.size = sizeof(firmware::encoderConfiguration);

	encoderExtern[board].setWrapValue(cfg.ticksPerTurnDivided);
	encoderExtern[board].setZeroPosition(cfg.zeroPosition * directions[board]);
		
	return msg;
    }

    
    can::Message Driver::Driver::setEncoderConfiguration(int board, const hbridge::EncoderConfiguration& cfg)
    {
	can::Message msg;
        bzero(&msg, sizeof(can::Message));
	
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
    
    void Driver::setPID(can::Message &msg,
                       double kp, double ki, double kd,
                       double minMaxValue) const
    {
        firmware::setPidData *data = reinterpret_cast<firmware::setPidData *>(msg.data);
        data->kp = kp;
        data->ki = ki;
        data->kd = kd;
        data->minMaxPidOutput = minMaxValue;
        
        msg.size = sizeof(firmware::setPidData);
    }
}

