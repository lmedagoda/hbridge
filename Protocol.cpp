#include "Protocol.hpp"
#include <strings.h>
#include <stdlib.h>

namespace hbridge
{

    Protocol::Protocol() :
        current(), oldPosition(), absPosition(), error()
    {
        bzero(&this->current, BOARD_COUNT * sizeof(int));
        bzero(&this->oldPosition, BOARD_COUNT * sizeof(int));
        bzero(&this->error, BOARD_COUNT * sizeof(firmware::errorCodes));
    }

    Protocol::~Protocol()
    {
    }

    bool Protocol::updateFromCAN(can::Message &msg)
    {
        firmware::statusData *data =
            reinterpret_cast<firmware::statusData *>(msg.data);

        int index = ((msg.can_id & ~0x1f) >> 5) - 1;

        if ((index < 0) || (index > (BOARD_COUNT - 1)))
        {
            // Invalid id specified in packet
            return false;
        }

        this->current[index] = data->currentValue; // Current in [mA]

        int diff = (data->position - this->oldPosition[index]);

        // We assume that a motor rotates less than half a turn per [ms]
        // (a status packet is sent every [ms])
        if (abs(diff) > TICKS_PER_TURN / 2)
        {
            diff += (diff < 0 ? 1 : -1) * TICKS_PER_TURN;
        }

        // Track the position
        this->absPosition[index] = this->absPosition[index] + diff;

        // Don't know what to do with the PWM value
        // this->pwm[index] = data->pwm;

        this->error[index] = data->error;

        return true;
    }

    int Protocol::getCurrent(BOARDID board)
    {
        return this->current[(int)board >> 5];
    }

    const AbsolutePosition &Protocol::getAbsolutePosition(BOARDID board)
    {
        return this->absPosition[(int)board >> 5];
    }

    firmware::errorCodes Protocol::getErrorCode(BOARDID board)
    {
        return this->error[(int)board >> 5];
    }

    can::Message Protocol::emergencyShutdown() const
    {
        can::Message msg;

        bzero(&msg, sizeof(can::Message));

        msg.can_id = firmware::PACKET_ID_EMERGENCY_STOP;
        msg.size = 0;

        return msg;
    }

    can::Message Protocol::setDriveMode(DRIVE_MODE mode) const
    {
        return setDriveMode(mode, mode, mode, mode);
    }

    can::Message Protocol::setDriveMode(DRIVE_MODE board1, DRIVE_MODE board2,
                                        DRIVE_MODE board3, DRIVE_MODE board4) const
    {
        can::Message msg;

        bzero(&msg, sizeof(can::Message));

        firmware::setModeData *data =
            reinterpret_cast<firmware::setModeData *>(msg.data);

        data->board1Mode = (firmware::controllerModes)board1;
        data->board2Mode = (firmware::controllerModes)board2;
        data->board3Mode = (firmware::controllerModes)board3;
        data->board4Mode = (firmware::controllerModes)board4;

        msg.can_id = firmware::PACKET_ID_SET_MODE;
        msg.size = sizeof(firmware::setModeData);

        return msg;
    }

    can::Message Protocol::setTargetValues(short int value1, short int value2,
                                           short int value3, short int value4) const
    {
        can::Message msg;

        bzero(&msg, sizeof(can::Message));

        firmware::setValueData *data =
            reinterpret_cast<firmware::setValueData *>(msg.data);

        data->board1Value = value1;
        data->board2Value = value2;
        data->board3Value = value3;
        data->board4Value = value4;

        msg.can_id = firmware::PACKET_ID_SET_VALUE;
        msg.size = sizeof(firmware::setValueData);

        return msg;
    }

    can::Message Protocol::setSpeedPID(BOARDID board,
                                       double kp, double ki, double kd,
                                       double minMaxValue) const
    {
        can::Message msg;

        bzero(&msg, sizeof(can::Message));

        setPID(msg, kp, ki, kd, minMaxValue);

        msg.can_id = board | firmware::PACKET_ID_SET_PID_SPEED;

        return msg;
    }

    can::Message Protocol::setPositionPID(BOARDID board,
                                          double kp, double ki, double kd,
                                          double minMaxValue) const
    {
        can::Message msg;

        bzero(&msg, sizeof(can::Message));

        setPID(msg, kp, ki, kd, minMaxValue);

        msg.can_id = board | firmware::PACKET_ID_SET_PID_POS;

        return msg;
    }

    Protocol::MessagePair Protocol::setConfiguration(BOARDID board,
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

        msgs.first.can_id = board | firmware::PACKET_ID_SET_CONFIGURE;
        msgs.first.size = sizeof(firmware::configure1Data);

        msgs.second.can_id = board | firmware::PACKET_ID_SET_CONFIGURE2;
        msgs.second.size = sizeof(firmware::configure2Data);

        return msgs;
    }

}

