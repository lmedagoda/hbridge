#ifndef HBRIDGE_PROTOCOL_H
#define HBRIDGE_PROTOCOL_H

#ifndef __orogen

#include <utility>

#include <canmessage.hh>
#include <stdint.h>

// Move all the firmware stuff to its own namespace
namespace firmware
{
#define __NO_STM32

    //define types similar to STM32 firmeware lib,
    //so we can use same protocoll.h
    typedef uint8_t u8;
    typedef uint16_t u16;
    typedef int16_t s16;

#include "firmware/protocol.h"
}
#endif /* __orogen */

namespace hbridge
{
#ifndef __orogen
    const int TICKS_PER_TURN = 512 * 729 / 16;
#endif /* __orogen */

    struct AbsolutePosition
    {
        int turns;
        int ticks;

#ifndef __orogen
        /**
         * Only initialises the member variables to 0
         */
        AbsolutePosition() :
            turns(0), ticks(0)
        {}
      
        /**
         * Addition operator adds a given number of ticks to the current position.
         *
         * @param val Ticks to be added
         *
         * @return An updated AbsolutePosition structure instance
         */
        AbsolutePosition operator +(const int &val) const
        {
            AbsolutePosition ret = *this;
            ret.ticks += val;
            ret.canonize();
            return ret;
        }

        /**
         * Subtraction operator subtracts a given number of ticks to the
         * current position.
         *
         * @param val Ticks to be subtracted
         *
         * @return An updated AbsolutePosition structure instance
         */
        AbsolutePosition operator -(const int &val) const
        {
            AbsolutePosition ret = *this;
            ret.ticks -= val;
            ret.canonize();
            return ret;
        }
      
        /**
         * Comparison operator to compare two AbsolutePosition structures
         *
         * @param pos Another AbsolutePosition structure
         *
         * @return Returns true if the current instance is smaller, false
         * otherwise.
         */
        bool operator <(const AbsolutePosition &pos) const
        {
            return ((turns < pos.turns) ||
                    ((turns == pos.turns) && (ticks < pos.ticks)));
        }

        /**
         * Comparison operator to compare two AbsolutePosition structures
         *
         * @param pos Another AbsolutePosition structure
         *
         * @return Returns true if the current instance is greater, false
         * otherwise.
         */
        bool operator >(const AbsolutePosition &pos) const
        {
            return ((turns > pos.turns) ||
                    ((turns == pos.turns) && (ticks > pos.ticks)));
        }

        /**
         * Equality operator to compare two AbsolutePosition structures
         *
         * @param pos Another AbsolutePosition structure
         *
         * @return Returns true if both structures are equals, false otherwise.
         */
        bool operator ==(const AbsolutePosition &pos) const
        {
            return ((turns == pos.turns) && (ticks == pos.turns));
        }

        /**
         * Returns the number of ticks instead of turns and ticks
         *
         * @return The absolute number of ticks
         */
        uint64_t getTicks() const
        {
            return ((uint64_t) this->turns) * TICKS_PER_TURN + this->ticks;
        }

        /**
         * Fixes the ticks and turns values if required (Ticks >! 0).
         */
        inline void canonize()
        {
            int offset = (ticks < 0 ?
                          ticks / TICKS_PER_TURN - 1 :
                          ticks / TICKS_PER_TURN);
            ticks -= (offset * TICKS_PER_TURN);
            turns += offset;
        }
#endif /* __orogen */
    };

    struct Configuration {
        unsigned char openCircuit;
        unsigned char activeFieldCollapse;
        unsigned char externalTempSensor;
        unsigned char cascadedPositionController;
        unsigned char pidDebugActive;
        unsigned char maxMotorTemp;
        unsigned char maxMotorTempCount;
        unsigned char maxBoardTemp;
        unsigned char maxBoardTempCount;
        unsigned short timeout;
        unsigned short maxCurrent;
        unsigned char maxCurrentCount;
        unsigned short pwmStepPerMs;
        
#ifndef __orogen
        /**
         * Initialise all firlds of the configuration structure with 0
         */
        Configuration() :
            openCircuit(0), activeFieldCollapse(0), externalTempSensor(0),
            cascadedPositionController(0), maxMotorTemp(0),
            maxMotorTempCount(0), maxBoardTemp(0), maxBoardTempCount(0),
            timeout(0), maxCurrent(0), maxCurrentCount(0), pwmStepPerMs(0)
        {}
  
        /**
         * Equality operator.
         *
         * @param val Another configuration structure
         *
         * @return Returns true if the fields of both structures are equal,
         * false otherwise.
         */
        bool operator ==(const Configuration &val) const
        {
            return ((openCircuit == val.openCircuit) &&
                    (activeFieldCollapse == val.activeFieldCollapse) &&
                    (externalTempSensor == val.externalTempSensor) &&
                    (cascadedPositionController == val.cascadedPositionController) &&
                    (maxMotorTemp == val.maxMotorTemp) &&
                    (maxMotorTempCount == val.maxMotorTempCount) &&
                    (maxBoardTemp == val.maxBoardTemp) &&
                    (maxBoardTempCount == val.maxBoardTempCount) &&
                    (timeout == val.timeout) &&
                    (maxCurrent == val.maxCurrent) &&
                    (maxCurrentCount == val.maxCurrentCount) &&
                    (pwmStepPerMs == val.pwmStepPerMs));
        }

        /**
         * Inequality operator.
         *
         * @param val Another configuration structure
         *
         * @return Returns the opposite of the equality operator.
         */
        bool operator !=(const Configuration &val) const
        {
            return !(*this == val);
        }
#endif
    };

    /**
     * Board (hbridge) id constants
     */
    enum BOARDID
    {
        H_BRIDGE1 = (1 << 5),
        H_BRIDGE2 = (2 << 5),
        H_BRIDGE3 = (3 << 5),
        H_BRIDGE4 = (4 << 5)
    };

    /**
     * Drive mode constants for the hbridges
     */
    enum DRIVE_MODE
    {
        DM_PWM = 0,
        DM_SPEED = 1,
        DM_POSITION = 2
    };

    struct BoardState
    {
        int current;
        int position;
        int positionOld;
        firmware::errorCodes error;
    };

#ifndef __orogen
    class Protocol
    {
    protected:
        static const int BOARD_COUNT = 4;

        int                  current[BOARD_COUNT];
        int                  oldPosition[BOARD_COUNT];
        AbsolutePosition     absPosition[BOARD_COUNT];
        firmware::errorCodes error[BOARD_COUNT];

    public:
        Protocol();
        ~Protocol();

        /**
         * Updates the internal state from a CAN message
         *
         * @param msg A newly received CAN message
         *
         * @return True if the message was processed successfully, false otherwise.
         */
        bool updateFromCAN(can::Message &msg);

        /**
         * Query the current [mA] for the given board (hbridge)
         *
         * @param board hbridge identifier
         *
         * @return The current in [mA] for the given board
         */
        int getCurrent(BOARDID board);

        /**
         * Query the absolute position [turns,ticks] for the given board (hbridge)
         *
         * @param board hbridge identifier
         *
         * @return The absolute position [turns,ticks] for the given board
         */
        const AbsolutePosition &getAbsolutePosition(BOARDID board);

        /**
         * Query the error code for the given board (hbridge)
         *
         * @param board hbridge identifier
         *
         * @return The error code for the given board
         */
        firmware::errorCodes getErrorCode(BOARDID board);

        /**
         * Generate a CAN message for emergency shutdown
         *
         * @return A new CAN message (PACKET_ID_EMERGENCY_STOP)
         */
        can::Message emergencyShutdown() const;

        /**
         * Generate a CAN message for setting the drive mode for all
         * hbridges at once.
         *
         * @param mode Drive mode for all hbridges
         *
         * @return A new CAN message (PACKET_ID_SET_MODE)
         */
        can::Message setDriveMode(DRIVE_MODE mode) const;

        /**
         * Generate a CAN message for setting the drive mode fpr every
         * hbridge.
         *
         * @param board1 Drive mode for board 1
         * @param board2 Drive mode for board 2
         * @param board3 Drive mode for board 3
         * @param board4 Drive mode for board 4
         *
         * @return A new CAN message (PACKET_ID_SET_MODE)
         */
        can::Message setDriveMode(DRIVE_MODE board1, DRIVE_MODE board2,
                             DRIVE_MODE board3, DRIVE_MODE board4) const;

        /**
         * Generate a CAN message for setting new motor values. Interpretation
         * of values depends on the motor mode (PWM, Speed, Position)
         *
         * PWM
         * - Value Range:   [-1800, 1800]
         * - Physical Unit: %
         *
         * Speed
         * - Value Range:   ??
         * - Physical Unit: ticks/ms
         *
         * Position
         * - Value Range:   [0, 23328] (23328 = 512 * 729 / 16)
         * - Physical Unit: P° where P = 64.8° (((512 * 729 / 16) / 360)°)
         *
         * @param value1 [PWM,Speed,Position] value of motor 1
         * @param value2 [PWM,Speed,Position] value of motor 2
         * @param value3 [PWM,Speed,Position] value of motor 3
         * @param value4 [PWM,Speed,Position] value of motor 4
         *
         * @return A new CAN message (PACKET_ID_SET_NEW_VALUE)
         */
        can::Message setTargetValues(short int value1, short int value2,
                                     short int value3, short int value4) const;

        /**
         * Generate a CAN message for setting the PID values for the speed
         * controller of a given hbridge.
         *
         * @param board hbridge identifier
         * @param kp Proportial term parameter
         * @param ki Integral term parameter
         * @param kd Differential term parameter
         * @param minMaxValue Min-Max value
         *
         * @return A new CAN message (PACKET_ID_SET_PID_POS)
         */
        can::Message setSpeedPID(BOARDID board,
                            double kp, double ki, double kd,
                            double minMaxValue) const;

        /**
         * Generate a CAN message for setting the PID values for the position
         * controller of a given hbridge.
         *
         * @param board hbridge identifier
         * @param kp Proportial term parameter
         * @param ki Integral term parameter
         * @param kd Differential term parameter
         * @param minMaxValue Min-Max value
         *
         * @return A new CAN message (PACKET_ID_SET_PID_SPEED)
         */
        can::Message setPositionPID(BOARDID board,
                               double kp, double ki, double kd,
                               double minMaxValue) const;

        typedef std::pair<can::Message, can::Message> MessagePair;
        /**
         * Generate a CAN message for setting the configuration of a given hbridge.
         *
         * @param board hbridge identifier
         * @param cfg Configuration values for the given hbridge
         *
         * @return Two new CAN messages (PACKET_ID_CONFIGURE, PACKET_ID_CONFIGURE2)
         */
        MessagePair setConfiguration(BOARDID board,
                                     const Configuration &cfg) const;

    protected:
        /**
         * Generate a CAN message for PID values but with no id set.
         *
         * @param msg A reference to a can::Message structure
         * @param kp Proportial term parameter
         * @param ki Integral term parameter
         * @param kd Differential term parameter
         * @param minMaxValue Min-Max value
         */
        inline void setPID(can::Message &msg,
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

    };
#endif /* __orogen */

}

#endif /* HBRIDGE_PROTOCOL_H */

