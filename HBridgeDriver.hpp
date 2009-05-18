#ifndef HBRIDGE_DRIVER_HPP
#define HBRIDGE_DRIVER_HPP

#ifndef __orogen
#include <utility>
#include <canmessage.hh>
#include <stdint.h>
// Move all the firmware stuff to its own namespace
#endif /* __orogen */

namespace hbridge
{
#ifndef __orogen
    const int TICKS_PER_TURN = 512 * 729 / 16;
#endif /* __orogen */

    typedef int64_t Ticks;

    struct Configuration
    {
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
         * Initialise all fields of the configuration structure with 0
         */
        Configuration() :
            openCircuit(0), activeFieldCollapse(0), externalTempSensor(0),
            cascadedPositionController(0), pidDebugActive(0), maxMotorTemp(0),
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

#define HBRIDGE_BOARD_ID(x) ((hbridge::BOARDID)(x << 5))

    /**
     * Board (hbridge) id constants
     */
    enum BOARDID
    {
        H_BROADCAST = 0,
        H_BRIDGE1 = (1 << 5),
        H_BRIDGE2 = (2 << 5),
        H_BRIDGE3 = (3 << 5),
        H_BRIDGE4 = (4 << 5)
    };

    enum ERROR_CODES {
        ERROR_CODE_NONE = 0,
        ERROR_CODE_OVERHEATMOTOR = 1,
        ERROR_CODE_OVERHEATBOARD = 2,
        ERROR_CODE_OVERCURRENT = 3,
        ERROR_CODE_TIMEOUT = 4,
        ERROR_CODE_BAD_CONFIG = 5
    };

    /**
     * Drive mode constants for the hbridges
     */
    enum DRIVE_MODE
    {
        DM_PWM = 0,
        DM_SPEED = 1,
        DM_POSITION = 2,
        DM_UNINITIALIZED = 3
    };

    struct BoardState
    {
        int index;
        int current;
        Ticks position;
        int delta;
        int error;
        float pwm;
    };

#ifndef __orogen
    typedef std::pair<can::Message, can::Message> MessagePair;

    const int BOARD_COUNT = 4;

    class Driver
    {
    protected:

        BoardState states[BOARD_COUNT];
        int16_t positionOld[BOARD_COUNT];

    public:
        Driver();
        ~Driver();

        /**
         * Updates the internal state from a CAN message
         *
         * @param msg A newly received CAN message
         *
         * @return True if the message was processed successfully, false otherwise.
         */
        bool updateFromCAN(can::Message &msg);

        /**
         * Query the board state (current [mA], position [ticks], errors) for the given board (hbridge)
         *
         * @param board hbridge identifier
         *
         * @return The current state for the given board
         */
        const BoardState &getState(BOARDID board) const;

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

#endif /* HBRIDGE_DRIVER_HPP */

