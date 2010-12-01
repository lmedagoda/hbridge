#ifndef HBRIDGE_DRIVER_HPP
#define HBRIDGE_DRIVER_HPP

#include <utility>
#include <canmessage.hh>
#include <stdint.h>

#include "HBridge.hpp"

namespace hbridge
{
    typedef std::pair<canbus::Message, canbus::Message> MessagePair;

    enum BOARD_SET {
	BOARDS_14,
	BOARDS_58,	    
    };

    class Encoder
    {
	private:
	    unsigned int wrapValue;
	    Ticks zeroPosition;
	    unsigned int positionInTurn;
	    unsigned int lastPositionInTurn;
	    int turns;
	    bool gotValidReading;
	public:
	    Encoder();
	    void setWrapValue(unsigned int value);
	    void setZeroPosition(Ticks zeroPos);
	    void setRawEncoderValue(unsigned int value);
	    Ticks getAbsolutPosition();
    };

    class Driver
    {
    public:
    protected:

        BoardState states[BOARD_COUNT];
	EncoderConfiguration encoderConfigurations[BOARD_COUNT];
        hbridge::DRIVE_MODE current_modes[BOARD_COUNT];
	
	Encoder encoderIntern[BOARD_COUNT];
	Encoder encoderExtern[BOARD_COUNT];
    public:
	
        Driver();
        ~Driver();

	/**
	* Extracts and returns the board id from the given message.
	* Note, this method does not perform range checks.
	**/
	int getBoardIdFromMessage(const canbus::Message &msg) const;
	
        /**
         * Resets the internal state of the driver. This has to be called upon
         * reconfiguration of the h-bridges
         */
        void reset();

        /**
         * Updates the internal state from a CAN message
         *
         * @param msg A newly received CAN message
         *
         * @return True if the message updated the internal state. False if it did not.
	 * @throw std::invalid_argument if a protocoll error was detected 
         */
        bool updateFromCAN(const canbus::Message &msg);

        /**
         * Query the board state (current [mA], position [ticks], errors) for the given board (hbridge)
         *
         * @param board hbridge identifier
         *
         * @return The current state for the given board
         */
        const BoardState &getState(int board) const;

        /**
         * Generate a CAN message for emergency shutdown
         *
         * @return A new CAN message (PACKET_ID_EMERGENCY_STOP)
         */
        canbus::Message emergencyShutdown() const;

        /**
         * Generate a CAN message for setting the drive mode for all
         * hbridges at once.
         *
         * @param mode Drive mode for all hbridges
         *
         * @return A new CAN message (PACKET_ID_SET_MODE)
         */
        canbus::Message setDriveMode(BOARD_SET set, DRIVE_MODE mode);

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
        canbus::Message setDriveMode(BOARD_SET set ,DRIVE_MODE board1, DRIVE_MODE board2,
                             DRIVE_MODE board3, DRIVE_MODE board4);

        /**
         * Generate a CAN message for setting new motor values. Interpretation
         * of values depends on the motor mode (PWM, Speed, Position)
         *
         * PWM
         * - Value Range:   [-1800, 1800]
         * - Physical Unit: %
         *
         * Speed
         * - Value Range:   limited by the motor. On 12V Faulhaber motors, it is [-400, 400].
         * - Physical Unit: (quarter of a tick)/ms
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
        canbus::Message setTargetValues(BOARD_SET set, short int* targets) const;

        /**
         * \overloaded
         */
        canbus::Message setTargetValues(BOARD_SET set, short int value1, short int value2,
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
        canbus::Message setSpeedPID(int board,
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
        canbus::Message setPositionPID(int board,
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
        MessagePair setConfiguration(int board,
                                     const Configuration &cfg) const;

	/**
	* Generates a CAN message for configuraing encoder parameters.
	* This need to be done only once and in an UNCONFIGURED state.
	*
	* @param board hbridge identifier
	* @param cfg encoder configuration for the given hbridge
	*
	* @return A new CAN message (PACKET_ID_ENCODER_CONFIG)
	**/
	canbus::Message setEncoderConfiguration(int board, const hbridge::EncoderConfiguration& interncfg, const hbridge::EncoderConfiguration& externcfg);
    protected:
        /**
         * Generate a CAN message for PID values but with no id set.
         *
         * @param msg A reference to a canbus::Message structure
         * @param kp Proportial term parameter
         * @param ki Integral term parameter
         * @param kd Differential term parameter
         * @param minMaxValue Min-Max value
         */
        void setPID(canbus::Message &msg,
                           double kp, double ki, double kd,
                           double minMaxValue) const;

    };
}

#endif /* HBRIDGE_DRIVER_HPP */

