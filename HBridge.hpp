#ifndef HBRIDGE_HPP
#define HBRIDGE_HPP

#ifndef __orogen
#include <stdint.h>
#endif

#include <base/time.h>

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
#endif
    };

    enum MOTOR_ID
    {
        MOTOR_REAR_LEFT  = 0,
        MOTOR_REAR_RIGHT = 1,
        MOTOR_FRONT_RIGHT = 2,
        MOTOR_FRONT_LEFT = 3
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
        base::Time can_time;
    };
}

#endif /* HBRIDGE_DRIVER_HPP */

