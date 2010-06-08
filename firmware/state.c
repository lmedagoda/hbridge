#include "state.h"
#include "printf.h"

volatile struct ControllerState state1;
volatile struct ControllerState state2;

volatile struct ControllerState *activeCState = &state1;
volatile struct ControllerState *lastActiveCState = &state2;

volatile u8 errorState = 0;

//motorticks * gear reduction
//512 * 729 / 16
#define HALF_WHEEL_TURN_TICKS (23328 * 2)

void initStateStruct(volatile struct ControllerState *cs) {
    //init cotroller state with sane values
    cs->controllMode = CONTROLLER_MODE_PWM;
    cs->internalState = STATE_UNCONFIGURED;
    cs->useBackInduction = 0;
    cs->useOpenLoop = 0;
    cs->cascadedPositionController = 0;
    cs->pwmStepPerMillisecond = 0;
    cs->maxCurrent = 0;
    cs->maxMotorTemp = 0;
    cs->maxMotorTempCount = 0;
    cs->maxBoardTemp = 0;
    cs->maxBoardTempCount = 0;
    cs->timeout = 1;
    cs->targetValue = 0;
    cs->speedPIDValues.kp = 0;
    cs->speedPIDValues.ki = 0;
    cs->speedPIDValues.kd = 0;
    cs->speedPIDValues.minMaxPidOutput = 0;
    cs->positionPIDValues.kp = 0;
    cs->positionPIDValues.ki = 0;
    cs->positionPIDValues.kd = 0;
    cs->positionPIDValues.minMaxPidOutput = 0;
    cs->enablePIDDebug = 0;
    cs->ticksPerTurn = HALF_WHEEL_TURN_TICKS * 2;
    
    errorState = 0;
};

void printStateDebug(volatile struct ControllerState* cs)
{
    char *ctrl_s = "";
    char *int_state_s = "";
    switch(cs->controllMode) {
	case CONTROLLER_MODE_PWM:
	    ctrl_s = "PWM";
	    break;
	case CONTROLLER_MODE_POSITION:
	    ctrl_s = "POSITION";
	    break;
	case CONTROLLER_MODE_SPEED:
	    ctrl_s = "SPEED";
	    break;
    }
    switch(cs->internalState) {
	case STATE_CONFIG1_RECEIVED:
	    int_state_s = "CONF1";
	    break;
	case STATE_CONFIG2_RECEIVED:
	    int_state_s = "CONF2";
	    break;
	case STATE_CONFIGURED:
	    int_state_s = "CONFIGURED";
	    break;
	case STATE_ERROR:
	    int_state_s = "ERROR";
	    break;
	case STATE_GOT_TARGET_VAL:
	    int_state_s = "GOT_TARGET";
	    break;
	case STATE_UNCONFIGURED:
	    int_state_s = "UNCONFIGURED";
	    break;
    }
    printf("ControllMode: %s ,internal State: %s ,targetVal : %li ,openloop:%hi ,backIndo %hi ,pwmstep %hu \n", ctrl_s, int_state_s, cs->targetValue, cs->useOpenLoop, cs->useBackInduction, cs->pwmStepPerMillisecond);    
}

u8 inErrorState() {
    return errorState;
}

struct ErrorState *getErrorState() {
    return (struct errorState *) (&errorState);
}

void clearErrors() {
    errorState = 0;
}


